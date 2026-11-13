# Polyglot architecture

Polyglot is a protocol emulator that is programmed in plain C. Protocol knowledge lives in software and in one header, `protocol.h`; the hardware only provides generic, cycle-exact ways to move bits. This document describes the hardware. Every area or timing number is tagged with how it was obtained: **synthesis** (Yosys with the CMOS5L typical library, before place and route), **P&R** (LibreLane place and route, sign-off STA), or **simulation**.

## 1. Block diagram

```
  host MCU (RP2350 on the demo board)
      |  SPI target: load program, read/write memory, run/halt     HOST_IRQ
      v                                                              ^
 +----+--------------------------------------------------------------+----+
 | pg_host_spi --+                                                       |
 |               |   +-----------+  AHB-Lite   +-------------------+     |
 |  SRAM 4 KB <--+-->| pg_ahb    |<----------->| Hazard3 RV32EC    |     |
 |  (IHP macro)      | fabric    |             | (C, stock GCC)    |     |
 |                   +-----+-----+             +-------------------+     |
 |                         | register port (wait states = blocking I/O) |
 |   +---------------------+---------------------------------------+     |
 |   | pg_io: decode, input synchronizers, blocking timeout, pinmux |     |
 |   |   pg_sysio: NOW counter, GPIO, timed GPIO queue, capture     |     |
 |   |   pg_wire x2: Wire Engines (serial, line codes, CRC, ...)    |     |
 |   +-------------------------------+------------------------------+     |
 +-----------------------------------|------------------------------------+
                                     v
                ui[4:0] inputs, uo[5:0] outputs, uio[7:0] bidirectional
```

| Block | Source | Role |
|---|---|---|
| CPU | `src/hazard3_*` | Hazard3 by Luke Wren (the RP2350 core), RV32EC with M-mode traps, vendored with one marked change (the never-written x0 register-file entry is cleared during reset, so gate-level simulation has no X from it). Runs C compiled with stock GCC or Clang. |
| Memory | `pg_sram.v` | One IHP `RM_IHPSG13_1P_1024x32_c2_bm_bist` macro: 4 KB for code, data and stack. |
| Fabric | `pg_ahb.v` | AHB-Lite: SRAM with zero read wait states; register window at 0x8000_0000 (one wait state per access: read data and ready are registered, which keeps the I/O decode off the CPU's critical path); host port that uses idle SRAM cycles. |
| Host port | `pg_host_spi.v` | SPI target (mode 0): WRITE/READ memory, CTRL (run), STATUS. |
| I/O | `pg_io.v` | Address decode, two-flop input synchronizers, the blocking-access timeout, and the pinmux. |
| System | `pg_sysio.v` | Cycle counter, GPIO, timed GPIO, input edge capture, host mailbox and IRQ. |
| Wire Engines | `pg_wire.v`, `pg_fifo.v` | Two protocol-agnostic serial engines (section 4). |

## 2. Programming model

- Memory map: SRAM at 0x0000_0000 (4 KB; the last 256 bytes are shared with the host), system registers at 0x8000_0000, Wire Engine *n* at 0x8000_0100 + 0x100·*n*.
- Toolchain: `riscv-none-elf-gcc -march=rv32ec -mabi=ilp32e`, `sw/rt/crt0.S`, `sw/rt/link.ld`. No custom assembler or compiler.
- **Blocking I/O**: reading an empty RX FIFO, writing a full TX FIFO, waiting for an edge capture or for the timed-GPIO queue to drain all simply stall the CPU's bus access (AHB wait states) until the hardware is ready. C code reads like ordinary sequential code (`uart_getc()` waits for a byte) and the CPU uses no power-hungry polling loop. A global timeout (`BLK_TIMEOUT`) bounds every wait.
- **Several sources at once**: `wait_events(mask)` sleeps until any selected engine has received data (or an edge was captured) and returns which ones; one C loop serves several protocols (the USB keyboard example also listens on a UART).
- **Time**: `now()` is a 32-bit cycle counter. Everything that must happen at an exact time is done by hardware that takes a time stamp from C.

## 3. Timed GPIO and capture

`pin_out_at(pin, value, t)` queues a change (4 deep) that the hardware applies in exactly the cycle in which `now() == t`; `pins_out_at()` changes several pins in the same cycle, `pin_oe_at()` switches a driver (open drain). C only has to stay ahead of time; how long the compiled code takes does not matter. A change whose time has passed is applied at once and sets a LATE flag. Edge capture time-stamps an input edge to the cycle (the synchronizer delay is compensated).

This is the universal fallback for protocols no engine fits: the 1-Wire master in `sw/examples/onewire` uses nothing else. *(simulation: edges exactly on the scheduled cycle, including changes one cycle apart; capture error 0 cycles)*

## 4. Wire Engines

A Wire Engine is a serial engine with no protocol in it. Protocols are parameter sets and a few lines of C in `protocol.h`.

```
 TX FIFO (4 x {attributes, length, 32-bit data})
   -> shift register (1..32 bits, LSB or MSB first)
   -> [bit stuffing: insert after N equal bits / N ones]
   -> [CRC unit 1..32 bits: append after a word]
   -> line code: NRZ / NRZI / Manchester
   -> pin driver: push-pull / open drain / driven only in a frame / differential pair (+SE0)
 RX pin -> [sync to bit clock | async start edge + bit timer + edge resync | external clock]
   -> line decode (NRZI) -> destuffing (+error flag) -> CRC residual
   -> shift register -> RX FIFO (words, plus frame start/end markers)
 bit clock: internal divider with 1/256-cycle fraction, or edges on a pin; SCK output for SPI modes 0-3
```

| Mechanism | Used by |
|---|---|
| Fractional bit clock (DIV + FRAC/256), RX edge resync | UART at any rate (3 Mbaud at 50 MHz), USB LS (33.33 cycles/bit), WS2812 |
| Internal clock with SCK output (CPOL/CPHA), RX sample delay | SPI controller, modes 0-3, up to F/4 |
| External clock, chip-select gate, frame events | SPI target |
| Open-drain data and clock, clock stretching both ways, START/STOP gate, late data launch | I2C controller and target |
| Per-word length and attributes (`TXATTR`): no-stuff, no-CRC, CRC init/append, SE0, send-after-RX | USB, Ethernet, I2C ACK slots |
| NRZI, stuffing, CRC16, differential pair with SE0, packet RX until SE0 | USB low speed |
| Auto-reply: a queued answer leaves TURN cycles after the end of a received packet that matches RXMATCH | USB handshakes and IN data within the 7.5-bit-time limit |
| Manchester, CRC-32, idle tail (TP_IDL) | 10BASE-T transmit |
| Arbitration by read-back (open drain or a separate RX pin), stuffing after five equal bits, CRC-15, end of frame by an idle run | CAN 2.0A |
| Partial last word at a packet end (bit count in the end marker) | CAN frames of any length |

The engine never decides *what* to send; it only decides *when* each bit moves. For example, the USB device in `sw/examples/usb_hid` is ordinary C, yet every answer starts 3.1 bit times after the host's packet, because C queues the answer early and the engine releases it at the right moment *(simulation)*.

## 5. Pins

| Pins | Use |
|---|---|
| ui[4:0] | protocol inputs (pin numbers 16..20 in `protocol.h`) |
| ui[7:5] | host SPI: SCK, MOSI, CS_N |
| uo[5:0] | protocol outputs (8..13) |
| uo[6] | HOST_IRQ (set by firmware) |
| uo[7] | host SPI MISO |
| uio[7:0] | bidirectional protocol pins (0..7) |

Any engine can own any output pin; pins an engine does not own follow GPIO (and timed GPIO). Inputs from all pins (and a read-back of the outputs) go through two-flop synchronizers.

## 6. Implementation results

| Item | Value | Evidence |
|---|---|---|
| Clock | 50 MHz rated (slow corner 1.08 V / 125 C); typical-corner slack suggests 60 MHz or more at room temperature | P&R |
| Setup slack at 50 MHz | +2.61 ns slow corner, +7.87 ns typical; hold +0.10 ns fast corner | P&R (2026-11-12) |
| Critical path | SRAM clock-to-output (7.75 ns slow corner) into the CPU fetch/decode | P&R |
| Cell area | 26,657 cells, 389,589 um^2 after synthesis (flat; 41 % of it flip-flops) | synthesis |
| Std-cell utilization | 44.6 % of the 8x4 tile (51.0 % with the SRAM) | P&R |
| Sign-off | LVS clean, KLayout DRC clean, antenna clean, Tiny Tapeout precheck passes | P&R |

The SRAM macro needs special care on the CMOS5L tile, whose power grid is a single layer of Metal4 stripes: every stripe must run the full tile height, and a stripe may only cross the macro on one of the macro's own Metal4 supply columns of the same net. A LibreLane step in this repository (`librelane_plugin_polyglot.py`, `flow/odb_sram_stripes.py`) moves the stripes that cross the macro onto supply columns and connects them to the standard-cell rails. Routing: the logic above the macro talks to the logic on its right only through Metal3 (the one horizontal signal layer), so the corner is the congestion hot spot; a 6 um cell-free halo around the macro and a 55 % placement density cut global-routing overflow from 642 to 143, after which detailed routing converges to zero violations. The approach follows Ken Pettit's IHP SRAM write-up; the code is our own.

## 7. Credits

Hazard3 (Luke Wren, Apache-2.0); IHP Open PDK and SRAM macros; Tiny Tapeout template and tools; LibreLane; Ken Pettit's IHP SRAM macro walkthrough for the power-grid approach.
