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
 |  SRAM 8 KB <--+-->| pg_ahb    |<----------->| Hazard3 RV32EC    |     |
 |  (IHP macro)      | fabric    |             | (C, stock GCC)    |     |
 |                   +-----+-----+             +-------------------+     |
 |                         | register port (wait states = blocking I/O) |
 |   +---------------------+---------------------------------------+     |
 |   | pg_io: decode, input synchronizers, blocking timeout, pinmux |     |
 |   |   pg_sysio: NOW counter, GPIO, timed GPIO queue, capture     |     |
 |   |   pg_wire x4: Wire Engines (serial, line codes, CRC, ...)    |     |
 |   +-------------------------------+------------------------------+     |
 +-----------------------------------|------------------------------------+
                                     v
                ui[4:0] inputs, uo[5:0] outputs, uio[7:0] bidirectional
```

| Block | Source | Role |
|---|---|---|
| CPU | `src/hazard3_*` | Hazard3 by Luke Wren (the RP2350 core), RV32EC without CSRs or traps (the firmware uses none; this saves about 15k um^2), vendored with one marked change (the never-written x0 register-file entry is cleared during reset, so gate-level simulation has no X from it). Runs C compiled with stock GCC or Clang. |
| Memory | `pg_sram.v` | One IHP `RM_IHPSG13_1P_1024x64_c2_bm_bist` macro used as 2048 x 32: 8 KB for code, data and stack. |
| Fabric | `pg_ahb.v` | AHB-Lite: SRAM with zero read wait states; register window at 0x8000_0000 (one wait state per access: read data and ready are registered, which keeps the I/O decode off the CPU's critical path); host port: while the CPU runs, a host byte access takes priority over a CPU SRAM read (the CPU waits one cycle per host byte), so the host can load a program into one part of memory or read results while the CPU keeps running. A held read word shares the register that carries register-port read data, so the SRAM still reaches the CPU through a single 2-input mux. |
| Host port | `pg_host_spi.v` | SPI target (mode 0): WRITE/READ memory, CTRL (run), STATUS. |
| I/O | `pg_io.v` | Address decode, two-flop input synchronizers, the blocking-access timeout, and the pinmux. |
| System | `pg_sysio.v` | Cycle counter, GPIO, timed GPIO, input edge capture, host mailbox and IRQ. |
| Wire Engines | `pg_wire.v`, `pg_fifo.v` | Four protocol-agnostic serial engines: 0 and 1 full, 2 and 3 lite (section 4). |
| DMA | `pg_dma.v` | Two channels between SRAM ring buffers and any IO register (engine RX/TX, GPIO), paced by an engine's FIFO or a timer (section 4). |

## 2. Programming model

- Memory map: SRAM at 0x0000_0000 (8 KB; the last 256 bytes, 0x1F00-0x1FFF, are shared with the host), system registers at 0x8000_0000, Wire Engine *n* at 0x8000_0100 + 0x100·*n*.
- Toolchain: `riscv-none-elf-gcc -march=rv32ec -mabi=ilp32e`, `sw/rt/crt0.S`, `sw/rt/link.ld`. No custom assembler or compiler.
- **Blocking I/O**: reading an empty RX FIFO, writing a full TX FIFO, waiting for an edge capture or for the timed-GPIO queue to drain all simply stall the CPU's bus access (AHB wait states) until the hardware is ready. C code reads like ordinary sequential code (`uart_getc()` waits for a byte) and the CPU uses no power-hungry polling loop. A global timeout (`BLK_TIMEOUT`) bounds every wait.
- **Several sources at once**: `wait_events(mask)` sleeps until any selected engine has received data (or an edge was captured) and returns which ones; one C loop serves several protocols (the USB keyboard example also listens on a UART).
- **Time**: `now()` is a 32-bit cycle counter. Everything that must happen at an exact time is done by hardware that takes a time stamp from C.

## 3. Timed GPIO and capture

`pin_out_at(pin, value, t)` queues a change (4 deep) that the hardware applies in exactly the cycle in which `now() == t`; `pins_out_at()` changes several pins in the same cycle, `pin_oe_at()` switches a driver (open drain). C only has to stay ahead of time; how long the compiled code takes does not matter. A change whose time has passed is applied at once and sets a LATE flag. Edge capture time-stamps an input edge to the cycle (the synchronizer delay is compensated).

This is the universal fallback for protocols no engine fits: the 1-Wire master in `sw/examples/onewire` uses nothing else. *(simulation: edges exactly on the scheduled cycle, including changes one cycle apart; capture error 0 cycles)*

## 4. Wire Engines

A Wire Engine is a serial engine with no protocol in it. Protocols are parameter sets and a few lines of C in `protocol.h`.

Engines 0 and 1 are full engines. Engines 2 and 3 are built with `FULL = 0` ("lite"): they lack line codes (NRZI, Manchester, biphase), bit stuffing, differential/SE0 output, packet framing options, CRC, TX word attributes, the second bit rate and the timed auto-reply; those registers read 0. They keep the fractional bit period and RX resync, so UART, SPI, I2C, WS2812 and similar bit streams work on every engine; CAN and 10BASE-T transmit need engine 0 or 1. Engine 1 alone (`MRX = 1`) also has the Manchester decoder and the SYNC hunt, so USB and 10BASE-T receive run on engine 1: keeping these out of engine 0 saved about 3.4k um^2, which mattered because the routing near the SRAM is the limit of this floorplan. Engine 1's decoder also reads biphase (S/PDIF, AES3, USB PD), and engine 0 alone (`FD = 1`) has the CAN FD unit (below). A full engine is about 116k um^2 after synthesis, a lite one about 80k. Four is the most this floorplan takes at 50 MHz: with a fifth (lite) engine the layout still routed DRC- and LVS-clean but missed setup at the slow corner by 0.45 ns (the SRAM-to-CPU path got longer as the logic spread out), and with a third full engine global routing overflowed (P&R).

```
 TX FIFO (4 x {attributes, length, 32-bit data})
   -> shift register (1..32 bits, LSB or MSB first)
   -> [bit stuffing: insert after N equal bits / N ones]
   -> [CRC unit 1..32 bits: append after a word]
   -> line code: NRZ / NRZI / Manchester / biphase mark / differential Manchester
      (per word: RAW = no line code; FAST = second bit period, switched at a word
      boundary or at the sample point of the last bit)
   -> pin driver: push-pull / open drain (per word: push-pull) / driven only in a frame / differential pair (+SE0)
 RX pin -> [sync to bit clock | async start edge + bit timer + edge resync | external clock]
   -> line decode (NRZI; engine 1: Manchester and biphase by edge timing) -> [SYNC hunt, engine 1]
   -> destuffing (+error flag) -> CRC residual
   -> [engine 0: CAN FD unit: FDF/BRS/DLC, RX bit rate switch, fixed stuff bits,
       CRC-17/21, stuff count, automatic ACK]
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
| Reply inside a packet (`ACKBIT`): the answer leaves TURN cycles after RX decodes bit N, only if the RX CRC over the bits before it is zero | CAN receiver ACK (C sets N from the DLC in the first 32 bits) |
| Manchester, CRC-32, idle tail (TP_IDL) | 10BASE-T transmit |
| Manchester decoding by edge timing: an edge at least 3/4 bit after the last mid-bit transition is the next one, so the decoder follows the sender's clock; frame end after 2 bits without one; MSTART for buses that idle at the first half's level | 10BASE-T receive (with SFD hunt and CRC-32), DALI backward frames (engine 1) |
| SYNC hunt: RX delivers, counts and checks only the bits after an 8-bit pattern | Ethernet SFD; USB SYNC, so packets arrive PID first as 16-bit words (needed at full speed) (engine 1) |
| Arbitration by read-back (open drain or a separate RX pin), stuffing after five equal bits, CRC-15, end of frame by an idle run | CAN 2.0A |
| Partial last word at a packet end (bit count in the end marker) | CAN frames of any length |
| Second bit period (`DIV2`, per-word `FAST`), switched at the sample point of the last bit (`RSW_SP`), programmable sample points (`PHASE`) | CAN FD data phase (up to 5 Mbit/s tested); S/PDIF preambles as half-slot words |
| CAN FD unit (engine 0): reads IDE, FDF, BRS and the DLC of every frame, switches the RX bit rate at BRS and back at the CRC delimiter, removes the fixed stuff bits, checks CRC-17/21 (with dynamic stuff bits) and the Gray-coded stuff count, ACKs good frames by itself; also follows classic frames | CAN FD (ISO 11898-1:2015) receive and ACK |
| Biphase mark / differential Manchester TX (`CODE 3`), and per-word RAW words for code violations | S/PDIF, AES3, USB Power Delivery BMC, SMPTE LTC |
| Biphase RX by edge intervals (short / long / violation), preamble type delivered in the word | S/PDIF and AES3 at 48 and 96 kHz (engine 1) |
| Per-word push-pull drive in open-drain mode (`PP`) | MIPI I3C SDR: open-drain address phase, push-pull data at 12.5 MHz |

**DMA.** Two channels each move 32-bit words between a ring buffer in SRAM and one IO register, without the CPU. The DMA is a second master on the IO register bus (when it needs a cycle, the CPU's register access waits one cycle) and a requester on the SRAM port after the host. A word moves when its pace allows: the engine has received data, the engine's TX FIFO has room, or a timer tick every N cycles. With an engine's RX register this is a receive buffer of any size (CAN FD frames back to back at 1 + 5 Mbit/s while C spends 100 us on each frame: without the DMA the 4-word FIFO overflows); with its TX register a transmit buffer (UART frames without gaps); with GPIO_IN a logic analyzer that samples all inputs every N cycles; with GPIO_OUT a pattern generator that changes the uo pins exactly every N cycles. Ring pointers: the DMA owns HEAD, C owns TAIL; a full receive ring stops the channel, so a ring with TAIL = START is a one-shot capture. Evidence: RTL simulation (`test_dma`, `test_canfd`).

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

Any engine can own any output pin; pins an engine does not own follow GPIO (and timed GPIO). The engines' pin signals (value, output enable, ownership) leave each engine through flip-flops, and GPIO is a register too, so the pin mux only selects between register outputs: the pins do not glitch. Inputs from all pins (and a read-back of the outputs) go through two-flop synchronizers.

## 6. Implementation results

| Item | Value | Evidence |
|---|---|---|
| Clock | 50 MHz rated (slow corner 1.08 V / 125 C); typical-corner slack suggests 60 MHz or more at room temperature | P&R |
| Setup slack at 50 MHz | +4.13 ns slow corner, +9.90 ns typical; hold +0.048 ns fast corner. Global placement is timing-driven (with it off, the same netlist kept only +0.27 ns). Setup repair runs again after global routing and continues until 3.3 ns of slack remain instead of stopping at zero. A 60 MHz build of an earlier layout missed the slow corner by 0.6 ns, so the rating stays at 50 MHz | P&R (2026-11-21) |
| Critical path | inside the CPU: instruction decode into the fetch frontend (slow corner) | P&R |
| Cell area | 34,475 cells, 523,068 um^2 after synthesis (flat, area-oriented mapping; Hazard3 without CSRs). The CAN FD unit, second bit rate, biphase decoder and per-word drive options added 38k um^2 to the 467k of the four engines before them, the two DMA channels 29k (254 flip-flops). One more lite engine would add about 55k um^2, one more full engine about 87k | synthesis |
| Std-cell utilization | 69.2 % of the 8x4 tile with the 8 KB SRAM; 572k um^2 of cells after place and route | P&R |
| Power | About 18 mW at 50 MHz (typical corner, 1.2 V, 25 C, no switching activity file): 70 % in the flip-flops' clock pins, 30 % in the clock tree, under 1 % in logic. The SRAM's library models almost no dynamic power, so its share is missing. There is no clock gating: the PDK's flow configuration excludes its clock-gating cells | P&R estimate |
| Sign-off | LVS clean, KLayout DRC clean, antenna clean, Tiny Tapeout precheck passes | P&R |

The SRAM macro needs special care on the CMOS5L tile, whose power grid is a single layer of Metal4 stripes: every stripe must run the full tile height, and a stripe may only cross the macro on one of the macro's own Metal4 supply columns of the same net. A LibreLane step in this repository (`librelane_plugin_polyglot.py`, `flow/odb_sram_stripes.py`) moves the stripes that cross the macro onto supply columns and connects them to the standard-cell rails. Floorplan: all Tiny Tapeout pins sit on the top edge between x = 29 and 191 um, so the macro sits in the bottom-right corner, as far from them as it can be. CMOS5L routes signals on Metal2-Metal4 only (TopMetal1 belongs to the chip-level power grid), so Metal3 is the only horizontal signal layer, and it is the layer that fills up first. With the macro in the bottom-left corner (the earlier layouts) the pins, the engines and the macro all crowded the left end; moving it to the right made no difference with four engines but was what let six engines route (global-routing overflow 0 instead of 207). A 6 um cell-free halo around the macro keeps tracks free at its corner. Placement turned out to be sensitive to where flip-flops sit on the output path: registering the pins after the pin mux made the placer's wire length 18 % worse and routing failed, while registering the same signals inside each engine improved it. The same happened with configuration bits: the Manchester receiver's SYNC and MSTART bits, stored and readable in all four engines although only engine 1 uses them, raised global-routing overflow from 127 to 1,284; storing them only in engine 1 brought it to 22. Changing density, cell padding or soft placement blockages instead made it worse every time.

Area: three flow settings took the placed-and-routed cell area from 676k to 518k um^2 (four engines) with more setup slack, not less. (1) Hold uncertainty. LibreLane's default constraints apply the 0.25 ns clock uncertainty to hold checks too, on top of propagated clocks and a 5 % early/late derate; after clock-tree synthesis 4,101 of the 4,402 flip-flops "failed" hold and repair added 4,527 delay buffers (about 60k um^2). `src/polyglot.sdc` keeps 0.25 ns for setup and 0.1 ns for hold; repair now adds 46 buffers and the fast-corner hold slack at sign-off is larger than before. (2) Synthesis strategy. The Tiny Tapeout default maps for delay (`DELAY 4`, 565k um^2); `AREA 0` gives 467k with the same 50 MHz rating. (3) Placement repair margins: 0.05 ns hold margin after placement, maximum fanout 16. A trap on the way: before sign-off, LibreLane's per-corner setup numbers for the slow and fast corners are copied from the pre-placement timing of the unbuffered netlist (the intermediate steps time only the typical corner), so an area-mapped netlist seemed to miss timing by 33 ns until sign-off showed +3.4 ns. With the room this frees, trial layouts with five Wire Engines (2 full + 3 lite, and 3 full + 2 lite) and with six (2 full + 4 lite) pass LVS, DRC and antenna checks at 50 MHz (the 2 full + 3 lite and the six-engine layouts also pass the Tiny Tapeout precheck); seven route cleanly but miss 50 MHz by 1.7 ns at the slow corner (P&R trials, 2026-11-21). The submitted design keeps four engines and used the room for protocol hardware (CAN FD, I3C, biphase) and two DMA channels instead (section 4); a six-engine layout (637k um^2 of cells) still closed, so roughly 40k um^2 of placed cells remained; that room went into doubling the SRAM to 8 KB (below).

Memory size: 8 KB is the most this tile takes with all of the logic. The tall 8 KB macro (2048 x 32, 417 x 627 um) did not route in any of five placements (global-routing overflow 1,463 to 15,721): it blocks the horizontal routing layer over most of the tile height, and Metal3 is the only horizontal signal layer. The same 8 KB as a wide, short 1024 x 64 macro (784 x 336 um, as tall as the 4 KB one) routes with no overflow, is as fast as the 4 KB macro (7.27 ns read at the slow corner, against 8.97 ns), and gave the best setup slack so far (+4.13 ns). A 12 KB trial (1024 x 64 + 1024 x 32 side by side) overflowed by 5,165 with Metal3 at 85 %. A smaller CPU would not make room for more at an acceptable cost: the bit-serial cores SERV and QERV are about 19k um^2 against Hazard3's 115k, but a simulation-only knob that slows the CPU shows that one extra wait state per memory access already breaks USB full speed and CAN FD at 1 + 5 Mbit/s, and an 8x slower CPU (like QERV) breaks 33 of 78 tests (P&R trials and RTL simulation, 2026-11-22).

Every SRAM input is driven by its own `buf_4` placed in `pg_sram.v`: the macro's inputs are characterized only up to 0.6 ns transition, and without these buffers some address bits arrived with 1.8 ns edges from small logic gates over long wires along the macro edge. Wire parasitics before routing: the CMOS5L flow configuration ships no fitted wire RC, so placement-time repair used the tech-LEF values (about 0.09 fF/um), half of what extraction later finds on this layout (0.18 fF/um median). Weak gates on long nets then passed repair and ended up with 3-4 ns edges. `src/config.json` sets `LAYERS_RC` to OpenROAD's fit for the same metal stack (from the SG13G2 configuration) and repairs once more after global routing; slow-corner slew warnings went from 42 to 11 (8 in the current layout) and capacitance warnings from 27 to 3, and setup slack improved. The approach to the power grid follows Ken Pettit's IHP SRAM write-up; the code is our own.

## 7. Credits

Hazard3 (Luke Wren, Apache-2.0); IHP Open PDK and SRAM macros; Tiny Tapeout template and tools; LibreLane; Ken Pettit's IHP SRAM macro walkthrough for the power-grid approach.
