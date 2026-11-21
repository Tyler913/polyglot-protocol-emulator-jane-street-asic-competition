# Programming Polyglot

Polyglot runs ordinary C. The whole interface is one header, `sw/include/protocol.h`; there is no custom assembler and no custom compiler. This guide goes from a blinking pin to a protocol the chip was never designed for.

## 1. Build and load

```sh
riscv-none-elf-gcc -march=rv32ec -mabi=ilp32e -Os -ffreestanding -nostdlib \
    -Isw/include -T sw/rt/link.ld sw/rt/crt0.S main.c -lgcc -o prog.elf
riscv-none-elf-objcopy -O binary prog.elf prog.bin
```

(`make -C sw` does this for every program in `sw/examples` and `sw/tests`.) The host MCU writes `prog.bin` to address 0 over the host SPI port while the CPU is held in reset, then sets RUN. Parameters and results can be exchanged through the last 256 bytes of SRAM (`PG_SHARED`), and the program can raise `HOST_IRQ`.

## 2. Exact timing from C

```c
#include "protocol.h"

int main(void) {
    uint32_t t = now() + 100;
    for (int i = 0; i < 8; i++, t += 50)
        pin_out_at(PIN_UO(0), i & 1, t);   /* each edge lands exactly on cycle t */
    timed_flush();                         /* wait until all of them happened */
}
```

The hardware applies each change in exactly the cycle asked for. The C code only has to schedule ahead, so compiler, optimisation level and loop shape do not matter. `capture_wait()` time-stamps input edges to the cycle, and every waiting access (`capture_wait`, FIFO reads and writes, `timed_flush`) just stalls the CPU until the hardware is ready, with an optional timeout.

## 3. A protocol is a set of parameters

A Wire Engine knows nothing about protocols. A UART is three lines:

```c
wire_config(W0, WIRE_UART_8N1(PIN_UO(1), PIN_UI(0), 115200));
uart_putc(W0, 'A');          /* blocks only if the TX FIFO is full */
int c = uart_getc(W0);       /* blocks until a byte arrives */
```

`WIRE_UART_8N1` expands to generic settings: 10-bit words sent LSB first, line idles high, receiver starts on an edge, bit period 434 + 7/256 cycles. SPI, I2C, USB and Ethernet are built the same way (`WIRE_SPI_CTRL`, `i2c_init`, `usb_ls_init`, `eth_init` in the header). Each is a few lines of C that anyone can read and change.

## 4. A protocol the chip was not designed for

**WS2812 LEDs** (`sw/examples/ws2812`): each LED bit is a high pulse of 0.4 or 0.8 us. There is no WS2812 hardware, but a plain NRZ engine at 2.4 Mbit/s sends `100` for a 0 and `110` for a 1:

```c
static uint32_t expand(uint8_t b) {            /* 8 LED bits -> 24 line bits */
    uint32_t w = 0;
    for (int i = 7; i >= 0; i--) w = (w << 3) | ((b >> i) & 1 ? 6u : 4u);
    return w;
}
...
wire_putn(LEDS, 24, expand(green));
```

**1-Wire** (`sw/examples/onewire`): no engine at all. Every edge of the reset pulse and of each time slot is scheduled with `pin_oe_at()` on an open-drain pin, and the read sample is taken at a known time:

```c
static int ow_read_bit(void) {
    uint32_t t = now() + US(2);
    pin_oe_at(DQ, 1, t);  pin_oe_at(DQ, 0, t + US(6));   /* 6 us low pulse */
    wait_until(t + US(13));
    int b = pin_read(DQ);
    wait_until(t + US(70));
    return b;
}
```

Both are checked against the device datasheets in simulation (`test/test_ws2812.py`, `test/test_onewire.py`).

**DMX512** stage lighting (`sw/examples/dmx512`, 220 bytes) is a 250 kbit/s stream of 11-bit slots, and each packet starts with a BREAK (low for at least 92 us) and a MARK AFTER BREAK (high for at least 12 us). With the line idling high, all of it is plain pushes:

```c
wire_putn(DMX, 26, 0x7u << 23);                        /* BREAK 23 bits low, MAB 3 bits high */
wire_putn(DMX, 11, 0x600u | (start_code << 1));        /* start bit, 8 data bits, 2 stop bits */
for (i = 1; i <= n; i++) wire_putn(DMX, 11, 0x600u | (level[i] << 1));
```

A receiver model written from ANSI E1.11 checks the timing (`test/test_dmx512.py`).

**Infrared remote** (`sw/examples/ir_nec`): an NEC remote control needs a 38 kHz carrier switched on and off in 562.5 us steps. No carrier generator is needed: at 76 kbit/s (fractional divider: 657 + 228/256 cycles per bit) a mark is the bit pattern `1010...` and a space is zeros, so the modulated waveform is plain pushes, and the receiver model sees a clean 38.000 kHz carrier (`test/test_ir_nec.py`).

**ARM Serial Wire Debug** (`sw/examples/swd`, `swd_xfer` in `protocol.h`): one bidirectional data line (SWDIO) with a turnaround cycle each time the direction changes. The engine has no half-duplex mode for this, and needs none: SWCLK is the bit clock in SPI mode 0, SWDIO is both the TX and the RX pin, and at each turnaround the C code waits for the last bit and rewrites `CTRL` to switch the TX driver off and RX on (or back). SWD lets the clock stop between phases, so the pause costs nothing:

```c
wire_putn(w, 8, req);                 /* request: host drives */
swd_turn(w, 0);                       /* release SWDIO, RX on */
wire_putn(w, rnw ? 4 : 5, 0u);        /* turnaround + 3-bit ACK (+ turnaround) */
int ack = (wire_get(w) >> 1) & 7;
```

The example runs on a lite engine: it switches a debug port from JTAG to SWD, reads DPIDR and the AP IDR, powers up the debug domain and writes and reads target memory through the MEM-AP, retrying WAIT answers (`test/test_swd.py`, 1 and 5 MHz).

**JTAG** (`sw/examples/jtag`, `jtag_tms` / `jtag_shift` in `protocol.h`) uses the same idea for its fourth wire: the engine clocks TDI out and TDO in on TCK, and TMS is a GPIO pin that C changes only while the clock is stopped, so it meets setup and hold by construction. A shift is one push; its last bit is pushed on its own so that TMS can rise with it. The example reads the IDCODEs of a two-device chain, puts one device in BYPASS and writes a data register in the other (`test/test_jtag.py`).

**PS/2** (`sw/examples/ps2_host`): here the device makes the clock. The engine runs on the keyboard's CLK (`WIRE_CLK_EXT`), reading DATA on falling edges. To send a command, C holds CLK low for 120 us, pushes the 11-bit frame on an open-drain TX (its start bit pulls DATA low at once: the request to send) and releases CLK; the keyboard then clocks the frame in, and the engine moves to the next bit on each falling edge. Switching direction is again just a `wire_config` (`test/test_ps2.py`).

## 5. Choosing a mechanism

| The protocol needs | Use |
|---|---|
| Shifting words at a fixed bit rate, any rate | engine, internal clock (fractional divider) |
| A clock from the other side | engine, `WIRE_CLK_EXT`, optional chip-select gate |
| Open drain, clock stretching, START/STOP | engine, `WIRE_OE_OPEN_DRAIN`, `WIRE_CLK_OD`, `WIRE_CLK_WAIT`, `WIRE_GATE_I2C`, `WIRE_STRETCH` |
| NRZI / Manchester, bit stuffing, CRC | engine `CTRL2` / `TXATTR` / `wire_crc()` |
| An answer within a few bit times of a received packet | queue it with `WIRE_A_AFTER_RX`; the engine sends it `TURN` cycles after the matching packet (`RXMATCH`) |
| An answer inside a packet, e.g. the CAN ACK slot | the same AFTER_RX word plus `ACKBIT = WIRE_ACK_AT(n)`: it leaves `TURN` cycles after RX bit n if the CRC checks (`can_recv_ack`) |
| Two bit rates in one frame (CAN FD) | `DIV2` and `WIRE_A_FAST` words; `WIRE_RSW_SP` switches at the sample point |
| Open drain for some words, push-pull for others (I3C) | `WIRE_OE_OPEN_DRAIN` plus `WIRE_A_PP` on the push-pull words |
| A line-code violation (sync, preamble) inside a coded stream | a `WIRE_A_RAW` word (with `WIRE_A_FAST` for half-bit steps) |
| Pulse widths, odd timing, anything else | timed GPIO (`pin_out_at`, `pins_out_at`, `pin_oe_at`) and edge capture |
| Several protocols at once | one engine each; `wait_events()` sleeps until any of them has data |
| More data than a 4-word FIFO holds while C is busy, or bits on exact cycles from a table | a DMA channel: `dma_rx` / `dma_tx` on an engine, `dma_capture` / `dma_pattern` on the GPIO registers |

### Several protocols in one program

Each engine runs on its own, so the CPU only has to keep the FIFOs fed. A simple way to write that in plain C is one cooperative task per protocol (`sw/include/pt.h`, in the style of protothreads). A task is written as straight-line code with `PT_WAIT(pt, condition)` where it would otherwise block, and `main()` calls the tasks round-robin. `sw/tests/mixed4` runs CAN, SPI, UART and I2C this way, one per engine. Two rules:

- Inside a task, use only accesses that cannot stall the CPU: wait for `wire_rx_count(w) > 0` before `wire_get()`, and for `!(w->STATUS & WIRE_TX_ACTIVE)` instead of `wire_flush()`.
- Keep every step short. The FIFOs are 4 words deep; four UART bytes at 1 Mbaud are 2000 cycles, so no task may run longer than that between waits.

### A queue of protocol jobs

More protocols than fit in memory at once can run as a queue (`sw/tests/queue`, `sw/include/job.h`, `sw/jobs/`). A 220-byte resident kernel calls up to four jobs round-robin, one per engine; each job is a cooperative task linked for its slot (slot 0 832 bytes, slot 1 768, slot 2 512, slot 3 1216), with 16 mailbox words for parameters and results. The host loads a job into a free slot over SPI and writes RUN into its mailbox; the other three slots keep running meanwhile (the host port takes one CPU cycle per byte). `test/test_queue.py` runs eight jobs this way in two waves of four: I2C, CAN, SPI and UART, then CAN at another rate, WS2812, UART at another rate and a second I2C EEPROM.

Job sizes: UART echo 208 bytes, WS2812 454, SPI RAM 694, CAN send/receive 784, I2C EEPROM write/poll/read 1122. To fit, the host passes bit periods already computed (`can_init_fix8`, `WIRE_UART_8N1_FIX8`) so no divide routine is linked into each job, and each job points `gp` at its own data, which makes most static-variable accesses one instruction.

### Fitting in 4 KB

The program, its data and a 256-byte stack share 3.5 KB (the top 256 bytes are the host mailbox). Measured sizes (code + data, GCC 15 `-Os`): UART echo 485 B, SPI RAM 674 B, I2C EEPROM 954 B, CAN 734 B, USB HID keyboard 1.9 KB, and all of CAN + SPI + UART + I2C as concurrent tasks 3.5 KB. The core has no multiply/divide unit, so avoid 64-bit arithmetic at run time: one 64-bit division pulls about 1.4 KB of libgcc into the image. `protocol.h` keeps its own helpers 32-bit.

## 6. Reference (protocol.h)

| Group | Functions and macros |
|---|---|
| Time | `now()`, `time_reached(t)`, `wait_until(t)`, `delay_ticks(n)`, `PG_US_TO_TICKS(us)`, `PG_F_CPU` |
| GPIO | `pin_output`, `pin_input`, `pin_write`, `pin_toggle`, `pin_read`; pins `PIN_UIO(n)`, `PIN_UO(n)`, `PIN_UI(n)` |
| Timed GPIO | `pin_out_at`, `pins_out_at`, `pin_oe_at`, `timed_flush`, `timed_late` |
| Capture and waiting | `capture_arm`, `capture_wait`, `capture_ready`, `pin_wait_edge`, `block_timeout`, `block_timed_out`, `wait_events` |
| Host | `PG_SHARED[]`, `host_irq`, `pg_park` |
| Engines | `W0`..`W3` (`W2`, `W3` lite), `wire_config`, `wire_put`, `wire_putn`, `wire_get`, `wire_rx_count`, `wire_flush`, `wire_tx_discard`, `wire_crc`, `WIRE_*` bits |
| UART | `WIRE_UART_8N1`, `uart_putc`, `uart_puts`, `uart_getc` |
| SPI | `WIRE_SPI_CTRL`, `WIRE_SPI_CTRL_DLY`, `spi_select`, `spi_deselect`, `spi_xfer`, `spi_transfer`, `WIRE_SPI_TARGET` |
| I2C | `i2c_init`, `i2c_start`, `i2c_stop`, `i2c_write`, `i2c_read`, `WIRE_I2C_TARGET` |
| USB (W1) | `usb_ls_init`, `usb_fs_init`, `usb_send_data`, `usb_queue_handshake`, `usb_rx_word`, `usb_crc5`, `USB_PID_*`, `USB_MATCH_*` |
| 10BASE-T | `eth_init`, `eth_send` (transmit); `eth_rx_init`, `eth_recv`, `ETH_CRC_RESIDUAL` (receive, W1) |
| Manchester RX, sync (W1) | `WIRE_MANCHESTER` on async RX, `WIRE_MSTART`, `wire_sync()` |
| CAN 2.0A | `can_init`, `can_send`, `can_recv`, `can_recv_ack` (receiver ACK), `can_parse`, `can_frame_t` |
| CAN FD (W0) | `canfd_init`, `canfd_send`, `canfd_recv`, `canfd_frame_t`, `canfd_dlc`, `FDCTL_*` |
| S/PDIF, AES3 | `spdif_tx_init`, `spdif_put` (any full engine); `spdif_rx_init`, `spdif_get` (W1); `SPDIF_B/M/W` |
| USB Power Delivery (W1) | `pd_init`, `pd_send`, `pd_recv`, `PD_HEADER`, `PD_HDR_*`, `PD_CTRL_*`, `PD_DATA_*` |
| SMPTE LTC | `ltc_init`, `ltc_send`, `ltc_decode`, `ltc_time_t` |
| MIPI I3C SDR | `i3c_init`, `i3c_start`, `i3c_stop`, `i3c_header`, `i3c_write`, `i3c_read`, `i3c_ccc`, `i3c_read_msg` |
| Second bit rate, biphase | `w->DIV2`, `WIRE_DIV2_FIX8`, `w->PHASE`, `WIRE_PHASE`, `WIRE_A_FAST`, `WIRE_A_PP`, `WIRE_A_RAW`, `WIRE_RSW_SP`, `WIRE_BIPHASE`, `WIRE_BIPHASE_DM` |
| SWD, JTAG | `WIRE_SWD`, `swd_connect`, `swd_xfer`, `SWD_REQ`; `WIRE_JTAG`, `jtag_tms`, `jtag_shift`, `JTAG_*` TMS sequences |
| DMA | `DMA0`, `DMA1`, `dma_start`, `dma_rx`, `dma_count`, `dma_get`, `dma_tx`, `dma_room`, `dma_put`, `dma_capture` (logic analyzer), `dma_pattern` (pattern generator), `canfd_rx_dma` / `canfd_recv_dma`, `EV_DMA(k)` |
| Several protocols at once | `pt.h`: `PT_BEGIN`, `PT_WAIT`, `PT_YIELD`, `PT_END` (cooperative tasks); `job.h`: queue jobs (`JOB_ENTRY`, slots, mailbox) |

The register-level description of every block is in the header comments of `src/pg_sysio.v`, `src/pg_io.v` and `src/pg_wire.v`.
