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

## 5. Choosing a mechanism

| The protocol needs | Use |
|---|---|
| Shifting words at a fixed bit rate, any rate | engine, internal clock (fractional divider) |
| A clock from the other side | engine, `WIRE_CLK_EXT`, optional chip-select gate |
| Open drain, clock stretching, START/STOP | engine, `WIRE_OE_OPEN_DRAIN`, `WIRE_CLK_OD`, `WIRE_CLK_WAIT`, `WIRE_GATE_I2C`, `WIRE_STRETCH` |
| NRZI / Manchester, bit stuffing, CRC | engine `CTRL2` / `TXATTR` / `wire_crc()` |
| An answer within a few bit times of a received packet | queue it with `WIRE_A_AFTER_RX`; the engine sends it `TURN` cycles after the matching packet (`RXMATCH`) |
| Pulse widths, odd timing, anything else | timed GPIO (`pin_out_at`, `pins_out_at`, `pin_oe_at`) and edge capture |
| Several protocols at once | one engine each; `wait_events()` sleeps until any of them has data |

## 6. Reference (protocol.h)

| Group | Functions and macros |
|---|---|
| Time | `now()`, `time_reached(t)`, `wait_until(t)`, `delay_ticks(n)`, `PG_US_TO_TICKS(us)`, `PG_F_CPU` |
| GPIO | `pin_output`, `pin_input`, `pin_write`, `pin_toggle`, `pin_read`; pins `PIN_UIO(n)`, `PIN_UO(n)`, `PIN_UI(n)` |
| Timed GPIO | `pin_out_at`, `pins_out_at`, `pin_oe_at`, `timed_flush`, `timed_late` |
| Capture and waiting | `capture_arm`, `capture_wait`, `capture_ready`, `pin_wait_edge`, `block_timeout`, `block_timed_out`, `wait_events` |
| Host | `PG_SHARED[]`, `host_irq`, `pg_park` |
| Engines | `W0`, `W1`, `wire_config`, `wire_put`, `wire_putn`, `wire_get`, `wire_rx_count`, `wire_flush`, `wire_tx_discard`, `wire_crc`, `WIRE_*` bits |
| UART | `WIRE_UART_8N1`, `uart_putc`, `uart_puts`, `uart_getc` |
| SPI | `WIRE_SPI_CTRL`, `WIRE_SPI_CTRL_DLY`, `spi_select`, `spi_deselect`, `spi_xfer`, `spi_transfer`, `WIRE_SPI_TARGET` |
| I2C | `i2c_init`, `i2c_start`, `i2c_stop`, `i2c_write`, `i2c_read`, `WIRE_I2C_TARGET` |
| USB low speed | `usb_ls_init`, `usb_send_data`, `usb_queue_handshake`, `usb_crc5`, `USB_PID_*`, `USB_MATCH_*` |
| 10BASE-T | `eth_init`, `eth_send` |

The register-level description of every block is in the header comments of `src/pg_sysio.v`, `src/pg_io.v` and `src/pg_wire.v`.
