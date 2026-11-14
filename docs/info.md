<!---
This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

Polyglot is a protocol emulator that you program in plain C. A Hazard3 RISC-V core (RV32EC, the core used in the RP2350) runs a program compiled with stock GCC or Clang from 4 KB of on-chip SRAM. The program drives the protocol pins through memory-mapped registers declared in one header, `protocol.h`.

Two kinds of hardware give the C program exact timing:

- **Timed GPIO**: `pin_out_at(pin, value, t)` queues a pin change that the hardware applies in exactly clock cycle `t`, however long the C code took to get there. Input edges can be timestamped the same way. Any protocol can be bit-banged with cycle-exact edges.
- **Wire Engines** (4 in this version; engines 0 and 1 are full, engines 2 and 3 are "lite": no line codes, bit stuffing, differential output, CRC or timed auto-reply, including the in-packet reply used for the CAN ACK): protocol-agnostic serial engines with a shift register, an internal or external bit clock, TX/RX FIFOs with per-word length, output-enable and open-drain control, clock stretching in both directions, chip-select or START/STOP framing with frame events, and arbitration detect. Reading an empty RX FIFO or writing a full TX FIFO simply stalls the CPU until the engine is ready. UART, SPI and I2C (controller and target for each) are a few lines of parameters and C in `protocol.h`, not hardware blocks.

Status: CPU, SRAM, host loader, timed GPIO, edge capture and the Wire Engines pass RTL simulation with C firmware for UART, SPI, I2C (both roles each), USB low-speed HID keyboard, 10BASE-T transmit, CAN 2.0A, JTAG, ARM SWD, WS2812, DMX512, 1-Wire, four UARTs at once and four different protocols at once, judged by independent peer models (36 tests). A gate-level simulation of the routed netlist boots C programs from the SRAM.

### Host interface (SPI target, mode 0, SCK <= clk/8)

| Command | Bytes | Meaning |
|---|---|---|
| WRITE | `01 AH AL D0 D1 ...` | write bytes to memory starting at byte address AH:AL |
| READ | `02 AH AL xx` then read | read bytes starting at AH:AL (after one dummy byte) |
| CTRL | `03 V` | bit 0 = RUN (1 releases the CPU from reset) |
| STATUS | `04 xx` | the second byte returns status (bit 0 = RUN) |

Load a program while RUN = 0, then set RUN = 1. The program starts at address 0. The last 256 bytes of SRAM (0x0F00-0x0FFF) are reserved for parameters and results shared with the host. The firmware can raise HOST_IRQ (uo[6]) to signal the host.

### Pin numbering in `protocol.h`

| Pins | Number |
|---|---|
| uio[0..7] (bidirectional) | 0..7 |
| uo[0..5] (outputs) | 8..13 |
| ui[0..4] (inputs) | 16..20 |

## How to test

1. Build the firmware: `make -C sw` (needs `riscv-none-elf-gcc` or any GCC with the rv32ec/ilp32e multilib).
2. Hold RUN = 0, write `sw/build/blink.bin` to address 0 with the WRITE command, then send CTRL `01`.
3. uo[0] toggles every 100 clock cycles.

UART echo: load `sw/build/uart_echo.bin` and connect a 115200 baud 8N1 serial adapter to uo[1] (chip TX) and ui[0] (chip RX). The chip prints `Polyglot` and echoes what you type in upper case.

SPI: `sw/build/spi_ram.bin` drives SCK on uo[2], MOSI on uo[3], CS_N on uo[4] and reads MISO on ui[1] (for a 23x-style SPI RAM with 8-bit addresses).

USB low speed: `sw/build/usb_hid.bin` is a USB HID boot keyboard (D- uio[4], D+ uio[5], 1.5 kOhm pull-up on D-, series resistors / level shifting to 3.3 V USB levels as usual). After enumeration it types "Hi", then whatever arrives on its UART (ui[0], 115200 baud): a serial-to-USB keyboard.

CAN: `sw/tests/can` shows CAN 2.0A at 500 kbit/s (TXD on uio[0], RXD on ui[2], through a CAN transceiver): sending with arbitration and retry; `sw/tests/can_rx` receives and acknowledges frames (ACK slot driven only when the CRC checks).

Protocols without hardware support: `sw/build/ws2812.bin` drives WS2812 LEDs on uo[3] from a plain engine bit stream; `sw/build/onewire.bin` is a 1-Wire master on uio[6] made only from timed GPIO.

I2C (pull-ups needed): `sw/build/i2c_eeprom.bin` talks to a 24C02 EEPROM on SDA uio[0] / SCL uio[1]; `sw/build/i2c_target.bin` makes the chip an I2C target at address 0x42 (16 registers) on SDA uio[2] / SCL uio[3].

## External hardware

None for the bring-up test. A logic analyser on uo[0] shows the blink. A USB serial adapter for the UART example.
