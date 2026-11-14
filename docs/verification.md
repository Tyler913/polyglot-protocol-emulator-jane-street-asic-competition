# Verification

## Approach

1. **The CPU is reused, not re-verified.** Hazard3 is silicon-proven (RP2350) and was verified with riscv-formal by its author. We test the integration: fetch from SRAM, byte/halfword/word accesses through our fabric, register accesses, blocking wait states.
2. **Protocols are tested end to end with real firmware.** Every test builds a C program with stock GCC, loads it over the host SPI port exactly as the board MCU would, runs it on the RTL, and watches the pins.
3. **Protocol correctness is judged by independent peer models.** Each model is written from the protocol's specification or a device datasheet, not from our RTL: it decodes what the chip sends, checks timing rules, and answers as a real device or host would. A test passes only if the model is satisfied *and* the firmware reports success.
4. **Evidence levels.** Each claim says whether it comes from RTL simulation, gate-level simulation, P&R sign-off, or (later) FPGA.

## Test list (RTL simulation, `test/`, `make -C test`)

| Test | Firmware | Independent model / checks |
|---|---|---|
| `test_boot` | `memtest`, `blink`, `busy` | SPI load and read-back; byte/halfword/word stores and loads through the fabric (incl. store-then-load stalls); pin toggling without drift; host writes and reads while the CPU keeps the SRAM port fully busy |
| `test_tgpio` | `timed_gpio` | Every scheduled pin change lands on its exact cycle (incl. changes 1 cycle apart, multi-pin, output-enable); LATE flag; edge capture error 0 cycles; blocking timeout returns after the limit |
| `test_uart` | `uart_echo` | UART model (`lib/uart.py`): mid-bit decoding plus a check that every edge lies on the bit grid; 1 Mbaud back to back, 115200, 3 Mbaud at 16.67 cycles/bit with the sender 3 % fast; framing-error recovery |
| `test_spi` | `spi_ram`, `spi_target` | SPI RAM model (23x-style) in modes 0-3 at 1, 5 and 12.5 MHz (F/4 with RX sample delay); protocol checks (SCK idle at CS edges, whole bytes); SPI target driven by a controller model with CS framing and an aborted frame |
| `test_i2c` | `i2c_eeprom`, `i2c_target` | Open-drain bus model; 24C02-style EEPROM (page write, write cycle with ACK polling, random read with repeated START, clock stretching) that checks setup/hold against SCL; the chip as a target driven by a controller model, which flags any SDA change by the target while SCL is high |
| `test_usb` | `usb_phy` | USB low-speed line model (`lib/usb.py`): NRZI, stuffing, SYNC, PID check bits, CRC16 residual, EOP length, bit timing; host packet at -1 % rate; hardware handshake within the 2..7.5 bit-time window |
| `test_usb_hid` | `usb_hid` | A USB host model enumerates the C keyboard (reset, descriptors at address 0 and 5, SET_ADDRESS, SET_CONFIGURATION, SET_IDLE) and reads 4 reports ("Hi"); all 37 answers within the turnaround limit. Concurrency: a UART model sends text at 115200 baud while the host polls; every character comes out as a key press |
| `test_eth` | `eth_tx` | IEEE 802.3 transmit decoder (`lib/eth.py`): Manchester symbols, differential pair, preamble/SFD, FCS checked with zlib, TP_IDL; run at a 40 MHz clock |
| `test_ws2812` | `ws2812` | WS2812 datasheet timing model: high-pulse widths and bit periods |
| `test_onewire` | `onewire` | 1-Wire device model (DS18B20-style ROM): reset/presence, write and read slot timing; CRC-8 checked in C |
| `test_dmx512` | `dmx512` | DMX512 receiver model from ANSI E1.11 (`lib/dmx512.py`): BREAK >= 92 us, MARK AFTER BREAK >= 12 us, 4 us bits on the start edge's grid (+/- 2 %), two stop bits, start code and channel data; 16 channels x 3 packets and a full 512-channel universe. Broken variants (84 us BREAK, 8 us MAB, one stop bit) fail it |
| `test_swd` | `swd` | ARM SW-DP + MEM-AP target model written from the ADIv5 rules (`lib/swd.py`): JTAG-to-SWD selection, lockout until DPIDR is read, request parity, ACK, turnarounds, posted AP reads, FAULT before power-up and ABORT, WAIT retries; flags contention on SWDIO and data changes too close to the rising SWCLK edge. Run at 1 MHz (with WAIT answers) and 5 MHz. Checked against broken firmware: a missing turnaround, a wrong write parity and a host that keeps driving SWDIO through the ACK all fail the test |
| `test_jtag` | `jtag` | IEEE 1149.1 chain model (`lib/jtag.py`): two devices with 16-state TAP controllers, IR capture (...01), IDCODE after reset, BYPASS, an 8-bit user register; checks TMS/TDI setup and hold around the rising TCK edge. Reads both IDCODEs, the captured IRs, counts the devices in BYPASS, writes the user register twice and reads the old values; 1 and 5 MHz |
| `test_quad` | `quad_uart` | All four Wire Engines at once (two full, two lite) as UARTs at 1 Mbaud, each judged by its own UART model: banners, then all four receivers busy at the same time and every byte echoed |
| `test_can` | `can`, `can_rx` | CAN bus model (`lib/can.py`, wired-AND bus with transceiver delay) and a second controller node: stuffing, CRC-15, ACK; the node joins the chip's SOF with a lower identifier, the chip loses arbitration at identifier bit 5, receives and acknowledges the winner's frame and retries. As a receiver only, the chip acknowledges frames with 0, 3 and 8 data bytes and does not acknowledge a frame with a bad CRC; each ACK is one bit long, starts 7 cycles (7 % of a bit) after the slot start in the sending node's timing, and the ACK delimiter stays recessive |
| `test_mixed` | `mixed4` | Four different protocols at once, one per engine, as four cooperative C tasks (`pt.h`): CAN (frame acknowledged by the bus model's node), SPI RAM write/read at 5 MHz, UART echo at 1 Mbaud and I2C EEPROM page write, ACK polling and read at 400 kHz. All four bus models run in the same simulation; the test also checks that the four tasks overlap in time |
| `test_queue` | `queue` + `sw/jobs/*` | A resident kernel runs up to four protocol jobs, one per engine; the host model loads eight jobs in two waves (I2C EEPROM 100 kHz, CAN 500 kbit/s, SPI RAM, UART 115200; then CAN 250 kbit/s on other pins, WS2812, UART 1 Mbaud, a second EEPROM at 400 kHz), each into its slot while the other slots keep running. Seven bus models run in the same simulation; the test checks every result and that second-wave jobs ran while a first-wave job was still running |

36 tests in all; they also run in GitHub Actions on every push.

## What the models found

Independent models catch mistakes in both the RTL and the test itself. Examples from development:

- RTL: after a UART frame with a low stop bit, the receiver started a phantom frame because it reacted to the line *level*. Fixed: start on an *edge* from idle, and drop a frame whose start bit is gone at mid-bit (glitch filter).
- RTL design gaps found while writing the I2C and USB tests: an external-clock engine must change data only on the launch edge (otherwise an I2C target would make a false START/STOP); a pre-queued auto-reply must not fire on a packet that ended before it was queued; cancelling a queued answer must not reset the receiver.
- Board-level issue found by the CAN model: after reset all `uo` outputs are 0, which on a CAN TXD is *dominant* and would block the bus while the program loads. CAN TXD therefore goes on a `uio` pin, which is high-impedance until the program enables the engine.
- RTL, found by `test_queue`: the host port only used SRAM cycles the CPU left idle. With four jobs running, the CPU used the port nearly every cycle, the host waited longer than one SPI byte, and the next byte overwrote the pending one, so a freshly loaded job had three missing bytes and trapped. Fixed in `pg_ahb.v` (the host now has priority, one CPU wait cycle per host byte); `test_boot` has a regression test that keeps the port 100 % busy.
- Firmware structure, found by `test_mixed`: with four tasks sharing one CPU, every task step must be shorter than the tightest FIFO allows (four UART bytes at 1 Mbaud = 2000 cycles). The CAN frame parser took longer, bit by bit, and the UART lost a byte; it now works on whole words with shifts.
- Test-side bugs (the RTL was right): an I2C controller model that released SDA for START, an EEPROM model that took a repeated START's SCL rise for a data bit, a 1-Wire model that took its own presence release for a reset.

## Gate level and sign-off

- P&R sign-off of the full design (four engines): LVS clean, KLayout DRC clean, antenna clean, setup/hold met at 50 MHz in all corners (+2.50 ns setup slack at the slow corner), Tiny Tapeout precheck passes *(P&R, local LibreLane run, 2026-11-13)*.
- Gate-level simulation of the routed netlist (`make -C test GATES=yes`, Icarus 13 with the IHP cell and SRAM models): SPI load and read-back, `memtest`, `blink`, host access while the CPU keeps the SRAM busy, all three UART tests and the four-engine `test_quad` pass *(gate-level simulation, local and in the Tiny Tapeout GDS workflow, 2026-11-13)*. `test_tgpio` reads the RTL's cycle counter by name and is skipped at gate level.
- What gate level found: registers that a program never writes are X in simulation; a speculative register read let that X spread through the pipeline and stopped the CPU (silicon would just hold a random, unused value). The startup code now clears every register (22 bytes), and the CPU clears x0's register-file entry in reset.

## FPGA

Planned: the same RTL and firmware on a Zynq UltraScale+ board, against real devices. *(pending)*
