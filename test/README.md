# Tests

cocotb tests that run real C firmware on the RTL. Each test builds its program from `sw/examples/` or `sw/tests/` with the
RISC-V GCC on your `PATH` (`riscv-none-elf-gcc` by default), loads it over the host SPI port and checks the pins.
Without a RISC-V GCC (as on the Tiny Tapeout CI runners) the tests use the prebuilt images in `firmware/`;
`make -C ../sw prebuilt` refreshes them, and a test warns when a fresh build differs from the committed copy.
Protocol behaviour is judged by independent peer models written from the protocol definitions, not from the RTL.

```sh
pip install -r requirements.txt
make -B                 # RTL simulation (Icarus Verilog)
make -B GATES=yes       # gate-level simulation of a hardened netlist (copy it to gate_level_netlist.v first);
                        # runs test_boot, test_quad and test_uart by default
```

| File | Purpose |
|---|---|
| `test_boot.py` | bring-up: SPI load/read-back, byte/halfword/word memory test, blink timing, host access while the CPU keeps the SRAM busy |
| `test_tgpio.py` | timed GPIO: every scheduled edge lands on its exact cycle; LATE flag, edge capture, blocking timeout |
| `test_uart.py` | UART echo at 1 Mbaud (back to back) and 115200 baud; exact bit timing; framing error recovery |
| `test_spi.py` | SPI controller against an SPI RAM model in modes 0-3 (1, 5 and 12.5 MHz with RX sample delay); SPI target with CS framing, frame events and an aborted frame (modes 0, 3) |
| `test_i2c.py` | I2C controller against a 24C02-style EEPROM model (NACK, page write, ACK polling, repeated START, clock stretching, setup/hold checks); chip as I2C target driven by a controller model |
| `lib/host.py` | model of the host SPI loader protocol |
| `lib/soc.py` | reset, firmware build/load, shared-memory and HOST_IRQ helpers |
| `test_usb.py` | USB low-speed line layer: DATA0 from the chip (NRZI, stuffing, CRC16, EOP, 33.33-cycle bits) and a host DATA1 into the chip with the hardware-timed ACK inside the 2..7.5 bit-time window |
| `test_usb_hid.py` | USB low-speed HID keyboard written in C is enumerated by a host model (descriptors, SET_ADDRESS, SET_CONFIGURATION, SET_IDLE) and types "Hi"; every answer within the 7.5 bit-time turnaround. Second test: text sent to the chip's UART is typed over USB at the same time (two engines, one C loop) |
| `test_eth.py` | 10BASE-T transmit at a 40 MHz clock: Manchester, differential TD+/TD-, preamble/SFD, CRC-32 FCS checked with zlib |
| `test_ws2812.py` | WS2812 LEDs from a plain NRZ engine stream (3 line bits per LED bit), checked against datasheet pulse timing |
| `test_onewire.py` | 1-Wire master from timed GPIO only (reset/presence, write and read slots), READ ROM from a DS18B20-style model with timing checks |
| `test_can.py` | CAN 2.0A at 500 kbit/s against a CAN bus model with a second node: stuffing, CRC-15, ACK, and lost arbitration followed by a retry |
| `test_quad.py` | all four engines as UARTs at 1 Mbaud at the same time |
| `test_mixed.py` | CAN, SPI, UART and I2C at the same time, one per engine, as cooperative C tasks (`sw/include/pt.h`) |
| `test_queue.py` | job queue: eight protocol jobs in two waves of four, each loaded into its slot by the host while the other slots run (`sw/tests/queue`, `sw/jobs/`) |
| `lib/uart.py`, `lib/spi.py`, `lib/i2c.py`, `lib/usb.py`, `lib/eth.py`, `lib/ws2812.py`, `lib/onewire.py`, `lib/can.py` | independent UART, SPI, I2C, USB LS, 10BASE-T, WS2812, 1-Wire and CAN peer models; open-drain and USB bus models for the uio pins |
| `lib/pins.py` | shared driver for `ui_in` (several models drive different bits) |
| `tb.v` | testbench wrapper around the top level |
