# Tests

cocotb tests that run real C firmware on the RTL. Each test builds its program from `sw/examples/` or `sw/tests/` with the
RISC-V GCC on your `PATH` (`riscv-none-elf-gcc` by default), loads it over the host SPI port and checks the pins.
Protocol behaviour is judged by independent peer models written from the protocol definitions, not from the RTL.

```sh
pip install -r requirements.txt
make -B                 # RTL simulation (Icarus Verilog)
make -B GATES=yes       # gate-level simulation of a hardened netlist (copy it to gate_level_netlist.v first)
```

| File | Purpose |
|---|---|
| `test_boot.py` | bring-up: SPI load/read-back, byte/halfword/word memory test, blink timing |
| `test_tgpio.py` | timed GPIO: every scheduled edge lands on its exact cycle; LATE flag, edge capture, blocking timeout |
| `test_uart.py` | UART echo at 1 Mbaud (back to back) and 115200 baud; exact bit timing; framing error recovery |
| `test_spi.py` | SPI controller against an SPI RAM model in modes 0-3 (1 and 5 MHz); SPI target with external clock (modes 0, 3) |
| `lib/host.py` | model of the host SPI loader protocol |
| `lib/soc.py` | reset, firmware build/load, shared-memory and HOST_IRQ helpers |
| `lib/uart.py`, `lib/spi.py` | independent UART and SPI peer models |
| `lib/pins.py` | shared driver for `ui_in` (several models drive different bits) |
| `tb.v` | testbench wrapper around the top level |
