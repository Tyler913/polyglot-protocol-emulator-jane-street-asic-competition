# Tests

cocotb tests that run real C firmware on the RTL. Each test builds its program from `sw/examples/` with the
RISC-V GCC on your `PATH` (`riscv-none-elf-gcc` by default), loads it over the host SPI port and checks the pins.

```sh
pip install -r requirements.txt
make -B                 # RTL simulation (Icarus Verilog)
make -B GATES=yes       # gate-level simulation of a hardened netlist (copy it to gate_level_netlist.v first)
```

| File | Purpose |
|---|---|
| `test_boot.py` | bring-up: SPI load/read-back, byte/halfword/word memory test, blink timing |
| `lib/host.py` | model of the host SPI loader protocol |
| `tb.v` | testbench wrapper around the top level |
