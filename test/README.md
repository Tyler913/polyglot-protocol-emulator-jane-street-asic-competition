# Tests

cocotb tests that run real C firmware on the RTL. Each test builds its program from `sw/examples/` or `sw/tests/` with the
RISC-V GCC on your `PATH` (`riscv-none-elf-gcc` by default), loads it over the host SPI port and checks the pins.
Without a RISC-V GCC (as on the Tiny Tapeout CI runners) the tests use the prebuilt images in `firmware/`;
`make -C ../sw prebuilt` refreshes them, and a test warns when a fresh build differs from the committed copy.
Protocol behaviour is judged by independent peer models written from the protocol definitions, not from the RTL.

```sh
pip install -r requirements.txt
make -B                           # default RTL suite (Icarus Verilog; 37 modules, 79 tests)
make -B SIM=verilator EXTENDED=yes # default + extended suite (62 modules, 114 tests)
make -B GATES=yes                 # gate-level simulation of a hardened netlist (copy it to gate_level_netlist.v first);
                                  # runs test_boot, test_quad and test_uart by default
python3 mutation.py               # run mutation checks against the independent models (Verilator + RISC-V GCC)
```

| Path | Purpose |
|---|---|
| `test_*.py` | 65 cocotb test modules (117 tests total): 37 default modules (79 tests), 25 extended modules (35 tests, `EXTENDED=yes`), `test_stress4.py` (manual multi-protocol stress sweep), and `test_rmii.py` / `test_eth100_ping.py` (experimental RMII variant, `RMII=yes`). See [../docs/verification.md](../docs/verification.md) and [../docs/protocols.md](../docs/protocols.md) for the full list |
| `lib/` | 48 helper and independent protocol peer-model modules (`host.py` for the SPI loader, `soc.py` for reset/firmware build/shared memory, `pins.py` for `ui_in`/`uo_out` helpers, `edges.py`, plus one model module per protocol family) |
| `firmware/` | committed prebuilt `.bin` images used when no RISC-V GCC is installed (`make -C ../sw prebuilt`) |
| `mutation.py` | mutation-check harness that injects deliberate firmware faults to verify the peer models reject them |
| `tb.v` | testbench wrapper around the top level |
