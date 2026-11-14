# FPGA prototype (work in progress)

Target board: ALIENTEK DFZU2EG (AMD Zynq UltraScale+ XCZU2EG-SFVC784-1).

- `pg_sram_fpga.v`: replaces `src/pg_sram.v` (the IHP SRAM macro) with one
  inferred block RAM with the same ports and read timing.
- `synth.tcl`: out-of-context synthesis, placement and routing of the chip
  top (`tt_um_tyler913_polyglot`) at 50 MHz, without board pins. Run in this
  directory: `vivado -mode batch -source synth.tcl` (reports in `out/`).

Result with Vivado 2025.2 (2026-11-14, evidence: FPGA place and route, out of
context): 5,119 LUTs (10.8 %), 2,904 flip-flops (3.1 %), 1 block RAM tile,
no DSPs; setup slack +12.42 ns at 50 MHz (critical path 7.56 ns, CPU register
bypass to the instruction decompressor), hold slack +0.018 ns.

Next: a board top (PL clock, reset, header pin map) and a host path.
