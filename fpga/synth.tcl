# Out-of-context synthesis, placement and routing of the chip top on the
# prototype board's FPGA (ALIENTEK DFZU2EG: XCZU2EG-SFVC784-1) at 50 MHz.
# Run from this directory: vivado -mode batch -source synth.tcl
set part xczu2eg-sfvc784-1-e
set srcs [list ../src/pg_top.v ../src/pg_ahb.v ../src/pg_host_spi.v ../src/pg_io.v ../src/pg_sysio.v ../src/pg_wire.v ../src/pg_fifo.v ../src/hazard3_alu.v ../src/hazard3_branchcmp.v ../src/hazard3_core.v ../src/hazard3_cpu_1port.v ../src/hazard3_csr.v ../src/hazard3_decode.v ../src/hazard3_frontend.v ../src/hazard3_instr_decompress.v ../src/hazard3_irq_ctrl.v ../src/hazard3_mul_fast.v ../src/hazard3_muldiv_seq.v ../src/hazard3_onehot_encode.v ../src/hazard3_onehot_priority.v ../src/hazard3_onehot_priority_dynamic.v ../src/hazard3_pmp.v ../src/hazard3_power_ctrl.v ../src/hazard3_priority_encode.v ../src/hazard3_regfile_1w2r.v ../src/hazard3_shift_barrel.v ../src/hazard3_triggers.v  pg_sram_fpga.v]
file mkdir out
read_verilog $srcs
set_property include_dirs ../src [current_fileset]
synth_design -top tt_um_tyler913_polyglot -part $part -mode out_of_context -include_dirs ../src
create_clock -name clk -period 20.000 [get_ports clk]
report_utilization -file out/util_synth.rpt
opt_design
place_design
route_design
report_utilization -file out/util.rpt
report_timing_summary -max_paths 5 -file out/timing.rpt
report_timing -max_paths 1 -nworst 1 -path_type full -file out/critical.rpt
