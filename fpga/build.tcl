# Full board build: bitstream for the ALIENTEK DFZU2EG (PL only).
# Run from this directory: vivado -mode batch -source build.tcl
# Output: out/polyglot_dfzu2eg.bit, reports in out/.
set part xczu2eg-sfvc784-1-e
set srcs [list ../src/pg_top.v ../src/pg_ahb.v ../src/pg_host_spi.v ../src/pg_io.v ../src/pg_sysio.v ../src/pg_wire.v ../src/pg_fifo.v ../src/hazard3_alu.v ../src/hazard3_branchcmp.v ../src/hazard3_core.v ../src/hazard3_cpu_1port.v ../src/hazard3_csr.v ../src/hazard3_decode.v ../src/hazard3_frontend.v ../src/hazard3_instr_decompress.v ../src/hazard3_irq_ctrl.v ../src/hazard3_mul_fast.v ../src/hazard3_muldiv_seq.v ../src/hazard3_onehot_encode.v ../src/hazard3_onehot_priority.v ../src/hazard3_onehot_priority_dynamic.v ../src/hazard3_pmp.v ../src/hazard3_power_ctrl.v ../src/hazard3_priority_encode.v ../src/hazard3_regfile_1w2r.v ../src/hazard3_shift_barrel.v ../src/hazard3_triggers.v  pg_sram_fpga.v uart_host_bridge.v board_top.v]
file mkdir out
read_verilog $srcs
read_xdc dfzu2eg.xdc
synth_design -top board_top -part $part -include_dirs ../src
opt_design
place_design
route_design
report_utilization -file out/board_util.rpt
report_timing_summary -max_paths 5 -file out/board_timing.rpt
report_drc -file out/board_drc.rpt
# Pin check: each port with its package pin and the pin's I/O function name
set f [open out/board_pins.txt w]
foreach p [lsort [get_ports]] {
  set pin [get_package_pins -of_objects $p]
  puts $f "$p $pin [get_property PIN_FUNC $pin]"
}
close $f
write_bitstream -force out/polyglot_dfzu2eg.bit
