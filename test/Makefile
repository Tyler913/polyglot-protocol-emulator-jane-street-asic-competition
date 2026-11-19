# Makefile
# See https://docs.cocotb.org/en/stable/quickstart.html for more info

# defaults
SIM ?= icarus
FST ?= -fst # Use more efficient FST format
TOPLEVEL_LANG ?= verilog
SRC_DIR = $(PWD)/../src
PROJECT_SOURCES = pg_top.v pg_ahb.v pg_host_spi.v pg_sram.v pg_io.v pg_sysio.v pg_wire.v pg_fifo.v hazard3_alu.v hazard3_branchcmp.v hazard3_core.v hazard3_cpu_1port.v hazard3_csr.v hazard3_decode.v hazard3_frontend.v hazard3_instr_decompress.v hazard3_irq_ctrl.v hazard3_mul_fast.v hazard3_muldiv_seq.v hazard3_onehot_encode.v hazard3_onehot_priority.v hazard3_onehot_priority_dynamic.v hazard3_pmp.v hazard3_power_ctrl.v hazard3_priority_encode.v hazard3_regfile_1w2r.v hazard3_shift_barrel.v hazard3_triggers.v 

ifneq ($(GATES),yes)

# RTL simulation:
SIM_BUILD				= sim_build/rtl$(if $(filter verilator,$(SIM)),_verilator,)
VERILOG_SOURCES += $(addprefix $(SRC_DIR)/,$(PROJECT_SOURCES))

else

# Gate level simulation:
SIM_BUILD				= sim_build/gl
COMPILE_ARGS    += -DGL_TEST
COMPILE_ARGS    += -DFUNCTIONAL
COMPILE_ARGS    += -DSIM
# Cell models for the PDK in $(PDK) (ihp-sg13cmos5l or ihp-sg13g2), e.g. sg13cmos5l_stdcell.v
PDK ?= ihp-sg13cmos5l
PDK_CELL_PREFIX = $(patsubst ihp-%,%,$(PDK))
PDK_LIBS_REF = $(PDK_ROOT)/$(PDK)/libs.ref
VERILOG_SOURCES += $(PDK_LIBS_REF)/$(PDK_CELL_PREFIX)_io/verilog/$(PDK_CELL_PREFIX)_io.v
# Newer PDK versions ship the UDP (user-defined primitive) models in a separate file
VERILOG_SOURCES += $(wildcard $(PDK_LIBS_REF)/$(PDK_CELL_PREFIX)_stdcell/verilog/$(PDK_CELL_PREFIX)_udp.v)
VERILOG_SOURCES += $(PDK_LIBS_REF)/$(PDK_CELL_PREFIX)_stdcell/verilog/$(PDK_CELL_PREFIX)_stdcell.v
# SRAM macro model (functional, -DFUNCTIONAL above)
VERILOG_SOURCES += $(PDK_LIBS_REF)/$(PDK_CELL_PREFIX)_sram/verilog/RM_IHPSG13_1P_core_behavioral_bm_bist.v
VERILOG_SOURCES += $(PDK_LIBS_REF)/$(PDK_CELL_PREFIX)_sram/verilog/RM_IHPSG13_1P_1024x32_c2_bm_bist.v

# this gets copied in by the GDS action workflow
VERILOG_SOURCES += $(PWD)/gate_level_netlist.v

endif

# Verilator (make SIM=verilator): about 30x faster than Icarus on the long
# low-speed tests; two-state, so Icarus (the default, and the CI simulator)
# stays the reference for X propagation.
ifeq ($(SIM),verilator)
EXTRA_ARGS += --timing -Wno-fatal -Wno-lint -Wno-style
endif

# Allow sharing configuration between design and testbench via `include`:
COMPILE_ARGS 		+= -I$(SRC_DIR)

# Include the testbench sources:
VERILOG_SOURCES += $(PWD)/tb.v
TOPLEVEL = tb

# List test modules to run, separated by commas and without the .py suffix.
# Gate level (GATES=yes) runs a subset by default (boot, four-engine UART, UART baud rates): a whole RISC-V CPU at
# gate level is slow, and some RTL tests look at internal signals.
ifneq ($(GATES),yes)
TESTS_DEFAULT = test_boot,test_tgpio,test_uart,test_spi,test_i2c,test_i2c_sniff,test_sd_spi,test_usb,test_usb_hid,test_usb_host,test_eth,test_eth_rx,test_eth_ping,test_ws2812,test_pulse_codes,test_framing,test_mdio,test_smbus,test_dmx512,test_hd44780,test_onewire,test_can,test_swd,test_jtag,test_ps2,test_quad,test_mixed,test_queue,test_biss
# Slow at real protocol speeds (milliseconds of bus time): make EXTENDED=yes, best with SIM=verilator
TESTS_EXTENDED = test_lin,test_modbus,test_dali,test_ir_nec,test_ir_rx,test_gpio_protocols,test_cec,test_ps2_device,test_onewire_slave,test_eth_nlp,test_sbus,test_crsf,test_modbus_server,test_lin_slave,test_iolink,test_sdi12,test_kline,test_dht22
ifeq ($(EXTENDED),yes)
COCOTB_TEST_MODULES ?= $(TESTS_DEFAULT),$(TESTS_EXTENDED)
else
COCOTB_TEST_MODULES ?= $(TESTS_DEFAULT)
endif
else
COCOTB_TEST_MODULES ?= test_boot,test_quad,test_uart
endif

# include cocotb's make rules to take care of the simulator setup
include $(shell cocotb-config --makefiles)/Makefile.sim
