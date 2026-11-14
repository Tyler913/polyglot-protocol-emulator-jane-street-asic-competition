# cocotb simulation of the FPGA UART bridge with the chip RTL: make -C fpga/sim
SIM ?= icarus
TOPLEVEL_LANG = verilog
SRC_DIR = $(PWD)/../../src
PROJECT_SOURCES = $(shell sed -n "s/^PROJECT_SOURCES = //p" $(PWD)/../../test/Makefile)
VERILOG_SOURCES = $(addprefix $(SRC_DIR)/,$(PROJECT_SOURCES)) $(PWD)/../uart_host_bridge.v $(PWD)/sim_top.v
COMPILE_ARGS += -I$(SRC_DIR)
COCOTB_TOPLEVEL = sim_top
COCOTB_TEST_MODULES = test_bridge
include $(shell cocotb-config --makefiles)/Makefile.sim
