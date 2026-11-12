# Build all example and test programs with a stock RISC-V GCC.
#   make            -> build/<name>.elf / .bin / .lst for examples/<name> and tests/<name>
#   make CROSS=...  -> use another toolchain prefix
#
# Default toolchain: xPack riscv-none-elf-gcc (any GCC with rv32ec/ilp32e multilib works).

CROSS   ?= riscv-none-elf-
CC      := $(CROSS)gcc
OBJCOPY := $(CROSS)objcopy
OBJDUMP := $(CROSS)objdump
SIZE    := $(CROSS)size

ARCH    := -march=rv32ec -mabi=ilp32e
CFLAGS  := $(ARCH) -Os -g -ffreestanding -fno-builtin -Wall -Wextra -Iinclude --param=min-pagesize=0
LDFLAGS := $(ARCH) -nostdlib -nostartfiles -Wl,--gc-sections -Wl,--no-warn-rwx-segments -T rt/link.ld

BUILD    := build
PROGRAMS := $(notdir $(wildcard examples/* tests/*))

all: $(foreach e,$(PROGRAMS),$(BUILD)/$(e).bin $(BUILD)/$(e).lst)

define LINK
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -ffunction-sections -fdata-sections $(LDFLAGS) rt/crt0.S $< -lgcc -o $@
	$(SIZE) $@
endef

$(BUILD)/%.elf: examples/%/main.c rt/crt0.S rt/link.ld include/protocol.h
	$(LINK)

$(BUILD)/%.elf: tests/%/main.c rt/crt0.S rt/link.ld include/protocol.h
	$(LINK)

$(BUILD)/%.bin: $(BUILD)/%.elf
	$(OBJCOPY) -O binary $< $@

$(BUILD)/%.lst: $(BUILD)/%.elf
	$(OBJDUMP) -d $< > $@

clean:
	rm -rf $(BUILD)

.PHONY: all clean
.PRECIOUS: $(BUILD)/%.elf
