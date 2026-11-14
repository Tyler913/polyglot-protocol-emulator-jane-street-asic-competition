# Build all example and test programs with a stock RISC-V GCC.
#   make            -> build/<name>.elf / .bin / .lst for examples/<name> and tests/<name>,
#                      and build/job_<name>_<slot>.bin for each queue job in jobs/<name>
#   make CROSS=...  -> use another toolchain prefix
#   make prebuilt   -> also copy the .bin images to test/firmware/ (used when no toolchain)
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
HEADERS  := $(wildcard include/*.h)

# Queue jobs (jobs/<name>): one image per slot listed in jobs/<name>/slots, linked at
# that slot's base (slot map in include/job.h)
JOBS        := $(notdir $(wildcard jobs/*))
SLOT_BASE_0 := 0x100
SLOT_SIZE_0 := 0x340
SLOT_BASE_1 := 0x440
SLOT_SIZE_1 := 0x300
SLOT_BASE_2 := 0x740
SLOT_SIZE_2 := 0x200
SLOT_BASE_3 := 0x940
SLOT_SIZE_3 := 0x4C0
job_slots    = $(shell cat jobs/$(1)/slots)
JOB_BINS    := $(foreach j,$(JOBS),$(foreach k,$(call job_slots,$(j)),$(BUILD)/job_$(j)_$(k).bin))

all: $(foreach e,$(PROGRAMS),$(BUILD)/$(e).bin $(BUILD)/$(e).lst) $(JOB_BINS)

define LINK
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -ffunction-sections -fdata-sections $(LDFLAGS) rt/crt0.S $< -lgcc -o $@
	$(SIZE) $@
endef

# The job-queue kernel must end below the first job slot (rt/kernel.ld).
$(BUILD)/queue.elf: tests/queue/main.c rt/crt0.S rt/link.ld rt/kernel.ld $(HEADERS)
	$(subst rt/link.ld,rt/kernel.ld,$(LINK))

$(BUILD)/%.elf: examples/%/main.c rt/crt0.S rt/link.ld $(HEADERS)
	$(LINK)

$(BUILD)/%.elf: tests/%/main.c rt/crt0.S rt/link.ld $(HEADERS)
	$(LINK)

define JOB_RULE
$(BUILD)/job_$(1)_$(2).elf: jobs/$(1)/main.c rt/job.ld $(HEADERS)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -DJOB_SLOT=$(2) -ffunction-sections -fdata-sections $(ARCH) -nostdlib -nostartfiles \
	  -Wl,--gc-sections -Wl,--no-warn-rwx-segments -Wl,--defsym=SLOT_BASE=$(SLOT_BASE_$(2)) \
	  -Wl,--defsym=SLOT_SIZE=$(SLOT_SIZE_$(2)) -T rt/job.ld $$< -lgcc -o $$@
	$(SIZE) $$@
endef
$(foreach j,$(JOBS),$(foreach k,$(call job_slots,$(j)),$(eval $(call JOB_RULE,$(j),$(k)))))

$(BUILD)/%.bin: $(BUILD)/%.elf
	$(OBJCOPY) -O binary $< $@

$(BUILD)/%.lst: $(BUILD)/%.elf
	$(OBJDUMP) -d $< > $@

# Copy the images into test/firmware/ (committed) so the testbench runs without a toolchain.
prebuilt: $(foreach e,$(PROGRAMS),$(BUILD)/$(e).bin) $(JOB_BINS)
	@mkdir -p ../test/firmware
	cp $^ ../test/firmware/

clean:
	rm -rf $(BUILD)

.PHONY: all clean prebuilt
.PRECIOUS: $(BUILD)/%.elf
