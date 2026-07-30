# USB -> Dante Firmware Makefile
# Builds as BIOS replacement for LiteX SoC

# Derived from this Makefile's own location, NOT hardcoded: the upstream
# avb-aes3 Makefile pinned an absolute /home/lisp/FPGA/avb-aes3/build/... path,
# so a fork silently compiled against the PARENT repo's generated headers
# (csr.h, regions.ld). That is the "stale CSR addr trap" with a second repo
# attached to it. Keep this relative.
FIRMWARE_DIR := $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))
BUILD_DIR    := $(abspath $(FIRMWARE_DIR)/../build/colorlight_i9plus)

include $(BUILD_DIR)/software/include/generated/variables.mak
include $(SOC_DIRECTORY)/software/common.mak

# Our firmware sources.
# Dante Phase 0 dropped srp.o (MSRP), avdecc.o (1722.1) and aaf.o (AVB audio
# format) -- 31,936 B of .text (57%) and the 32,852 B aaf BSS block.
# Parked in _avb_reference/firmware/.
OBJECTS = crt0.o main.o gptp.o mcr.o cfgflash.o config.o cap.o net.o

# Add our source directory to includes
CFLAGS += -I.

# Region sizes read from the generated regions.ld so they cannot drift from the
# SoC. LiteX's own memusage tool is NOT used here: it assumes .text lives in
# `rom`, but since Dante Phase 0.5 the firmware links into `coderam` and rom
# holds only the loader -- memusage would report a nonsense "261% of ROM".
GEN_REGIONS := $(BUILD_DIR)/software/include/generated/regions.ld
region_hex   = $(shell sed -n 's/^[[:space:]]*$(1) :.*LENGTH = \(0x[0-9a-fA-F]*\).*/\1/p' $(GEN_REGIONS))
CODERAM_HEX := $(call region_hex,coderam)
SRAM_HEX    := $(call region_hex,sram)

all: firmware.bin
	@echo "--- firmware memory usage ---"
	@code=$$(riscv64-unknown-elf-size -A firmware.elf | awk '/^\.text|^\.rodata|^\.srodata/ {s+=$$2} END {print s+0}'); \
	 data=$$(riscv64-unknown-elf-size -A firmware.elf | awk '/^\.data|^\.sdata|^\.bss|^\.sbss/ {s+=$$2} END {print s+0}'); \
	 cr=$$(( $(CODERAM_HEX) )); sr=$$(( $(SRAM_HEX) )); \
	 printf "coderam (.text+.rodata) : %6d / %6d bytes (%d%%)\n" $$code $$cr $$((100*code/cr)); \
	 printf "sram    (.data+.bss)    : %6d / %6d bytes (%d%%)  stack: %d bytes\n" \
	        $$data $$sr $$((100*data/sr)) $$((sr-data)); \
	 if [ $$code -gt $$cr ]; then echo "*** .text+.rodata OVERFLOWS coderam"; exit 1; fi; \
	 if [ $$data -gt $$sr ]; then echo "*** .data+.bss OVERFLOWS sram"; exit 1; fi

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@
ifneq ($(OS),Windows_NT)
	chmod -x $@
endif
ifeq ($(CPUENDIANNESS),little)
	$(PYTHON) -m litex.soc.software.crcfbigen $@ --little
else
	$(PYTHON) -m litex.soc.software.crcfbigen $@
endif

firmware.elf: $(OBJECTS)
	$(CC) $(LDFLAGS) -T linker.ld -N -o $@ \
		$(OBJECTS) \
		$(PACKAGES:%=-L$(BUILD_DIR)/software/%) \
		-Wl,--start-group \
		$(LIBS:lib%=-l%) \
		-Wl,--end-group \
		-Wl,--gc-sections \
		-Wl,-Map,$@.map
ifneq ($(OS),Windows_NT)
	chmod -x $@
endif

# Pull crt0.S from VexRiscv CPU directory
VPATH = $(CPU_DIRECTORY)

%.o: %.c
	$(compile)

%.o: %.S
	$(assemble)

clean:
	$(RM) $(OBJECTS) $(OBJECTS:.o=.d) firmware.elf firmware.bin firmware.elf.map .*~ *~

.PHONY: all clean
