# USB -> Dante Firmware Makefile
# Builds as BIOS replacement for LiteX SoC

# Derived from this Makefile's own location, NOT hardcoded: the upstream
# avb-aes3 Makefile pinned an absolute /home/lisp/FPGA/avb-aes3/build/... path,
# so a fork silently compiled against the PARENT repo's generated headers
# (csr.h, regions.ld). That is the "stale CSR addr trap" with a second repo
# attached to it. Keep this relative.
FIRMWARE_DIR := $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))
# BUILD_DIR IS OVERRIDABLE FROM THE ENVIRONMENT, not just the command line.
# It used to be a plain `:=`, which a command-line `make BUILD_DIR=...` beats but
# an exported BUILD_DIR does not -- so flash.sh, which runs `make -C firmware`,
# had no way to say which gateware tree it was flashing and always compiled
# against build/colorlight_i9plus. That is how an 18-hour run ended up with the
# peak meter and the ring-health opcode silently compiled out (2026-11-07).
ifeq ($(origin BUILD_DIR),undefined)
BUILD_DIR    := $(abspath $(FIRMWARE_DIR)/../build/colorlight_i9plus)
endif
BUILD_DIR    := $(abspath $(BUILD_DIR))

include $(BUILD_DIR)/software/include/generated/variables.mak
include $(SOC_DIRECTORY)/software/common.mak

# Our firmware sources.
# Dante Phase 0 dropped srp.o (MSRP), avdecc.o (1722.1) and aaf.o (AVB audio
# format) -- 31,936 B of .text (57%) and the 32,852 B aaf BSS block.
# Parked in _avb_reference/firmware/.
# Header dependencies. Without -MMD/-MP, editing a .h does NOT rebuild the .c
# that includes it -- which cost a full flash-and-capture cycle chasing a
# timestamp offset that had simply never been recompiled.
DEPFLAGS := -MMD -MP

OBJECTS = crt0.o main.o gptp.o mcr.o cfgflash.o config.o cap.o net.o dante_dev.o dante_msg.o mdns.o dante_arc.o dante_cmc.o dante_info.o ptpv1.o dante_tx.o dante_flows.o dstats.o rx_gate.o mcr_dante.o telem.o osc.o kvm.o

# Add our source directory to includes
CFLAGS += $(DEPFLAGS) -I.

# A CHANGE OF BUILD_DIR INVALIDATES EVERY OBJECT, AND make CANNOT SEE IT.
#
# The .d files name the csr.h of the tree the object was LAST compiled against.
# Point BUILD_DIR somewhere else and that old file is still unchanged, so make
# rebuilds nothing and the objects silently keep the old header's #ifdef
# decisions. On 2026-11-07 that put a firmware on the board whose dstats.o was
# a day old: the build log said CSR_AAF_PKT_RING_LEVEL_RAW_ADDR "in" (true of
# the header) while the object had been compiled without it (true of the
# binary). The report above describes the HEADER; this stamp is what makes it
# describe the BINARY.
#
# So: remember which tree the objects belong to, and start clean when it moves.
BUILD_STAMP := .build_dir
$(shell [ "$$(cat $(BUILD_STAMP) 2>/dev/null)" = "$(BUILD_DIR)" ] || { \
    rm -f $(OBJECTS) $(OBJECTS:.o=.d) firmware.elf firmware.elf.map firmware.bin; \
    printf '%s' "$(BUILD_DIR)" > $(BUILD_STAMP); })

# THE CONSOLE NARRATIVE IS OFF BY DEFAULT. `make DANTE_VERBOSE=1` brings all 203
# printf lines back; quiet they cost nothing, because log.h makes printf a macro
# that never evaluates its arguments, so the format strings never reach .rodata.
# Measured 2026-11-06: coderam 92108 -> 72364 bytes (93% -> 73%).
ifeq ($(DANTE_VERBOSE),1)
CFLAGS += -DDANTE_VERBOSE=1
endif

# Region sizes read from the generated regions.ld so they cannot drift from the
# SoC. LiteX's own memusage tool is NOT used here: it assumes .text lives in
# `rom`, but since Dante Phase 0.5 the firmware links into `coderam` and rom
# holds only the loader -- memusage would report a nonsense "261% of ROM".
GEN_REGIONS := $(BUILD_DIR)/software/include/generated/regions.ld
region_hex   = $(shell sed -n 's/^[[:space:]]*$(1) :.*LENGTH = \(0x[0-9a-fA-F]*\).*/\1/p' $(GEN_REGIONS))
CODERAM_HEX := $(call region_hex,coderam)
SRAM_HEX    := $(call region_hex,sram)

# OPTIONAL FEATURES ARE GUARDED BY #ifdef ON GENERATED CSR NAMES, AND A MISSING
# CSR SILENTLY TAKES THE #else BRANCH.
#
# Measured 2026-11-07: the board ran 18 hours reporting all 48 channels as
# silent and refusing the 'r' ring-health opcode, because BUILD_DIR still
# pointed at a tree whose csr.h was three days old and had neither the peak
# detector nor the ring counters. NOTHING WARNED -- both #else branches are
# legitimate for an older bitstream, so the downgrade is invisible in the build
# log and in the firmware. The only symptom was a grey meter in Dante
# Controller, which looks exactly like a working meter on a silent input.
#
# So every build now names the header it used and what that header switched off.
CSR_FEATURES := CSR_AAF_PKT_RING_LEVEL_RAW_ADDR CSR_AAF_PKT_PEAK_VALUE_ADDR

all: firmware.bin
	@echo "--- gateware headers ---"
	@csr=$(BUILD_DIR)/software/include/generated/csr.h; \
	 echo "BUILD_DIR : $(BUILD_DIR)"; \
	 echo "csr.h     : $$(date -r $$csr '+%F %T' 2>/dev/null || echo MISSING)"; \
	 miss=0; \
	 for sym in $(CSR_FEATURES); do \
	   if grep -q "^#define $$sym " $$csr 2>/dev/null; then \
	     printf "  %-34s in\n" $$sym; \
	   else \
	     printf "  %-34s *** OUT, #else compiled in ***\n" $$sym; miss=1; \
	   fi; \
	 done; \
	 if [ $$miss = 1 ]; then \
	   echo "  ^ that csr.h predates these CSRs: peaks report 255 (silence) and"; \
	   echo "    the 'r' ring-health opcode will not answer. Build against the"; \
	   echo "    tree the LOADED bitstream came from, e.g."; \
	   echo "    make -C firmware BUILD_DIR=\$$PWD/build_seed8"; \
	 fi
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
	$(RM) $(OBJECTS) $(OBJECTS:.o=.d) firmware.elf firmware.bin firmware.elf.map $(BUILD_STAMP) .*~ *~

.PHONY: all clean

-include $(OBJECTS:.o=.d)
