# Loader Makefile — Dante Phase 0.5
#
# Produces loader.bin, which avb_soc.py bakes into the 8 KB BRAM ROM.
#
# THE LOADER IS FROZEN. Rebuilding it changes ROM contents and therefore
# re-rolls nextpnr placement, which is exactly the cost Phase 0.5 exists to
# avoid. Change it only when you must, and re-sweep seeds + hardware-test USB
# afterwards. Application firmware changes do NOT touch this.

LOADER_DIR := $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))
BUILD_DIR  := $(abspath $(LOADER_DIR)/../../build/colorlight_i9plus)

include $(BUILD_DIR)/software/include/generated/variables.mak
include $(SOC_DIRECTORY)/software/common.mak

# cfgflash.o is reused from ../ rather than reimplemented: it encodes the
# STARTUPE2 warm-up and the 40-bit top-aligned auto-CS transfer convention,
# both hardware-proven and easy to get subtly wrong.
OBJECTS = crt0.o loader.o cfgflash.o

CFLAGS += -I. -I.. -Os

# ROM budget is read from the generated regions.ld, not hardcoded, so it cannot
# drift from integrated_rom_size in avb_soc.py.
# sed out the hex length, then let the shell evaluate it ($(( )) understands 0x).
# Avoids awk's strtonum, which only exists in gawk.
ROM_HEX   := $(shell sed -n 's/^[[:space:]]*rom :.*LENGTH = \(0x[0-9a-fA-F]*\).*/\1/p' \
               $(BUILD_DIR)/software/include/generated/regions.ld)
ROM_BYTES := $(shell echo $$(( $(ROM_HEX) )))

all: loader.bin
	@echo "--- loader size (ROM budget $(ROM_BYTES) bytes, from regions.ld) ---"
	@riscv64-unknown-elf-size loader.elf
	@s=$$(stat -c%s loader.bin); b=$(ROM_BYTES); \
	 echo "loader.bin = $$s bytes ($$((100*s/b))% of $$((b/1024)) KB ROM)"; \
	 if [ $$s -gt $$b ]; then \
	   echo "*** OVER BUDGET: shrink the loader or raise integrated_rom_size"; \
	   exit 1; \
	 fi

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@
ifneq ($(OS),Windows_NT)
	chmod -x $@
endif

# Link with $^, not $(OBJECTS): VPATH resolves targets as well as prerequisites,
# so crt0.o and cfgflash.o may legitimately live in ../ (already built by the
# firmware). $(OBJECTS) would pass bare names the linker cannot find; $^ carries
# the VPATH-resolved paths.
loader.elf: $(OBJECTS)
	$(CC) $(LDFLAGS) -T linker.ld -N -o $@ \
		$^ \
		$(PACKAGES:%=-L$(BUILD_DIR)/software/%) \
		-Wl,--start-group \
		$(LIBS:lib%=-l%) \
		-Wl,--end-group \
		-Wl,--gc-sections \
		-Wl,-Map,$@.map
ifneq ($(OS),Windows_NT)
	chmod -x $@
endif

# crt0.S from the VexRiscv CPU directory; cfgflash.c from the firmware dir.
VPATH = $(CPU_DIRECTORY):$(LOADER_DIR)/..

%.o: %.c
	$(compile)

%.o: %.S
	$(assemble)

clean:
	$(RM) $(OBJECTS) $(OBJECTS:.o=.d) loader.elf loader.bin loader.elf.map .*~ *~

.PHONY: all clean
