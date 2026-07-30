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
OBJECTS = crt0.o main.o gptp.o mcr.o cfgflash.o config.o cap.o osc.o

# Add our source directory to includes
CFLAGS += -I.

all: firmware.bin
	$(PYTHON) -m litex.soc.software.memusage firmware.elf $(BUILD_DIR)/software/include/generated/regions.ld $(TRIPLE)

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
