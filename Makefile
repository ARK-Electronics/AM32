
QUIET = @

# tools
CC = $(ARM_SDK_PREFIX)gcc
OBJCOPY = $(ARM_SDK_PREFIX)objcopy
ECHO = echo

# common variables
IDENTIFIER := AM32

# Folders
HAL_FOLDER := Mcu
MAIN_SRC_DIR := Src
MAIN_INC_DIR := Inc

SRC_DIRS_COMMON := $(MAIN_SRC_DIR)

# Working directories
ROOT := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))

# include the rules for OS independence
include $(ROOT)/make/tools.mk

# supported MCU types

MCU_TYPES := E230 F031 F051 F415 F421 G071 L431 G431 V203 G031 A153 SITL

MCU_TYPE := NONE

# Function to include makefile for each MCU type
define INCLUDE_MCU_MAKEFILES
$(foreach MCU_TYPE,$(MCU_TYPES),$(eval include $(call lc,$(MCU_TYPE))makefile.mk))
endef
$(call INCLUDE_MCU_MAKEFILES)

# additional libs
LIBS := -lnosys

# extract version from Inc/version.h
VERSION_MAJOR := $(shell $(FGREP) "define VERSION_MAJOR" $(MAIN_INC_DIR)/version.h | $(CUT) -d" " -f3 )
VERSION_MINOR := $(shell $(FGREP) "define VERSION_MINOR" $(MAIN_INC_DIR)/version.h | $(CUT) -d" " -f3 )

FIRMWARE_VERSION := $(VERSION_MAJOR).$(VERSION_MINOR)

# Compiler options
#
# Global -Os keeps flash/RAM in check on small MCUs (esp. F051). Hot paths
# (RAM_FUNC / selected phase code) force -O3 via attributes or file pragmas —
# a trailing -O3 on the single-shot link line would re-optimize *all* TUs.

CFLAGS_BASE := -fsingle-precision-constant -fomit-frame-pointer -ffast-math
CFLAGS_BASE += -I$(MAIN_INC_DIR) -g3 -Os -ffunction-sections --specs=nosys.specs
CFLAGS_BASE += -Wall -Wundef -Wextra -Werror -Wno-unused-parameter -Wno-stringop-truncation

CFLAGS_COMMON := $(CFLAGS_BASE)

# Hardware-CI performance instrumentation (opt-in, off by default).
# Build with `make <TARGET> HWCI_PERF=1` to emit the hwci_perf RAM struct that
# the hardware-CI harness (see hwci/) reads over SWD. Production/release builds
# leave this unset and are completely unaffected.
ifeq ($(HWCI_PERF),1)
CFLAGS_COMMON += -DHWCI_PERF
endif

# Linker options
LDFLAGS_COMMON := -specs=nano.specs $(LIBS) -Wl,--gc-sections -Wl,--print-memory-usage

# Search source files (top-level Src only — not recursive into DroneCAN/)
SRC_COMMON_ALL := $(foreach dir,$(SRC_DIRS_COMMON),$(wildcard $(dir)/*.[cs]))

# Optional translation units: only linked when the product needs them.
# (Empty #ifdef stubs still cost flash/link time on F051.)
SRC_OPTIONAL_BRUSHED := $(MAIN_SRC_DIR)/brushed.c
SRC_OPTIONAL_HWCI    := $(MAIN_SRC_DIR)/hwci_perf.c
SRC_COMMON_BASE := $(filter-out $(SRC_OPTIONAL_BRUSHED) $(SRC_OPTIONAL_HWCI),$(SRC_COMMON_ALL))

# configure some directories that are relative to wherever ROOT_DIR is located
OBJ := obj
BIN_DIR := $(ROOT)/$(OBJ)

# Function to check for _CAN / _BRUSHED product suffixes in the target name
has_can_suffix = $(findstring _CAN,$1)
has_brushed_suffix = $(findstring BRUSHED,$1)

# find the SVD files
$(foreach MCU,$(MCU_TYPES),$(eval SVD_$(MCU) := $(wildcard $(HAL_FOLDER_$(MCU))/*.svd)))

.PHONY : clean all binary $(foreach MCU,$(MCU_TYPES),$(call lc,$(MCU)))
# Host-native SITL is opt-in (`make AM32_SITL_CAN` / `make sitl`), not part of
# the cross-compiled `make all` matrix used by Linux firmware CI.
ALL_TARGETS := $(foreach MCU,$(filter-out SITL,$(MCU_TYPES)),$(TARGETS_$(MCU)))
all : $(ALL_TARGETS)

# create targets for compiling one mcu type, eg "make f421"
define CREATE_TARGET
$(call lc,$(1)) : $$(TARGETS_$(1))
endef
$(foreach MCU,$(MCU_TYPES),$(eval $(call CREATE_TARGET,$(MCU))))

clean :
	@echo Removing $(OBJ) directory
	@$(RM) -rf $(OBJ)

#####################
# main firmware build
define CREATE_BUILD_TARGET
$(2)_BASENAME = $(BIN_DIR)/$(IDENTIFIER)_$(2)_$(FIRMWARE_VERSION)

# native (SITL) targets build to an executable elf, no bin/hex conversion
$(2) : $$($(2)_BASENAME).$(if $(NATIVE_$(1)),elf,bin)

# get MCU specific compiler, objcopy and link script or use the ARM SDK one
$(eval xCC := $(if $($(MCU)_CC), $($(MCU)_CC), $(CC)))
$(eval xOBJCOPY := $(if $($(MCU)_OBJCOPY), $($(MCU)_OBJCOPY), $(OBJCOPY)))

# Generate bin and hex files from elf
$$($(2)_BASENAME).bin: $$($(2)_BASENAME).elf
	echo building BIN $$@
	@$(ECHO) Generating $$(notdir $$@)
	$(QUIET)$(xOBJCOPY) -O binary $$(<) $$@
	$(QUIET)python3 Src/DroneCAN/set_app_signature.py $$@ $$(<)
	$(QUIET)$(xOBJCOPY) $$(<) -O ihex $$(@:.bin=.hex)
	$(QUIET)$(CP) -f $$(<) $(OBJ)$(DSEP)debug.elf > $(NUL)

# check for CAN support
$(eval xLDSCRIPT := $$(if $$(call has_can_suffix,$$(2)),$(LDSCRIPT_CAN_$(1)),$(LDSCRIPT_$(1))))
$(eval xCFLAGS := $$(if $$(call has_can_suffix,$$(2)),$(CFLAGS_CAN_$(1))))
$(eval xSRC := $$(if $$(call has_can_suffix,$$(2)),$(SRC_CAN_$(1))))

# Per-target app sources: drop brushed/hwci unless the product asks for them
$(eval SRC_APP_$(2) := $(SRC_COMMON_BASE)$(if $(call has_brushed_suffix,$(2)), $(SRC_OPTIONAL_BRUSHED))$(if $(filter 1,$(HWCI_PERF)), $(SRC_OPTIONAL_HWCI)))

# allow an MCU type to override the common compiler/linker flags (used by SITL
# for a native build) and to have no linker script
$(eval xCFLAGS_COMMON := $(if $(CFLAGS_COMMON_$(1)),$(CFLAGS_COMMON_$(1)),$(CFLAGS_COMMON)))
$(eval xLDFLAGS_COMMON := $(if $(LDFLAGS_COMMON_$(1)),$(LDFLAGS_COMMON_$(1)),$(LDFLAGS_COMMON)))

CFLAGS_$(2) = -DAM32_MCU=\"$(MCU)\" $(MCU_$(1)) -D$(2) $(CFLAGS_$(1)) $(xCFLAGS_COMMON) $(xCFLAGS)
LDFLAGS_$(2) = $(xLDFLAGS_COMMON) $(LDFLAGS_$(1)) $(if $(xLDSCRIPT),-T$(xLDSCRIPT))

-include $$($(2)_BASENAME).d

$$($(2)_BASENAME).elf: $$(SRC_APP_$(2)) $$(SRC_$(1)) $(xSRC)
	@$(ECHO) Compiling $$(notdir $$@)
	$(QUIET)$(MKDIR) -p $(OBJ)
	$(QUIET)$(xCC) $$(CFLAGS_$(2)) $$(LDFLAGS_$(2)) -MMD -MP -MF $$(@:.elf=.d) -o $$(@) $$(SRC_APP_$(2)) $$(SRC_$(1)) $(xSRC) $(LDLIBS_$(1))
# we copy debug.elf to give us a constant debug target for vscode
# this means the debug button will always debug the last target built
	$(if $(SVD_$(1)),$(QUIET)$(CP) -f $$(SVD_$(1)) $(OBJ)/debug.svd)
# also copy the openocd.cfg from the MCU directory to obj/openocd.cfg for auto config of Cortex-Debug
# in vscode
	$(if $(NATIVE_$(1)),,$(QUIET)$(CP) -f Mcu$(DSEP)$(call lc,$(1))$(DSEP)openocd.cfg $(OBJ)$(DSEP)openocd.cfg > $(NUL))
endef
$(foreach MCU,$(MCU_TYPES),$(foreach TARGET,$(TARGETS_$(MCU)), $(eval $(call CREATE_BUILD_TARGET,$(MCU),$(TARGET)))))

# include the targets for installing tools
include $(ROOT)/make/tools_install.mk

# useful target to list all of the board targets so you can see what
# make target to use for your board
targets:
	$(QUIET)echo List of targets. To build a target use 'make TARGETNAME'
	$(QUIET)echo $(ALL_TARGETS)

# Static analysis (cppcheck) of the ARK F051 control path. Fails on
# error/warning; style findings are printed but advisory. See
# scripts/cppcheck-ark.sh and scripts/cppcheck-suppressions.txt.
.PHONY : cppcheck
cppcheck:
	$(QUIET)bash scripts/cppcheck-ark.sh

# Build ARK F051 with HWCI_PERF and enforce flash/RAM headroom (F051 is tight).
# -B forces a rebuild so a prior non-HWCI image is not size-checked by mistake.
.PHONY : size-check-ark
size-check-ark:
	$(QUIET)$(MAKE) -B ARK_4IN1_F051 HWCI_PERF=1
	$(QUIET)bash scripts/check-size-ark.sh

