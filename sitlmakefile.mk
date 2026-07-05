MCU := SITL

MCU_LC := $(call lc,$(MCU))

TARGETS_$(MCU) := $(call get_targets,$(MCU))

HAL_FOLDER_$(MCU) := $(HAL_FOLDER)/$(MCU)

# native build using the host compiler
SITL_CC := gcc
SITL_OBJCOPY := objcopy
NATIVE_$(MCU) := 1

MCU_$(MCU) :=
LDSCRIPT_$(MCU) :=

SRC_DIR_$(MCU) := \
	$(HAL_FOLDER_$(MCU))/Src \
	$(HAL_FOLDER_$(MCU))/sim

CFLAGS_$(MCU) := \
	-I$(HAL_FOLDER_$(MCU))/Inc \
	-I$(HAL_FOLDER_$(MCU))/sim

CFLAGS_$(MCU) += -D_GNU_SOURCE

# native compiler flags, replacing the ARM specific CFLAGS_COMMON. Inc is
# searched via -iquote rather than -I so that Inc/signal.h does not shadow
# the system <signal.h>
CFLAGS_COMMON_$(MCU) := -fsingle-precision-constant -iquote $(MAIN_INC_DIR) -g3 -O2 \
	-Wall -Wundef -Wextra -Werror -Wno-unused-parameter -Wno-stringop-truncation \
	-fno-strict-aliasing -pthread

LDFLAGS_COMMON_$(MCU) := -pthread

LDLIBS_$(MCU) := -lm

SRC_$(MCU) := $(foreach dir,$(SRC_DIR_$(MCU)),$(wildcard $(dir)/*.c))

# optional CAN support
CFLAGS_CAN_$(MCU) = \
	-ISrc/DroneCAN \
	-ISrc/DroneCAN/libcanard \
	-ISrc/DroneCAN/dsdl_generated/include

SRC_DIR_CAN_$(MCU) = Src/DroneCAN \
		Src/DroneCAN/dsdl_generated/src \
		Src/DroneCAN/libcanard

SRC_CAN_$(MCU) := $(foreach dir,$(SRC_DIR_CAN_$(MCU)),$(wildcard $(dir)/*.[cs]))

LDSCRIPT_CAN_$(MCU) :=
