PROJECT_NAME     := openthread_lib_libopenthread_cli_mtd
TARGETS          := libopenthread_cli_mtd
OUTPUT_DIRECTORY := _build

SDK_ROOT := ../../../../../../..
PROJ_DIR := ../../../..


# Source files common to all targets
SRC_FILES += \
  $(PROJ_DIR)/openthread/src/cli/cli.cpp \
  $(PROJ_DIR)/openthread/src/cli/cli_coap.cpp \
  $(PROJ_DIR)/openthread/src/cli/cli_console.cpp \
  $(PROJ_DIR)/openthread/src/cli/cli_dataset.cpp \
  $(PROJ_DIR)/openthread/src/cli/cli_uart.cpp \
  $(PROJ_DIR)/openthread/src/cli/cli_udp_example.cpp \

# Include folders common to all targets
INC_FOLDERS += \
  $(PROJ_DIR)/openthread/examples/platforms/nrf52840 \
  $(PROJ_DIR)/openthread/third_party/mbedtls/repo.patched/include/mbedtls \
  $(PROJ_DIR)/openthread/third_party/mbedtls \
  $(PROJ_DIR)/openthread/examples/platforms/nrf52840/crypto \
  $(PROJ_DIR)/openthread/third_party/mbedtls/repo.patched/include \
  $(PROJ_DIR)/openthread/src \
  $(PROJ_DIR)/openthread/src/core \
  $(PROJ_DIR)/openthread/src/cli \
  $(PROJ_DIR)/openthread/include \
  $(PROJ_DIR) \

# Libraries common to all targets
LIB_FILES += \

# Optimization flags
OPT = -Os -g3
# Uncomment the line below to enable link time optimization
#OPT += -flto

# C flags common to all targets
CFLAGS += $(OPT)
CFLAGS += -DHAVE_CONFIG_H
CFLAGS += -DMBEDTLS_CONFIG_FILE="\"mbedtls-config.h\""
CFLAGS += -DMBEDTLS_USER_CONFIG_FILE="\"nrf52840-mbedtls-config.h\""
CFLAGS += -DOPENTHREAD_CONFIG_LOG_OUTPUT=OPENTHREAD_CONFIG_LOG_OUTPUT_PLATFORM_DEFINED
CFLAGS += -DOPENTHREAD_MTD=1
CFLAGS += -DOPENTHREAD_PROJECT_CORE_CONFIG_FILE="\"openthread-core-nrf52840-config.h\""
CFLAGS += -D_BSD_SOURCE=1
CFLAGS += -D_DEFAULT_SOURCE=1
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS +=  -Wall -Werror
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# keep every function in a separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin -fshort-enums 

# C++ flags common to all targets
CXXFLAGS += $(OPT)

# Assembler flags common to all targets
ASMFLAGS += -g3
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
ASMFLAGS += -DHAVE_CONFIG_H
ASMFLAGS += -DMBEDTLS_CONFIG_FILE="\"mbedtls-config.h\""
ASMFLAGS += -DMBEDTLS_USER_CONFIG_FILE="\"nrf52840-mbedtls-config.h\""
ASMFLAGS += -DOPENTHREAD_CONFIG_LOG_OUTPUT=OPENTHREAD_CONFIG_LOG_OUTPUT_PLATFORM_DEFINED
ASMFLAGS += -DOPENTHREAD_MTD=1
ASMFLAGS += -DOPENTHREAD_PROJECT_CORE_CONFIG_FILE="\"openthread-core-nrf52840-config.h\""
ASMFLAGS += -D_BSD_SOURCE=1
ASMFLAGS += -D_DEFAULT_SOURCE=1



# Add standard libraries at the very end of the linker input, after all objects
# that may need symbols provided by these libraries.
LIB_FILES += -lc -lnosys -lm


.PHONY: default help

# Default target - first one defined
default: libopenthread_cli_mtd

# Print all targets that can be built
help:
	@echo following targets are available:
	@echo		libopenthread_cli_mtd

TEMPLATE_PATH := $(SDK_ROOT)/components/toolchain/gcc


include $(TEMPLATE_PATH)/Makefile.common

$(call define_library, $(TARGETS), $(PROJ_DIR)/lib/openthread_cli/mtd/armgcc/libopenthread-cli-mtd.a)

define create_library
@echo Creating library: $($@)
$(NO_ECHO)$(AR) $($@) $^
@echo Done
endef
libopenthread_cli_mtd:
	$(create_library)

SDK_CONFIG_FILE := ../config/sdk_config.h
CMSIS_CONFIG_TOOL := $(SDK_ROOT)/external_tools/cmsisconfig/CMSIS_Configuration_Wizard.jar
sdk_config:
	java -jar $(CMSIS_CONFIG_TOOL) $(SDK_CONFIG_FILE)
