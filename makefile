#######################
# Directory shortcut
#######################
STM32-SDK=$(SDK_PATH)/Drivers

#######################
# Output directories
#######################

# Check if .config file is present and print error if not
ifeq (, $(wildcard .config))
    $(error ERROR: '.config' is missing. Please create '.config' file based on default.config as template.)
endif

-include .config

#######################
# Binaries
#######################
PREFIX=arm-none-eabi-
CC=$(GCC_PATH)/$(PREFIX)gcc
AS=$(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP=$(GCC_PATH)/$(PREFIX)objcopy
SZ=$(GCC_PATH)/$(PREFIX)size

#######################
# CFLAGS
#######################
