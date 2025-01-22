##############################################
# Makefile for Semtech radio module sx126x
# Define GCC_PATH (e.g. GCC_PATH=/path/to/gcc/bin) to set compiler path
# Define SM32 SDK path (e.g. SDK_PATH=/path/to/stm32/sdk)
##############################################

# Check if .config file is present and print error if not
ifeq (, $(wildcard .config))
    $(error ERROR: '.config' is missing. Please create '.config' file based on default.config as template.)
endif

-include .config

#######################
# Source directory
#######################
STM32_DRIVERS = $(SDK_PATH)/Drivers
SRCDIR = src

#######################
# Linker script path
#######################
LINKER_SCRIPT = $(SDK_PATH)/Projects/STM32F411RE-Nucleo/Demonstrations/STM32CubeIDE/STM32F411RETX_FLASH.ld

#######################
# Output directory
#######################
OBJDIR = Obj
BUILD = Build

#######################
# Binary
#######################
PREFIX = arm-none-eabi-
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size

#######################
# CFLAGS
#######################
CFLAGS = -mcpu=$(CPU) -mthumb -std=$(STANDARD) -g $(OPT) -MP -MMD -Wall

#######################
# List of directories
#######################
DIRS = $(SRCDIR)/app \
       $(SRCDIR)/board \
       $(SRCDIR)/hal/stm32f4xx \
       $(SRCDIR)/radio \
       $(SRCDIR)/radio/sx126x \
       $(STM32_DRIVERS)/CMSIS/Device/ST/STM32F4xx/Include \
       $(STM32_DRIVERS)/CMSIS/Include

#######################
# List of source files (.c)
#######################
SRC = $(foreach dir, $(DIRS), $(wildcard $(dir)/*.c))

#######################
# List of ASM files (.s)
#######################
ASMS = $(foreach dir, $(DIRS), $(wildcard $(dir)/*.s))

#######################
# List of header files (.h)
#######################
INC = $(foreach dir, $(DIRS), $(addprefix -I, $(dir)))

#######################
# Replace all '*.c' and all '.s' files to '.o'
#######################
COBJS = $(patsubst %.c, $(OBJDIR)/%.o, $(SRC))
ASMOBJS = $(patsubst %.s, $(OBJDIR)/%.o, $(ASMS))

#######################
# List ao all object files
#######################
OBJS = $(COBJS) $(ASMOBJS)

#######################
# Variables
#######################
CPU = cortex-m4
MCU = STM32F411xE
STANDARD = c99
OPT = -O0
DEFINES = -D$(MCU)
RADIO = sx126x
TARGET = $(BUILD)/$(RADIO)
EXEC = $(TARGET).elf
RM = rm -rf

#######################
# LDFLAGS (linker flags)
#######################
LDFLAGS = -T $(LINKER_SCRIPT) -Xlinker --gc-sections -Xlinker \
  -Map="$(TARGET).map" --specs=nano.specs -Wl,--print-memory-usage -o $(TARGET).elf

#######################
# Default action: build all
#######################
all: $(TARGET)
$(TARGET): $(EXEC)

$(EXEC): $(OBJS)
	@mkdir -p $(@D)
	@echo '============='
	@echo 'Linking the target $@'
	@echo '============='
	@mkdir -p $(BUILD)
	$(CC) $(OBJS) $(LDFLAGS) -o $@
	@echo 'Linking Successful !!!'

$(OBJDIR)/%.o: %.c
	@echo '> Building C file $<'
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(DEFINES) $(INC) -c $< -o $@

$(OBJDIR)/%.o: %.s
	@echo '> Building ASM file $<'
	@mkdir -p $(dir $@)
	$(AS) $(CFLAGS) -c $< -o $@
	
#######################
# Dependency
-include $(COBJS:%.o=%.d)
#######################

.PHONY: all clean

clean:
	@echo 'Cleaning...Done!'
	@$(RM) $(OBJDIR) $(BUILD)
	
#debug:
#	@echo 'SRC:' $(SRC)
#	@echo 'INC:' $(INC)
#	@echo 'ASM:' $(ASMS)
#	@echo 'COBJS:' $(COBJS)
#	@echo 'ASMOBJS:' $(ASMOBJS)
#	@echo 'OBJS:' $(OBJS)
#	@echo 'deps:' $(COBJS:%.o=%.d)
