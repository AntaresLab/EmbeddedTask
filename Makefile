##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [3.14.1] date: [Wed Oct 27 22:30:58 CEST 2021]
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = max6650_test


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# Build path
BUILD_DIR = out

######################################
# source
######################################
# C sources
C_SOURCES =  \
src/main.c \
src/stm32f4xx_it.c \
src/stm32f4xx_hal_msp.c \
src/system_stm32f4xx.c \
libs/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c \
libs/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c \
libs/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
libs/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
libs/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
libs/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
libs/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
libs/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
libs/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
libs/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
libs/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
libs/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
libs/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
libs/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
libs/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c \
libs/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
libs/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
libs/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c \

# ASM sources
ASM_SOURCES =  \
src/startup_stm32f401xc.s


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32F401xC


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-Iinc \
-Ilibs/STM32F4xx_HAL_Driver/Inc \
-Ilibs/STM32F4xx_HAL_Driver/Inc/Legacy \
-Ilibs/CMSIS/Device/ST/STM32F4xx/Include \
-Ilibs/CMSIS/Include \
-Ilibs/MAX6650/Inc


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = src/STM32F401CBUx_FLASH.ld


# libraries
LIBS = -lc -lm -lnosys -l:max6650.lib
LIBDIR = -Llibs/MAX6650/Out
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@
	
$(BUILD_DIR):
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***