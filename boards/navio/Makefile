###############################################################################
# Generic Makefile Template for C/C++ for use with NAVIO2
#
# Copyright (c) 2020 - (TBD)
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the <organization> nor the
#      names of its contributors may be used to endorse or promote products
#      derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

TARGET = rosflight

BOARD ?= NAVIO

DEBUG ?= GDB

SERIAL_DEVICE ?= /dev/ttyACM0

#################################
# GNU for Navio
#################################
CC_NAVIO=gcc
CXX_NAVIO=g++


#################################
# Working directories
#################################
ROSFLIGHT_ROOT		= ../..
NAVIO_ROOT          = Navio2/C++/Navio
BOARD_DIR       	= $(ROSFLIGHT_ROOT)/boards/navio
NAVIO_DIR           = $(NAVIO_ROOT)
BIN_DIR             = $(BOARD_DIR)/build


#################################
# ROSflight Common Build
#################################
include $(ROSFLIGHT_ROOT)/scripts/rosflight.mk


#################################
# Source Files
#################################
VPATH			:= $(VPATH):$(NAVIO_DIR)
# # Make a list of source files and includes
# NAVIO_SRCS = led.cpp \
#                  gpio.cpp \
#                  spi.cpp \
#                  vcp.cpp \
#                  led.cpp \
#                  mpu6000.cpp \
#                  advanced.cpp \
#                  i2c.cpp \
#                  pwm.cpp \
#                  mb1242.cpp \
#                  eeprom.cpp \
#                  hmc5883l.cpp \
#                  ms5611.cpp \
#                  rc_ppm.cpp \
#                  rc_sbus.cpp \
#                  uart.cpp \
#                  M25P16.cpp \
#                  ms4525.cpp \
#                  backup_sram.cpp


# board-specific source files
VPATH			:= $(VPATH):$(BOARD_DIR)
BOARD_CXX_SRC   = main.cpp

# Addd Navio CXX Sources
CXXSOURCES += $(BOARD_CXX_SRC)

# Add Navio Include Directories
INCLUDE_DIRS += $(NAVIO_DIR)

# #################################
# # Flags
# #################################

# MCFLAGS= # -mcpu=cortex-m4 -mthumb -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -Wdouble-promotion
# DEFS +=# -DSTM32F40_41xxx -D__CORTEX_M4 -D__FPU_PRESENT -DWORDS_STACK_SIZE=200 -DUSE_STDPERIPH_DRIVER -D__FPU_USED -DHSE_VALUE=8000000 -DUSE_USB_OTG_FS
# CFLAGS_NAVIO  += #$(MCFLAGS) $(OPTIMIZE) $(addprefix -I,$(INCLUDE_DIRS))
# CXXFLAGS_NAVIO  += #$(MCFLAGS) $(OPTIMIZE) $(addprefix -I,$(INCLUDE_DIRS))
# LDFLAGS_NAVIO = #-T $(LDSCRIPT) $(MCFLAGS)  -lc $(ARCH_FLAGS)  $(LTO_FLAGS)  $(DEBUG_FLAGS) -static  -Wl,-gc-sections -Wl,-Map=main.map

# #################################
# # Build
# #################################



# $(TARGET_ELF): $(OBJECTS)
# 	$(CXX_NAVIO) -o $@ $^ $(LDFLAGS_NAVIO)
# 	$(SIZE) $(TARGET_ELF)

# $(BIN_DIR)/$(TARGET)/%.o: %.cpp
# 	@mkdir -p $(dir $@)
# 	@echo %% $(notdir $<)
# 	@$(CXX_NAVIO) -c -o $@ $(CXXFLAGS_NAVIO) $<

# $(BIN_DIR)/$(TARGET)/%.o: %.c
# 	@mkdir -p $(dir $@)
# 	@echo %% $(notdir $<)
# 	@$(CC_NAVIO) -c -o $@ $(CFLAGS_NAVIO) $<

# $(BIN_DIR)/$(TARGET)/%.o: %.s
# 	@mkdir -p $(dir $@)
# 	@echo %% $(notdir $<)
# 	@$(CC_NAVIO) -c -o $@ $(CFLAGS_NAVIO) $<


#################################
# Recipes
#################################
.PHONY: all flash clean test

clean:
	rm -rf $(OBJECTS) $(BIN_DIR)

flash: $(TARGET_BIN)
	dfu-util -a 0 -s 0x08000000 -D $(TARGET_BIN) -R

test:
	@echo "Building version $(CXX)"