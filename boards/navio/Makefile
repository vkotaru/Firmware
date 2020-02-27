
TARGET	?= rosflight

BOARD ?= NAVIO

DEBUG ?= GDB

SERIAL_DEVICE ?= /dev/ttyUSB0

BOARD_LIB ?= libnavio.a


#################################
# Working directories
#################################
ROSFLIGHT_ROOT		= ../..
BOARD_DIR			= $(ROSFLIGHT_ROOT)/boards/navio
BOARD_SRC_DIR		= $(BOARD_DIR)/Navio2/C++/Navio
BIN_DIR				= $(BOARD_DIR)/build

##########################################
# Rosflight
##########################################
include $(ROSFLIGHT_ROOT)/scripts/rosflight.mk

#################################
# Source Files
#################################

INCLUDE_DIRS += $(BOARD_DIR) \
				$(BOARD_SRC_DIR) \
				$(BOARD_SRC_DIR)/Common \
				$(BOARD_SRC_DIR)/Navio2 \
				$(BOARD_SRC_DIR)/Navio+ \
				$(BIN_DIR)

BOARD_CXX_SRC = navio_board.cpp \
				main.cpp

VPATH		:= $(VPATH):$(BOARD_SRC_DIR):$(BOARD_SRC_DIR)/Common:$(BOARD_SRC_DIR)/Navio2:$(BOARD_SRC_DIR)/Navio+
NAVIO_CXX_SRC = $(notdir $(wildcard $(BOARD_SRC_DIR)/*/*.cpp))

CXXSOURCES += $(BOARD_CXX_SRC) \
			 $(NAVIO_CXX_SRC)


#################################
# Build
#################################

CC=gcc
CXX=g++
CXXFLAGS=-std=c++11 -Wno-psabi -lpigpio -lpthread \
			$(addprefix -I ,$(INCLUDE_DIRS)) $(DEFS) #$(DEBUG_FLAGS) \
			$(CXX_FILE_SIZE_FLAGS) # $(CXX_STRICT_FLAGS)
# DEPS = $(notdir $(wildcard $(BOARD_SRC_DIR)/*/*.cpp))
OBJECTS += $(addsuffix .o,$(addprefix $(BIN_DIR)/$(TARGET)/,$(basename $(CXXSOURCES))))


TARGET_EXE=$(BIN_DIR)/$(TARGET)_$(BOARD)_$(BUILD_TYPE)


$(TARGET_EXE): $(OBJECTS)
	$(CXX) -o $@ $^ $(CXXFLAGS)

$(BIN_DIR)/$(TARGET)/%.o: %.cpp
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	$(CXX) -c -o $@ $< $(CXXFLAGS)


#################################
# Recipes
#################################
.PHONY: all flash clean

clean:
	rm -rf $(BIN_DIR)

