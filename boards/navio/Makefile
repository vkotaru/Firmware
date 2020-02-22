
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

INCLUDE_DIRS = $(BOARD_DIR) \
				$(BOARD_SRC_DIR) \
				$(BOARD_SRC_DIR)/Common \
				$(BOARD_SRC_DIR)/Navio2 \
				$(BOARD_SRC_DIR)/Navio+ \
				$(BIN_DIR)
				

BOARD_CXX_SRC = navio_board.cpp \
				main.cpp

VPATH		:= $(VPATH):$(BOARD_SRC_DIR):$(BOARD_SRC_DIR)/Common:$(BOARD_SRC_DIR)/Navio2:$(BOARD_SRC_DIR)/Navio+
NAVIO_CXX_SRC = $(notdir $(wildcard $(BOARD_SRC_DIR)/*/*.cpp))

CXXSOURCES = $(BOARD_CXX_SRC) \
			 $(NAVIO_CXX_SRC)


#################################
# Build
#################################

CC=gcc
CXX=g++ 
CXXFLAGS=-std=c++11 -Wno-psabi -lpigpio $(addprefix -I ,$(INCLUDE_DIRS)) 
# DEPS = $(notdir $(wildcard $(BOARD_SRC_DIR)/*/*.cpp))
OBJ = $(addsuffix .o,$(addprefix $(BIN_DIR)/$(TARGET)/,$(basename $(CXXSOURCES))))


# NAVIO_CXX_SRC=$(wildcard $(BOARD_DIR)/Navio2/C++/Navio/*/*.cpp)
# OBJECTS = $(NAVIO_CXX_SRC:.cpp=.o) 

# %.o: %.cpp
# 	$(CXX) $(CXXFLAGS) -o $@ $< 

# main: $(OBJECTS)
# 	@echo %% $(NAVIO_CXX_SRC)
# 	$(CXX) -c -o $@ $< $(CXXFLAGS)

# all: $(OBJECTS)
# 	@echo %% $(NAVIO_CXX_SRC)
# 	ar rcs libnavio.a $(OBJECTS)

# NAVIO_OBJECTS =$(NAVIO_CXX_SRC:.cpp=.o) \
# 				main.cpp
# %.o: %.cpp
# 	$(CXX) $(CXXFLAGS) -o $@ $< 

# all: $(NAVIO_OBJECTS)
# 	ar rcs $(BOARD_LIB) $(NAVIO_OBJECTS)

# main: $(NAVIO_OBJECTS)
# 	$(CXX) -o $@ $^ $(CXXFLAGS) -libnavio

TARGET_EXE=$(BIN_DIR)/$(TARGET)_$(BOARD)_$(BUILD_TYPE)
	

$(TARGET_EXE): $(OBJ)
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

# CXX = g++
# CXXFLAGS = -std=c++11 -Wno-psabi -c -I .

# SRC=$(wildcard $(BOARD_SRC_DIR)*/*.cpp) \
# 	main.cpp
# OBJECTS = $(SRC:.cpp=.o) 

# %.o: %.cpp
# 	$(CXX) $(CXXFLAGS) -o $@ $< 


# test: $(OBJECTS)
# 	$(CXX) -o $@ $^ $(CXXFLAGS)

# all: $(OBJECTS)
# 	ar rcs libnavio.a $(OBJECTS)

# clean:
# 	rm -f */*.o *.a

