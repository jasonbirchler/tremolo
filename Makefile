# Project Name
TARGET = tremolo

#APP_TYPE = BOOT_SRAM

# Compiler options
OPT = -O1
#OPT=-Os

# Sources
CPP_SOURCES = src/tremolo.cpp src/tremolo_engine.cpp

# Library Locations
LIBDAISY_DIR = libDaisy
DAISYSP_DIR = DaisySP

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

# Project include paths
C_INCLUDES += -Iinclude -Isrc

# Host-side engine tests (no Daisy dependencies)
HOST_CXX ?= c++
TEST_DIR  = build/host
TEST_BIN  = $(TEST_DIR)/tremolo_engine_test
TEST_SRC  = test/tremolo_engine_test.cpp src/tremolo_engine.cpp

.PHONY: test
test: $(TEST_BIN)
	./$(TEST_BIN)

$(TEST_BIN): $(TEST_SRC) src/tremolo_engine.h
	@mkdir -p $(TEST_DIR)
	$(HOST_CXX) -std=c++14 -O2 -Wall -Wextra -Isrc \
		$(TEST_SRC) -o $@
