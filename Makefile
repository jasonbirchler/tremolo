# Project Name
.DEFAULT_GOAL := all
TARGET = tremolo

#APP_TYPE = BOOT_SRAM

# Compiler options
OPT = -O1
#OPT=-Os

# Sources
CPP_SOURCES = src/tremolo.cpp src/pedal.cpp src/calibration_state.cpp src/led_controller.cpp src/midi_control.cpp src/midi_mapper.cpp src/tremolo_engine.cpp src/vibrato.cpp

# Library Locations
LIBDAISY_DIR = libDaisy
DAISYSP_DIR = DaisySP

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

.PHONY: patch-libdaisy libs

patch-libdaisy:
	@test -f mod/daisy_petal.h && test -f mod/daisy_petal.cpp && test -d libDaisy/src
	@for file in daisy_petal.h daisy_petal.cpp; do \
		if ! cmp -s "mod/$$file" "libDaisy/src/$$file"; then \
			cp "mod/$$file" "libDaisy/src/$$file" || exit 1; \
		fi; \
	done

libs: patch-libdaisy
	$(MAKE) -C DaisySP
	$(MAKE) -C libDaisy

all: libs
$(OBJECTS): | libs
$(LIBDAISY_DIR)/build/libdaisy.a $(DAISYSP_DIR)/build/libdaisysp.a: | libs
$(BUILD_DIR)/$(TARGET).elf: $(LIBDAISY_DIR)/build/libdaisy.a $(DAISYSP_DIR)/build/libdaisysp.a

# Project include paths
C_INCLUDES += -Iinclude -Isrc

# Host-side engine tests (no Daisy dependencies)
HOST_CXX ?= c++
TEST_DIR  = build/host
TEST_BIN  = $(TEST_DIR)/tremolo_engine_test
TEST_SRC  = test/tremolo_engine_test.cpp src/tremolo_engine.cpp

.PHONY: test
test: $(TEST_BIN) $(TRIPLE_TEST_BIN)
	./$(TEST_BIN)
	./$(TRIPLE_TEST_BIN)

TRIPLE_TEST_BIN = $(TEST_DIR)/triple_buffer_test
$(TRIPLE_TEST_BIN): test/triple_buffer_test.cpp src/midi_bridge.h src/tremolo_engine.h src/tremolo_engine.cpp
	@mkdir -p $(TEST_DIR)
	$(HOST_CXX) -std=c++14 -O2 -Wall -Wextra -pthread -Isrc \
		test/triple_buffer_test.cpp src/tremolo_engine.cpp -o $@

$(TEST_BIN): $(TEST_SRC) src/tremolo_engine.h
	@mkdir -p $(TEST_DIR)
	$(HOST_CXX) -std=c++14 -O2 -Wall -Wextra -Isrc -I$(DAISYSP_DIR)/Source \
		$(TEST_SRC) -o $@
