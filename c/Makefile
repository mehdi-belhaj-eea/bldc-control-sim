# Embedded controller: host build (shared library for the simulation, unit tests) and
# Cortex-M4F build (object sizes).
CC      ?= cc
WARN     = -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wdouble-promotion -Werror
CFLAGS  ?= -std=c99 -O2 $(WARN)
SRC      = src/bldc_math.c src/foc.c src/six_step.c
HDR      = include/bldc_ctrl.h
BUILD    = build
LIB_EXT := $(if $(filter Darwin,$(shell uname -s)),dylib,so)

ARM_CC   ?= arm-none-eabi-gcc
ARM_SIZE ?= arm-none-eabi-size
ARM_FLAGS = -std=c99 -Os -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 \
            -ffunction-sections -fdata-sections -DBLDC_NO_LAYOUT $(WARN)

.PHONY: all lib test arm clean

all: lib test

lib: $(BUILD)/libbldcctrl.$(LIB_EXT)

$(BUILD)/libbldcctrl.$(LIB_EXT): $(SRC) $(HDR) | $(BUILD)
	$(CC) $(CFLAGS) -fPIC -shared -Iinclude $(SRC) -o $@ -lm

$(BUILD)/test_bldc_ctrl: $(SRC) $(HDR) tests/test_bldc_ctrl.c | $(BUILD)
	$(CC) $(CFLAGS) -Iinclude $(SRC) tests/test_bldc_ctrl.c -o $@ -lm

test: $(BUILD)/test_bldc_ctrl
	./$(BUILD)/test_bldc_ctrl

# Cortex-M4 with FPU: compile only (no libc link), print the code size of each object.
arm: | $(BUILD)/arm
	for f in $(SRC); do $(ARM_CC) $(ARM_FLAGS) -Iinclude -c $$f -o $(BUILD)/arm/$$(basename $$f .c).o || exit 1; done
	$(ARM_SIZE) -t $(BUILD)/arm/*.o

$(BUILD) $(BUILD)/arm:
	mkdir -p $@

clean:
	rm -rf $(BUILD)
