# Build, upload and test the sketches in this repository with arduino-cli.
#
# Every sketch folder contains a sketch.yaml, which pins the board platform and all library
# versions per board ("profile"). arduino-cli downloads them automatically on the first build,
# nothing to install by hand.
#
#   make                       compile read_weather_data for the Nano 33 IoT
#   make SKETCH=<folder>       compile another sketch, e.g. make SKETCH=DHT22
#   make upload PORT=<port>    compile and upload to the board
#   make monitor PORT=<port>   show the serial output of the board
#   make all-profiles          compile one sketch for every board in its sketch.yaml
#   make all                   compile every sketch for every board
#   make test                  build and run the unit tests of a sketch on this computer
#   make check                 run the pre-commit checks (formatting, whitespace, file checks) on all files
#   make list                  list all sketches and their profiles
#   make clean                 remove all build output
#
# Options: SKETCH=<sketch folder>, PROFILE=<profile from its sketch.yaml>,
#          PORT=<serial port, see `arduino-cli board list`>
#
# The same targets also work inside a sketch folder: its Makefile passes them on to this one
# with SKETCH set to that folder.

ARDUINO_CLI ?= arduino-cli
PRE_COMMIT ?= pre-commit
SKETCH ?= read_weather_data
PROFILE ?= nano_33_iot
PORT ?= /dev/ttyACM0
BAUDRATE ?= 9600
BUILD_DIR ?= build

# A folder is a sketch if it contains a sketch.yaml
SKETCHES := $(patsubst %/sketch.yaml,%,$(wildcard */sketch.yaml))
# Profile names are the keys indented by two spaces in a sketch.yaml
profiles_of = $(shell sed -n 's/^  \([A-Za-z0-9_-]*\):$$/\1/p' $(1)/sketch.yaml)

# Sketches with a secrets template can't be built before the secrets file was created
SECRETS = $(patsubst %.template,%,$(wildcard $(SKETCH)/arduino_secrets.h.template $(SKETCH)/src/arduino_secrets.h.template))
COMPILE = $(ARDUINO_CLI) compile --profile $(PROFILE) --build-path $(BUILD_DIR)/$(SKETCH)/$(PROFILE)
TEST_BUILD_DIR = $(BUILD_DIR)/$(SKETCH)/tests

.DEFAULT_GOAL := compile
.PHONY: compile upload monitor all-profiles all test check list clean help check-sketch

compile: check-sketch $(SECRETS)
	$(COMPILE) $(SKETCH)

upload: check-sketch $(SECRETS)
	$(COMPILE) --upload --port $(PORT) $(SKETCH)

monitor:
	$(ARDUINO_CLI) monitor --port $(PORT) --config baudrate=$(BAUDRATE)

all-profiles: check-sketch
	@for profile in $(call profiles_of,$(SKETCH)); do \
		echo "=== $(SKETCH) ($$profile)"; \
		$(MAKE) --no-print-directory compile SKETCH=$(SKETCH) PROFILE=$$profile || exit 1; \
	done

all:
	@for sketch in $(SKETCHES); do \
		$(MAKE) --no-print-directory all-profiles SKETCH=$$sketch || exit 1; \
	done

# Unit tests run on this computer. A sketch has unit tests if it contains a test/ folder,
# they are built with CMake from the CMakeLists.txt in the sketch folder.
test: check-sketch
	@if [ ! -d $(SKETCH)/test ]; then \
		echo "$(SKETCH) has no unit tests"; \
		exit 0; \
	fi; \
	if [ ! -f $(SKETCH)/CMakeLists.txt ]; then \
		echo "$(SKETCH)/test exists, but $(SKETCH)/CMakeLists.txt to build the tests is missing" >&2; \
		exit 1; \
	fi; \
	cmake -S $(SKETCH) -B $(TEST_BUILD_DIR) -DCMAKE_BUILD_TYPE=Debug && \
	cmake --build $(TEST_BUILD_DIR) && \
	cd $(TEST_BUILD_DIR) && ctest --output-on-failure

# Runs the pre-commit checks from .pre-commit-config.yaml on all files, like the CI does.
# clang-format fixes the formatting in place, so a failed run leaves the corrected files behind.
check:
	@command -v $(firstword $(PRE_COMMIT)) >/dev/null || { echo "pre-commit is missing, install it with: pipx install pre-commit" >&2; exit 1; }
	$(PRE_COMMIT) run --all-files

list:
	@$(foreach sketch,$(SKETCHES),echo "$(sketch): $(call profiles_of,$(sketch))";)

clean:
	rm -rf $(BUILD_DIR)

help:
	@sed -n 's/^#   \(make\)/\1/p' Makefile

check-sketch:
	@test -f $(SKETCH)/sketch.yaml || { echo "Unknown sketch '$(SKETCH)', available: $(SKETCHES)" >&2; exit 1; }

%/arduino_secrets.h:
	@echo "$@ is missing: copy $@.template to $@" >&2
	@echo "and fill in your WiFi and MQTT credentials." >&2
	@exit 1
