# Build, upload and test the weather station with arduino-cli.
#
# The board platform and all library versions are pinned per board in sketch.yaml.
# arduino-cli downloads them automatically on the first build, nothing to install by hand.
#
#   make                       compile for the default board (PROFILE=nano_33_iot)
#   make upload PORT=<port>    compile and upload to the board
#   make monitor PORT=<port>   show the serial output of the board
#   make all-profiles          compile for every board in sketch.yaml
#   make test                  build and run the unit tests on this computer
#   make format                format the code with clang-format
#   make format-check          check the formatting like the CI does
#   make clean                 remove all build output
#
# Options: PROFILE=<profile from sketch.yaml>, PORT=<serial port, see `arduino-cli board list`>

ARDUINO_CLI ?= arduino-cli
CLANG_FORMAT ?= clang-format
PROFILE ?= nano_33_iot
PORT ?= /dev/ttyACM0
BAUDRATE ?= 9600
# Keep build output out of the sketch folder, in the repository's build/ folder (ignored by git)
BUILD_DIR ?= ../build/read_weather_data

PROFILES := $(shell sed -n 's/^  \([A-Za-z0-9_-]*\):$$/\1/p' sketch.yaml)
SOURCES = $(shell find . \( -iname '*.ino' -o -iname '*.[ch]pp' \) -print)
COMPILE = $(ARDUINO_CLI) compile --profile $(PROFILE) --build-path $(BUILD_DIR)/$(PROFILE)

.PHONY: compile upload monitor all-profiles test format format-check clean help

compile: arduino_secrets.h
	$(COMPILE) .

upload: arduino_secrets.h
	$(COMPILE) --upload --port $(PORT) .

monitor:
	$(ARDUINO_CLI) monitor --port $(PORT) --config baudrate=$(BAUDRATE)

all-profiles: arduino_secrets.h
	@for profile in $(PROFILES); do \
		echo "=== $$profile"; \
		$(MAKE) --no-print-directory compile PROFILE=$$profile || exit 1; \
	done

test:
	cmake -S .. -B $(BUILD_DIR)/tests -DCMAKE_BUILD_TYPE=Debug
	cmake --build $(BUILD_DIR)/tests
	ctest --test-dir $(BUILD_DIR)/tests --output-on-failure

format:
	$(CLANG_FORMAT) -i --style=file $(SOURCES)

format-check:
	$(CLANG_FORMAT) --dry-run --Werror --style=file $(SOURCES)

clean:
	rm -rf $(BUILD_DIR)

help:
	@sed -n 's/^#   //p' Makefile

arduino_secrets.h:
	@echo "arduino_secrets.h is missing: copy arduino_secrets.h.template to arduino_secrets.h" >&2
	@echo "and fill in your WiFi and MQTT credentials." >&2
	@exit 1
