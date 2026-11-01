# Arduino playground

Arduino sketches for SAMD21 boards with WiFi (Arduino Nano 33 IoT, MKR WiFi 1010, MKR1000):

| Folder                                     | Description                                                        |
|--------------------------------------------|--------------------------------------------------------------------|
| [`read_weather_data`](read_weather_data)   | Weather station which sends its measurements over MQTT             |
| [`DHT22`](DHT22)                           | Test sketch for a DHT22 temperature and humidity sensor            |
| [`bme_280_test`](bme_280_test)             | Test sketch for a BME280 temperature, humidity and pressure sensor |
| [`simple_adc_reading`](simple_adc_reading) | Reads the voltage on pin A0                                        |

## Building

### Prerequisites

* [arduino-cli](https://arduino.github.io/arduino-cli/latest/installation/) and `make`
* Optional, for the unit tests: `cmake` (3.13 or newer) and a C++17 compiler
* Optional, for the code quality checks: [pre-commit](https://pre-commit.com), e.g. `pipx install pre-commit`.
  Run `pre-commit install` once in the repository to check every commit. pre-commit downloads the pinned
  `clang-format` version itself.

You don't need to install any Arduino libraries by hand. Each sketch folder contains a `sketch.yaml`, which
pins the board platform and the exact library versions per board, in so-called build profiles:

| Profile       | Board                         |
|---------------|-------------------------------|
| `nano_33_iot` | Arduino Nano 33 IoT (default) |
| `mkrwifi1010` | Arduino MKR WiFi 1010         |
| `mkr1000`     | Arduino MKR1000               |

On the first build `arduino-cli` downloads everything listed in the selected profile into its own cache, so
it doesn't interfere with libraries you installed for other projects. If you use the Arduino IDE instead,
install the platform and library versions listed in the sketch's `sketch.yaml` with the Boards Manager and
the Library Manager.

### Commands

Run these commands in the repository root. Find the serial port of a connected board with
`arduino-cli board list`.

```bash
make                                  # compile read_weather_data for the Nano 33 IoT
make SKETCH=DHT22                     # compile another sketch
make PROFILE=mkr1000                  # compile for another board
make upload PORT=/dev/ttyACM0         # compile and upload
make monitor PORT=/dev/ttyACM0        # show the serial output
make all                              # compile every sketch for every board
make test                             # build and run the unit tests on this computer
make check                            # run the pre-commit checks: formatting, whitespace, file checks
make list                             # list all sketches and their profiles
make help                             # list all targets
```

You can also run `make` inside a sketch folder: there it builds that sketch, without `SKETCH=`. For example,
`make upload PORT=/dev/ttyACM0` in `DHT22/` uploads the DHT22 sketch. In a sketch folder, `make all-profiles`
builds that sketch for every board, while `make all` still builds all sketches.

Unit tests run on this computer, not on the board. A sketch has unit tests if its folder contains a `test`
folder; they are built with CMake from the `CMakeLists.txt` in the sketch folder. Currently that's
`read_weather_data`, with its tests in `read_weather_data/test`.

Build output goes to the `build/` folder in the repository root. Without `make`, the same build works with
e.g. `arduino-cli compile --profile nano_33_iot read_weather_data`.

To add a new sketch, give its folder a `sketch.yaml` with the same profile names, and `make` picks it up.
