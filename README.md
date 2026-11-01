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
* Optional, for the unit tests: `cmake` and a C++ compiler
* Optional, for formatting the code: `clang-format` (the CI uses version 21)

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
make test                             # run the unit tests on this computer
make list                             # list all sketches and their profiles
make help                             # list all targets
```

Build output goes to the `build/` folder. Without `make`, the same build works with e.g.
`arduino-cli compile --profile nano_33_iot read_weather_data`.

To add a new sketch, give its folder a `sketch.yaml` with the same profile names, and `make` picks it up.
