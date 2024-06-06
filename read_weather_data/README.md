# Weather station

## Introduction

This is an Arduino codebase which allows broadcasting weather related data to make automated watering systems able to decide on the amount of water to use.
Those are the measured quantities so far:

* Temperature (measured by an DHT22 sensor)
* Humidity (measured by an DHT22 sensor)
* Output voltage of a PV panel (0-14.6V)

This data is collected by an Arduino MKR 1000 Wi-Fi board which is connected to a local Wi-Fi network to broadcast the above-mentioned measurements over MQTT topics, such as `watering/location/[dev,prod,v1,v2,...]/measurements` to a remote MQTT broker. The message will be in JSON format and contain a UTC Unix epoch timestamp in seconds and all relevant measurements such as temperature, humidity and solar panel voltage.

The server which runs the MQTT broker also runs a time series database and a bridge component which subscribes to all related topics and populates the database.

## Architecture

Tbd

## Arduino setup

### Prerequisites

#### Arduino libraries

In order to use this codebase the following Arduino libs are needed:

* WiFi101
* ArduinoMqttClient
* NTPClient
* RTCZero

If you use a DHT22 sensor you additionally need the `DHT sensor library`, for the more accurate BME280 sensor, which also measures air pressure, you need to
install the `Adafruit BME280` and the `Adafruit Unified Sensor` libraries.

If I didn't completely misunderstand or mess it up, you can use the following command together with the `arduino-cli` tool to install al the above-mentioned libraries by the following command for the current development platform (Arudino MKR1000):

```bash
arduino-cli compile --profile mkrwifi1010
```

If this does not work, you need to install each library individually via the library manager.

#### Credentials for Wi-Fi network and MQTT broker

After checking out the code make sure to adapt `arduino_secrets.h` to your network setup and credentials:

```cpp
// Secrets which are not supposed to be commited in version control!
constexpr const char *SECRETS_WIFI_SSID = "MyWifiNetwork"; // network SSID (name)
constexpr const char *SECRETS_WIFI_PASSWORD = "d3Adbeef"; // network password
constexpr const char *SECRETS_MQTT_USERNAME = "watering_station"; // MQTT username
constexpr const char *SECRETS_MQTT_PASSWORD = "123456"; // MQTT password
constexpr const char *SECRETS_MQTT_SERVER_IP = "192.168.0.1"; // IP address of the MQTT broker
constexpr const int SECRETS_MQTT_SERVER_PORT = 1883; // Port of MQTT broker
```

## Server setup

The following instructions assume a up-to-date debian/ubuntu based Linux distribution.

### MQTT broker: Mosquitto

#### Installation

Install the debian package by: `sudo apt install -y mosquitto`.

Check if the service has been started already: `sudo systemctl status mosquitto`.

Restarting after modification of the configuration is done by: `sudo systemctl restart mosquitto`.

#### Configuration

There are clear instructions in the Mosquitto config (`/etc/mosquitto/mosquitto.conf`) where to place the config:

```ApacheConf
# Place your local configuration in /etc/mosquitto/conf.d/
#
# A full description of the configuration file is at
# /usr/share/doc/mosquitto/examples/mosquitto.conf.example
```

So we create a new config, e.g. `/etc/mosquitto/conf.d/default.conf`:

```ApacheConf
listener 1883
allow_anonymous false
password_file /etc/mosquitto/pwfile
```

We also need to creat the above mentioned pwfile `/etc/mosquitto/pwfile` in the following form:

```ApacheConf
user1:this_is_my_bad_pw
user2:another_weak_123_pw
```

In my case I created an account for:

* Each device which publishes to topics
* Each human user
* Each application which must read msgs

After setting up the credentials, we convert this file to a version with hashed passwords by: `sudo mosquitto_passwd -U /etc/mosquitto/pwfile`.

Finally restarting the service should make the changes effective: `sudo systemctl restart mosquitto`.

### Time series database: InfluxDB

#### InfluxDB Installation

This is the most difficult to install server application, due to the huge variety of versions to choose from and recently deprecated way of installing over package managers.
A commercial and an OSS branch and versions v1, v2 and v3.
I went for the OSS v2 branch and version 2.7.6-1.

The official installation guide encourages people to download binaries in tgz packages instead of using Linux package managers.
However the following page gives clear and (still) working instructions on how to install DEB and RPM packages: <https://repos.influxdata.com/debian/>.

Here is short summary on the steps:

```bash
wget -q https://repos.influxdata.com/influxdata-archive_compat.key

echo '393e8779c89ac8d958f81f942f9ad7fb82a25e133faddaf92e15b16e6ac9ce4c influxdata-archive_compat.key' | sha256sum -c && cat influxdata-archive_compat.key | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/influxdata-archive_compat.gpg > /dev/null

echo 'deb [signed-by=/etc/apt/trusted.gpg.d/influxdata-archive_compat.gpg] https://repos.influxdata.com/debian stable main' | sudo tee /etc/apt/sources.list.d/influxdata.list

sudo apt-get update

sudo apt-get install influxdb2 influxdb2-cli
```

#### InfluxDB Configration

Also configuration of InfluxDB is not that easy as it varies across major versions.
The following instructions have been only tested with influxdb2:

* Make sure web GUI access is enabled in config `/etc/influxdb/influxdb.conf`:

  ```toml
  [http]
  # Determines whether HTTP endpoint is enabled.
  enabled = true
  # The bind address used by the HTTP service.
  bind-address = ":8086"
  # Determines whether user authentication is enabled over HTTP/HTTPS.
  auth-enabled = true
  ```

* Restart influxdb: `sudo service influxdb restart`.
* Access web GUI: `http://<IP-Adress>:8086`
* Create organisation `server` and admin account and write down API token
* Assign admin API token to env variable: `export INFLUX_TOKEN=...`
* Create config:

  ```bash
  influx config create --config-name telegraf \
                       --host-url http://localhost:8086 --org server \
                       --token $INFLUX_TOKEN
  ```

* Create new account for user `telegraf` and write down API token:

  ```bash
  influx auth create --org server  --user telegraf                  \
                     --read-authorizations --write-authorizations   \
                     --read-buckets           --write-buckets       \
                     --read-dashboards        --write-dashboards    \
                     --read-tasks             --write-tasks         \
                     --read-telegrafs         --write-telegrafs
  ```

* Create new account for user `grafana` for read-only access and write down API token:

  ```bash
  influx auth create --org server  --user grafana                   \
                     --read-buckets --read-dashboards --read-tasks           
  ```

### MQTT to InfluxDB bridge: Telegraf

#### Telegraf Installation

The Telegraf package is part of the InfluxDB package repository, see [Time series database: InfluxDB](#time-series-database-influxdb).

```bash
sudo apt-get install telegraf
```

#### Telegraf Configuration

Essentially Telegraf should convert the following JSON schema into influxdb database records:

```json
{
  "timestamp": 1717354252,
  "temperature": 20.80,
  "humidity": 82.00,
  "heat_index": 21.08,
  "pv_voltage": 10.2
}
```

Timestamps are provided as seconds since Unix epoch in UTC time-zone.

Here are some how-to's and examples on how to configure Telegraf to convert MQTT messages into InfluxDB measurements:

* <https://github.com/influxdata/telegraf/tree/master/plugins/inputs/mqtt_consumer>
* <https://www.influxdata.com/blog/how-parse-json-telegraf-influxdb-cloud>
* <https://www.influxdata.com/blog/mqtt-topic-payload-parsing-telegraf>
* <https://docs.influxdata.com/telegraf/v1/data_formats/input/json_v2>

There is also a good German blog article about how to configure Telegraf for what we plan to do:

<https://plantprogrammer.de/vom-datengrab-zum-datenschatz-mqtt-influxdb-und-grafana-teil-i/>

My config `/etc/telegraf/telegraf-watering.conf` looks like:

```toml
[[outputs.influxdb_v2]]
  ## The URLs of the InfluxDB cluster nodes.
  urls = ["http://127.0.0.1:8086"]

  ## Token for authentication ($INFLUX_TOKEN also possible).
  token = "ad82eikd2dkdkfajsdl2dkasdlasdfj=="
  organization = "server"
  bucket = "weather_station"

[[inputs.mqtt_consumer]]
  servers = ["tcp://127.0.0.1:1883"]
  username = "telegraf"
  password = "123456"

  topics = ["watering/#"]
  data_format = "json_v2"

  [[inputs.mqtt_consumer.topic_parsing]]
    topic = "watering/+/+/+"
    measurement = "_/_/_/measurement"
    tags = "_/location/branch/_"

  [[inputs.mqtt_consumer.json_v2]]
    [[inputs.mqtt_consumer.json_v2.object]]
      path = "@this"
      timestamp_key = "timestamp"
      timestamp_format = "unix"
      timestamp_timezone = "UTC"
```

### Grafana

#### Grafana Installation

Follow the description on the official Grafana homepage:
<https://grafana.com/docs/grafana/latest/setup-grafana/installation/debian/>

Installing the latest OSS version (stable release) is shown here in a nutshell:

```bash
# Install the prerequisite packages:
sudo apt-get install -y apt-transport-https software-properties-common wget

# Import the GPG key:
sudo mkdir -p /etc/apt/keyrings/
wget -q -O - https://apt.grafana.com/gpg.key | gpg --dearmor | sudo tee /etc/apt/keyrings/grafana.gpg > /dev/null

# To add a repository for stable releases, run the following command:
echo "deb [signed-by=/etc/apt/keyrings/grafana.gpg] https://apt.grafana.com stable main" | sudo tee -a /etc/apt/sources.list.d/grafana.list

# Updates the list of available packages and install latest OSS release:
sudo apt-get update && sudo apt-get install grafana
```

#### Grafana Configuration

Open `/etc/grafana/grafana.ini` config file and make sure the following config options are set:

```ApacheConf
app_mode = production
instance_name = ${HOSTNAME}

[server]
# Protocol (http, https, h2, socket)
protocol = https
http_port = 3000
```

After updating the config, restart the service by:

```bash
sudo systemctl restart grafana-server.service 
```

Now you should be able to access the web GUI at <https://localhost:3000> and create the initial `admin` account to get started with the remaining configuration steps, such as optionally creating further user accounts and further fine-tuning.

In the `Connections` section you can add the existing influxdb bucket by the following settings:

* Query language: Flux
* HTTP
  * URL: <http://localhost:8086>
* InfluxDB Details
  * Organization: server
  * Token: \<TOKEN-OF-YOUR-GRAFANA-READ-ONLY-USER\>
  * Default Bucket: weather_station
