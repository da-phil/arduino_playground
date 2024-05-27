# Weather station

## Introduction

This is an Arduino codebase which allows broadcasting weather related data to make automated watering systems able to decide on the amount of water to use.
Those are the measured quantities so far:

* Temperature (measured by an DHT22 sensor)
* Humidity (measured by an DHT22 sensor)
* Output voltage of a PV panel (0-14.6V)

This data is collected by a Arduino MKR 1000 WiFi board which is connected to a local WiFi network to broadcast the above mentioned measurements over MQTT topics, such as `watering/[dev,prod,v1,v2,...]/[temperature,humidity,heat_index,pv_voltage` to a remote MQTT broker.

The server which runs the MQTT broker also runs a time series database and a bridge component which subscribes to all related topics and populates the database.

## Architecture

Tbd

## Arduino setup

After checking out the code make sure to create a file `arduino_secrets.h` next to the `.ino` project file with the following contents:
```cpp
// Secrets which are not supposed to be commited in version control!
constexpr const char *secrets_ssid = "MyWifiNetwork"; // network SSID (name)
constexpr const char *secrets_pass = "d3Adbeef"; // network password
constexpr const char *secrets_mqtt_username = "watering_station"; // MQTT username
constexpr const char *secrets_mqtt_password = "123456"; // MQTT password

constexpr const char *secrets_mqtt_server_ip = "192.168.0.1"; // IP address of the MQTT broker
constexpr const int secrets_mqtt_server_port = 1883; // Port of MQTT broker
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
```
# Place your local configuration in /etc/mosquitto/conf.d/
#
# A full description of the configuration file is at
# /usr/share/doc/mosquitto/examples/mosquitto.conf.example
```

So we create a new config, e.g. `/etc/mosquitto/conf.d/default.conf`:
```
listener 1883
allow_anonymous false
password_file /etc/mosquitto/pwfile
```

We also need to creat the above mentioned pwfile `/etc/mosquitto/pwfile` in the following form:
```
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

#### Installation

This is the most difficult to install server application, due to the huge variety of versions to choose from and recently deprecated way of installing over package managers.
A commercial and an OSS branch and versions v1, v2 and v3.
I went for the OSS v2 branch and version 2.7.6-1.

The official installation guide encourages people to download binaries in tgz packages instead of using Linux package managers.
However the following page gives clear and (still) working instructions on how to install DEB and RPM packages: https://repos.influxdata.com/debian/.

Here is short summary on the steps:

```
wget -q https://repos.influxdata.com/influxdata-archive_compat.key

echo '393e8779c89ac8d958f81f942f9ad7fb82a25e133faddaf92e15b16e6ac9ce4c influxdata-archive_compat.key' | sha256sum -c && cat influxdata-archive_compat.key | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/influxdata-archive_compat.gpg > /dev/null

echo 'deb [signed-by=/etc/apt/trusted.gpg.d/influxdata-archive_compat.gpg] https://repos.influxdata.com/debian stable main' | sudo tee /etc/apt/sources.list.d/influxdata.list

sudo apt-get update

sudo apt-get install influxdb2 influxdb2-cli
```

#### Configration

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
  ```
  influx config create --config-name telegraf \
                       --host-url http://localhost:8086 --org server \
                       --token $INFLUX_TOKEN
  ```
* Create new account for user `telegraf`:
  ```
  influx auth create --org server  --user telegraf                  \
                     --read-authorizations --write-authorizations   \
                     --read-buckets           --write-buckets       \
                     --read-dashboards        --write-dashboards    \
                     --read-tasks             --write-tasks         \
                     --read-telegrafs         --write-telegrafs
  ```
  * Write down API token for new user

### MQTT to InfluxDB bridge: Telegraf

#### Installation

The Telegraf package is part of the InfluxDB package repository, see [Time series database: InfluxDB](#time-series-database-influxdb).
```
sudo apt-get install telegraf
```


#### Configuration

Here is an example how to configure Telegraf to convert MQTT messages into InfluxDB measurements:
https://github.com/influxdata/telegraf/tree/master/plugins/inputs/mqtt_consumer

There is also a good German blog article about how to configure Telegraf for what we plan to do:

https://plantprogrammer.de/vom-datengrab-zum-datenschatz-mqtt-influxdb-und-grafana-teil-i/

My config `/etc/telegraf/telegraf-watering.conf` looks like:

```toml
[[outputs.influxdb_v2]]
  ## The URLs of the InfluxDB cluster nodes.
  urls = ["http://127.0.0.1:8086"]

  ## Token for authentication ($INFLUX_TOKEN also possible).
  token = "ad82eikd2dkdkfajsdl2dkasdlasdfj=="

  ## Organization is the name of the organization you wish to write to; must exist.
  organization = "server"

  ## Destination bucket to write into.
  bucket = "weather_station"

  ## Topics that will be subscribed to.
  topics = ["watering/#"]
  data_format = "value"
  data_type = "float"

[[inputs.mqtt_consumer]]
  ## Broker URLs for the MQTT server or cluster.
  servers = ["tcp://127.0.0.1:1883"]
  username = "telegraf"
  password = "foobar42"

  ## Enable extracting tag values from MQTT topics
  ## _ denotes an ignored entry in the topic path
  [[inputs.mqtt_consumer.topic_parsing]]
    topic = "watering/+/+"
    measurement = "measurement/_/_"
    tags = "_/branch/_"
    fields = "_/_/field"
```