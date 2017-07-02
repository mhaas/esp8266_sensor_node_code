#include "wlan_secret.h"

/*
   BEGIN config constants
 */

//  74880 shows the boot messages
#define SERIAL_BAUD 74880

// Observed voltage divided by ADC reading
#define SENSOR_BAT_EXTERNAL_CONSTANT 4.26 / 509;

/*
   END config constants
 */

const int PIN_GNDSW_ENABLE = 15;

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Ticker.h>

#include <ESP8266httpUpdate.h>


extern "C"{
#include "user_interface.h"
}

#ifdef SENSOR_BAT_INTERNAL
ADC_MODE(ADC_VCC);
#else
ADC_MODE(ADC_TOUT);
#endif

Ticker sleepTicker;


int start_up = millis();

boolean sensor_si1145 = SENSOR_SI1145;;
boolean sensor_dht22 = SENSOR_DHT22;
boolean sensor_bmp085 = SENSOR_BMP085;
boolean sensor_chirp = SENSOR_CHIRP;
boolean sensor_bat = SENSOR_BAT;


#include <Wire.h>
#include "Adafruit_SI1145.h"
#include <Adafruit_Sensor.h>
Adafruit_SI1145 sensor = Adafruit_SI1145();
boolean sensor_ok = false;
uint16_t uv = 0;
uint16_t ir = 0;
uint16_t visible = 0;

#include <Adafruit_BMP085_U.h>
// The BMP085 driver - or rather the BME280 sensor - has several
// interesting power management options. The driver operators
// the sensor in "normal" mode, which means the sensor is continuously
// performing measurements according to some internal configuration
// The oversampling settings for pressure
// can be configured in the device - the single parameter to begin()
// is a shortcut to that. Note that the driver does not set the recommended x2
// temperature oversampling for default BMP085_MODE_ULTRAHIGHRES.

// The constants for the pressure oversampling also are completely wrong.


// The driver currently does not expose a method to set
// the internal polling rate.

// In any case, I measured ~100uA for the sensor + breakout in normal mode
// That seems to be a lot more than the numbers in the datasheet would indicate,
// but since we're nowadays turning off the sensor, it does not matter
// The breakout possibly contains a vreg which may account for the difference
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
boolean bmp_ok = false;
float pressure = 0;
float temperature = 0;

// DHT22
float dht_humidity = 0;
float dht_temperature = 0;
float dht_heat_index = 0;

#include "DHT.h"
DHT dht(PIN_DHT22, DHT22);

#include <I2CSoilMoistureSensor.h>
I2CSoilMoistureSensor chirp;

float chirp_moisture = 0;

WiFiClient wifiClient;
PubSubClient client(wifiClient, MQTT_HOST);

void publish(String name, float val);
void checkUpdate();


void mqttCallback(const MQTT::Publish& pub) {
    String givenTopic = pub.topic();
    String updateTopic = String("/") + String(NODE_NAME) + String("check_update");
    if (givenTopic.equals(updateTopic)) {
        checkUpdate();
    }

}

void checkUpdate() {
    String url = String("http://") + String(UPDATE_SERVER) + String("/") + String(NODE_NAME) + String(".bin");
    String currentVersion = String(BUILD_TIMESTAMP);
    ESPhttpUpdate.rebootOnUpdate(true);
    t_httpUpdate_return ret = ESPhttpUpdate.update(url, currentVersion);
            switch(ret) {
            case HTTP_UPDATE_FAILED:
                Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
                break;

            case HTTP_UPDATE_NO_UPDATES:
                Serial.println("HTTP_UPDATE_NO_UPDATES");
                break;

            case HTTP_UPDATE_OK:
                Serial.println("HTTP_UPDATE_OK");
                break;
        }

}

void connectWifi()  {
    IPAddress node_ip(192, 168, 1, 61);
    IPAddress node_gateway(192, 168, 1, 1);
    IPAddress node_subnet(255, 255, 255, 0);


   WiFiClient::setLocalPortStart(micros());

    if (WiFi.status() != WL_CONNECTED) {
        delay(10);
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid, password);
        WiFi.config(node_ip, node_gateway, node_subnet, node_gateway);

        int attempt = 0;
        while (WiFi.status() != WL_CONNECTED) {
            delay(100);
            Serial.print(".");
            attempt++;
            if (attempt == 150) {
                deepSleep();
            }
        }
    }
    client.set_callback(mqttCallback);
    if (client.connect(NODE_NAME)) {
        Serial.println("Connected to MQTT server");
    } else {
        Serial.println("Could not connect to MQTT server!");
    }
}

void enableSensors() {
    pinMode(PIN_GNDSW_ENABLE, OUTPUT);
    digitalWrite(PIN_GNDSW_ENABLE, HIGH);
}

void initSensors() {
    if (sensor_si1145) {
        initSensorSi1145();
    }
    if (sensor_bmp085) {
        initSensorBmp085();
    }
    if (sensor_dht22) {
        initSensorDht22();
    }
    if (sensor_chirp) {
        initSensorChirp();
    }
}

void initSensorSi1145() {
    if (sensor.begin()) {
        Serial.println("Si1145 OK!");
        sensor_ok = true;
    } else {
        Serial.println("Si1145 not OK - could not begin()");
    }
}

void initSensorBmp085() {
    if (bmp.begin()) {
        bmp_ok = true;
        Serial.println("BMP085 OK!");
    } else {
        /* There was a problem detecting the BMP085 ... check your connections */
        Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    }
}

void initSensorDht22() {
    // No return value for DHT::begin()
    dht.begin();
}

void initSensorChirp() {
    chirp.begin();
    // Apparently needed!
    delay(1000);
}

void readSensorSi1145() {
    if (sensor_ok) {
        uv = sensor.readUV() / 100;
        visible = sensor.readVisible();
        ir = sensor.readIR();
        Serial.println("Finished reading from Si1145. Publishing!");
    }
}

void readSensorBmp085() {
    if (bmp_ok) {
        Serial.println("BMP085 OK, reading!");
        /* Get a new sensor event */
        sensors_event_t event;
        bmp.getEvent(&event);

        /* Display the results (barometric pressure is measure in hPa) */
        if (event.pressure) {
            pressure = event.pressure;
        }

        bmp.getTemperature(&temperature);
    }

}

void readSensorDht22() {
    /* Adafruit recommends a dely of 2 seconds to get stable readings. We try to do without first. */
    dht_humidity = dht.readHumidity();
    // Read temperature as Celsius (the default)
    dht_temperature = dht.readTemperature();
    if (isnan(dht_humidity)) {
        Serial.println("Failed to read humidity from DHT22 sensor!");
        return;
    }
    if (isnan(dht_temperature)) {
        Serial.println("Failed to read temperature from DHT22 sensor!");
        return;
    }

    // Compute heat index in Celsius (isFahrenheit = false)
    dht_heat_index = dht.computeHeatIndex(dht_temperature, dht_humidity, false);
}

void readSensorChirp() {
    // there is a isBusy() method, but it's not available
    // on all firmware versions
    chirp_moisture = chirp.getCapacitance();
}

void readSensors() {
    if (sensor_si1145) {
        readSensorSi1145();
    }
    if (sensor_bmp085) {
        readSensorBmp085();
    }
    if (sensor_dht22) {
        readSensorDht22();
    }
    if (sensor_chirp) {
        readSensorChirp();
    }
}


void publishSensors() {
    if (sensor_si1145) {
        publishSensorSi1145();
    }
    if (sensor_bmp085) {
        publishSensorBmp085();
    }
    if (sensor_bat) {
        publishSensorBat();
    }
    if (sensor_dht22) {
        publishSensorDht22();
    }
    if (sensor_chirp) {
        publishSensorChirp();
    }
}

void publishSensorSi1145() {
    if (sensor_ok) {
        publish("uv", uv);
        publish("visible", visible);
        publish("ir", ir);
        Serial.println("Finished publishing from Si1145");
    }
}

void publishSensorBmp085() {
    if (bmp_ok) {
        Serial.println("Finished reading from BMP085. Publishing!");
        publish("temp", temperature);
        publish("pressure", pressure);
        Serial.println("Finished publishing from BMP085");
    }
}

void publishSensorBat() {
#ifdef SENSOR_BAT_INTERNAL
    publish("bat", ESP.getVcc());
#else
    // Read once to get noise out of the way
    analogRead(A0);
    float voltage = 0.0;
    int iterations = 3;
    for (int i = 0; i < iterations; i++) {
        voltage += analogRead(A0);
        delay(10);
    }
    voltage = voltage / iterations;
    voltage = voltage * SENSOR_BAT_EXTERNAL_CONSTANT;
    publish("bat", voltage);
#endif /* SENSOR_BAT_INTERNAL */
}

void publishSensorDht22()  {
    if (isnan(dht_humidity)) {
        return;
    }
    if (isnan(dht_temperature)) {
        return;
    }

    publish("dht-temperature", dht_temperature);
    publish("dht-humidity", dht_humidity);
    publish("dht-heat_index", dht_heat_index);
    Serial.println("Finished publishing to DHT22");
}

void publishSensorChirp() {
    publish("chirp-soil_capacitance", chirp_moisture);
    Serial.println("Finished publishing chirp!");
}

void publishCycleDuration() {
    int cur = millis();
    publish("cycle-duration", cur - start_up); 
}

void disableSensors() {
    digitalWrite(PIN_GNDSW_ENABLE, LOW);
}

void deepSleep() {
    Serial.println("going to sleep!");
    /* 
     * WAKE_RF_DEFAULT works.
     * WAKE_RF_DISABLED will keep the wifi disabled after the reboot!
     * 
     * WAKE_RF_DEFAULT will calibrate the RF according to the contents of
     * esp_init_data_default.bin which is flashed at 0x1fc00 at our 2M flash.
     * The value at byte 108 will dictate the frequency - after how many deep sleep
     * cycles RF calibration happens.
     * To change:
     * esptool.py read_flash 0x1fc000 1024 esp_init_data_default.bin
     * hexer esp_init_data_default.bin
     * esptool.py write_flash 0x1fc000 esp_init_data_default.bin
     */
    ESP.deepSleep(UPDATE_INTERVAL * 1000 * 1000, WAKE_RF_DEFAULT);
    /*
     * The internet says that the delay call is needed for ESP.deepSleep to work
     * properly
     */
    delay(100);
}


void setup(void) {
    Serial.begin(SERIAL_BAUD);
    // Wait at most 15s before going back to sleep
    sleepTicker.once_ms(15 * 1000, &deepSleep);
    // We enable sensors first, then wait for WiFi to connect. If WiFi connects faster
    // than we expect the sensors to be ready, we wait some additional time before initializing
    // and reading.
    // This means we will be online longer, but we use much less power during sleep if we turn off the
    // the sensors.
    // For the DHT-22, we stay online around 275ms longer.
    enableSensors();
    int sensor_enabled = millis();
    connectWifi();
    int wifi_connected = millis();
    const int SENSOR_STARTUP_TIME = 1500;
    if (wifi_connected - sensor_enabled < SENSOR_STARTUP_TIME) {
        delay(SENSOR_STARTUP_TIME - (wifi_connected - sensor_enabled));
    }
    initSensors();
    readSensors();
    //delay(5000); // power usage?
    int sensor_init_read = millis();
    publishSensors();
    int sensors_published = millis();
    publish("wifi-connect", wifi_connected - start_up);
    publish("sensor-init-read",  sensor_init_read - start_up);
    publish("sensor-publish", sensors_published - start_up);
    Serial.print("Wifi connect: ");
    Serial.println(wifi_connected - start_up);
    Serial.print("sensor-init-read: ");
    Serial.println(sensor_init_read - start_up);
    Serial.print("sensor-publish:");
    Serial.println(sensors_published - start_up);
    publishCycleDuration();
    deepSleep();
}


void loop(void) {
}

void publish(String measurement_name, float value) {
    String topic = String("/") + NODE_NAME + String("/") + measurement_name;
    client.publish(topic, String(value));
}
