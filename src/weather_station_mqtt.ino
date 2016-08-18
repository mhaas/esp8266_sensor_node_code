#include "wlan_secret.h"

/*
   BEGIN config constants
 */

//  74880 shows the boot messages
#define SERIAL_BAUD 74880
/*
   For the ADC/Battery voltage measurement: on the NodeMCU 1.0, Pin A0 already has a voltage divider so that it expects 0-3.3V.
   For a ~6V battery pack, you have go bring your own voltage divider and use PIN2 as the input (labeled as RSVD). See Page 5 "IO CONN" in NODEMCU_DEVKIT_V1.0.PDF.
   With my DMM, "deep sleep" voltage is 4.96V and about 4.85 once the ESP wakes. This results in a value of 743 from the ADC.
 */

#define SENSOR_BAT_EXTERNAL_CONSTANT 4.85 / 743.0;

/*
   END config constants
 */

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Ticker.h>

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

WiFiClient wifiClient;
PubSubClient client(wifiClient, MQTT_HOST);

void publish(String name, float val);

uint8_t bssid[] = {0xF4, 0xF2, 0x6D, 0x9C, 0x0D, 0xB9};

void connectWifi()  {
    IPAddress node_ip(192, 168, 1, 61);
    IPAddress node_gateway(192, 168, 1, 1);
    IPAddress node_subnet(255, 255, 255, 0);


  WiFiClient::setLocalPortStart(micros());

    if (WiFi.status() != WL_CONNECTED) {
        delay(10);
        WiFi.mode(WIFI_STA);
        // ****************
        WiFi.begin(ssid, password);
        WiFi.config(node_ip, node_gateway, node_subnet, node_gateway);
        // ****************

        int Attempt = 0;
        while (WiFi.status() != WL_CONNECTED) {
            delay(50);
            delay(50);
            Serial.print(".");
            Attempt++;
            if (Attempt == 150) {
                deepSleep();
            }
        }
    }
    if (client.connect(NODE_NAME)) {
        Serial.println("Connected to MQTT server");
    } else {
        Serial.println("Could not connect to MQTT server!");
    }
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
    float voltage = analogRead(A0) * SENSOR_BAT_EXTERNAL_CONSTANT;
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

void publishCycleDuration() {
    int cur = millis();
    publish("cycle-duration", cur - start_up); 
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
    // Channel is not stored (?), so we set it all the time
    // First things first: we set up the sensors first, the wifi should
    // auto-connect in the meantime - except for the very first boot,
    // where wifi will have to be set up in connectWifi().
    // That should shave off precious milliseconds - FWIW, reading the DHT22
    // takes about 275ms
    initSensors();
    readSensors();
    int sensor_init_read = millis();
    connectWifi();
    int wifi_connected = millis();
    publishSensors();
    int sensors_published = millis();
    publish("wifi-connect", wifi_connected - start_up);
    publish("sensor-init-read",  sensor_init_read - start_up);
    publish("sensor-publish", sensors_published - start_up);
    publishCycleDuration();
    deepSleep();
}


void loop(void) {
}

void publish(String measurement_name, float value) {
    String topic = String("/") + NODE_NAME + String("/") + measurement_name;
    client.publish(topic, String(value));
}
