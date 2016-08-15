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

#ifdef SENSOR_BAT_INTERNAL
ADC_MODE(ADC_VCC);
#else
ADC_MODE(ADC_TOUT);
#endif


int start_up = millis();
int wifi_connected = 0;
int sensor_init_published = 0;

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

#include "DHT.h"
DHT dht(PIN_DHT22, DHT22);


WiFiClient wifiClient;
PubSubClient client(wifiClient, MQTT_HOST);

void publish(String name, float val);

void connectWifi()  {
    Serial.begin(SERIAL_BAUD);
    Serial.println("Initializing!");
    Serial.println("Connecting to WiFi");

    IPAddress node_ip(192, 168, 1, 61);
    IPAddress node_gateway(192, 168, 1, 1);
    IPAddress node_subnet(255, 255, 255, 0);
    // We assume that gateway is also the DNS
    WiFi.config(node_ip, node_gateway, node_subnet, node_gateway);
    // https://github.com/z2amiller/sensorboard/blob/master/PowerSaving.md
    if (WiFi.SSID() != String(ssid)) {
        WiFi.begin(ssid, password);
        WiFi.mode(WIFI_STA);
        WiFi.persistent(true);
        WiFi.setAutoConnect(true);
    }

    Serial.println("");
    Serial.println("Test blah");

    while (WiFi.status() != WL_CONNECTED) {
        delay(100);
        Serial.print(".");
    }
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    if (client.connect(NODE_NAME)) {
        Serial.println("Connected to MQTT server");
    } else {
        Serial.println("Could not connect to MQTT server!");
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

void publishSensorSi1145() {
    if (sensor_ok) {
        Serial.println("Si1145 OK, reading!");
        uv = sensor.readUV() / 100;
        visible = sensor.readVisible();
        ir = sensor.readIR();
        Serial.println("Finished reading from Si1145. Publishing!");
        publish("uv", uv);
        publish("visible", visible);
        publish("ir", ir);
        Serial.println("Finished publishing from Si1145");
    }
}

void publishSensorBmp085() {
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
        Serial.println("Finished reading from BMP085. Publishing!");
        publish("temp", temperature);
        publish("/weather/pressure", pressure);
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
    /* Adafruit recommends a dely of 2 seconds to get stable readings. We try to do without first. */
    float humidity = dht.readHumidity();
    // Read temperature as Celsius (the default)
    float temperature = dht.readTemperature();
    if (isnan(humidity)) {
        Serial.println("Failed to read humidity from DHT22 sensor!");
        return;
    }
    if (isnan(temperature)) {
        Serial.println("Failed to read temperature from DHT22 sensor!");
        return;
    }

    // Compute heat index in Celsius (isFahrenheit = false)
    float heat_index = dht.computeHeatIndex(temperature, humidity, false);
    Serial.println("Heat index: " + String(heat_index));
    Serial.println("Finished reading from DHT22. Publishing!");
    publish("dht-temperature", temperature);
    publish("dht-humidity", humidity);
    publish("dht-heat_index", heat_index);
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
    connectWifi();

    wifi_connected = millis();
    // Debug
    publish("wifi-connect", wifi_connected - start_up);

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


void loop(void) {
    Serial.println("Entering loop");
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

    // Debug
    sensor_init_published = millis();
    publish("sensor-init-publish", sensor_init_published - wifi_connected);
    publishCycleDuration();
    deepSleep();
}

void publish(String measurement_name, float value) {
    String topic = String("/") + NODE_NAME + String("/") + measurement_name;
    client.publish(topic, String(value));
}
