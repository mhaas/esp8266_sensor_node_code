#include <DHT.h>

/**
   Flashing: for whatever reason, the Adafruit Huzzah and the Olimex USB-Serial-Cable-F do not play nicely together.

   In my first attempt, I interrupted a flash and only got a dimly lit red led ever since. I could flash sketches using Arduino (with the non-python
   esptool bundled in Arduino) and the flashing seemed to go OK, but the Huzzah remained catatonic. The python esptool.py would apparently succeed erasing the flash memory,
   but then abort flashing immediately.

   Only by replacing USB-Serial-Cable-F with an msp430 launchpad AND using esptool.py I could flash a sketch. Just using the Arduino-bundled esptool
   with the msp430 launchpad did not work.

   There are some reports on the internet that some usb<->serial adapters can be iffy. The "LED stays dimly lit" probably means that
   the flash is erased and the bootloader cannot load any code (?).


   TODO:
   - deep sleep: https://blog.adafruit.com/2016/02/23/iot-weather-station-with-adafruit-huzzah-esp8266-esp-12e-and-adafruit-io-iotuesday-adafruitio/
                 https://forums.adafruit.com/viewtopic.php?f=19&t=85703
   - mqtt: https://github.com/knolleary/pubsubclient/

   There seems to be a weird beavhior where deepSleep  w/o RF seems to go to bootloader mode - the ESP8266 seems to remember the last boot mode.

   Always physically power off and power on the ESP8266 after flashing.. or at least push the reset button. It seems the flashing process via esptool can leave the
   esp8266 in a weird state:  https://github.com/esp8266/Arduino/issues/1017

*/


#include "wlan_secret.h"
/*
   BEGIN config constants
*/

#define NODE_NAME "weather"
#define MQTT_HOST "mqtt"
#define UPDATE_INTERVAL 30
#define SERIAL_BAUD 9600
#define SENSOR_SI1145 1
#define SENSOR_BMP085 1
#define SENSOR_BAT 1
#define SENSOR_DHT22
#define SENSOR_BAT_INTERNAL
#define SENSOR_DHT22
#define PIN_LED 13
#define PIN_DHT22 6 /* Don't know */
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

/*
   TODO: the file-level initialization does not work well with enabling/disabling
   of features via (always visible) functions - we should move the declarations into the
   functions themselves. That way, they are always compiled (=> run through static analysis)
   even if they are later stripped - assuming that stripping actually happens.

   Alternatively, we could go back to run-time selection of sensors instead of compile-time selection
*/

#ifdef SENSOR_SI1145

#include <Wire.h>
#include "Adafruit_SI1145.h"
#include <Adafruit_Sensor.h>
Adafruit_SI1145 sensor = Adafruit_SI1145();
boolean sensor_ok = false;
uint16_t uv = 0;
uint16_t ir = 0;
uint16_t visible = 0;

#endif

#ifdef SENSOR_BMP085

#include <Adafruit_BMP085_U.h>
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
boolean bmp_ok = false;
float pressure = 0;
float temperature = 0;

#endif

#ifdef SENSOR_DHT22
#include "DHT.h"
DHT dht(PIN_DHT22, DHT22);
#endif


WiFiClient wifiClient;
PubSubClient client(wifiClient, MQTT_HOST);


void setup(void) {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, 1);
  connectWifi();

#ifdef SENSOR_SI1145
  initSensorSi1145();
#endif SENSOR_SI1145

#ifdef SENSOR_BMP085
  initSensorBmp085();
#endif


}

void connectWifi()  {
  Serial.begin(SERIAL_BAUD);
  Serial.println("Initializing!");
  Serial.println("Connecting to WiFi");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

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
    client.publish("/weather/uv", String(uv));
    client.publish("/weather/visible", String(visible));
    client.publish("/weather/ir", String(ir));
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
    client.publish("/weather/temp", String(temperature));
    client.publish("/weather/pressure", String(pressure));
    Serial.println("Finished publishing from BMP085");
  }
}

void publishSensorBat() {
#ifdef SENSOR_BAT_INTERNAL
  client.publish("/weather/bat", String(ESP.getVcc()));
#else
  float voltage = analogRead(A0) * SENSOR_BAT_EXTERNAL_CONSTANT;
  client.publish("/weather/bat", String(voltage));
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
  Serial.println("Finished reading from DHT22. Publishing!");
  publish("dht-humidity", pressure);
  publish("dht-heat_index", heat_index);
  Serial.println("Finished publishing to DHT22");


}

void deepSleep() {
  Serial.println("going to sleep!");
  // TODO: WAKE_RF_DEFAULT works, but WAKE_RF_DISABLED will keep the wifi disabled
  // after the reboot!
  digitalWrite(PIN_LED, 0);
  ESP.deepSleep(UPDATE_INTERVAL * 1000 * 1000, WAKE_RF_DEFAULT);
  /* The internet says that the delay call is needed for ESP.deepSleep to work
      properly
  */
  delay(100);
}

void publish(String measurement_name, float value) {
  String topic = String("/") + NODE_NAME + String("/") + measurement_name;
  client.publish(topic, String(value));

}

void loop(void) {
  Serial.println("Entering loop");
#ifdef SENSOR_SI1145
  publishSensorSi1145();
#endif

#ifdef SENSOR_BMP085
  publishSensorBmp085();
#endif SENSOR_BMP085

#ifdef SENSOR_BAT
  publishSensorBat();
#endif /* SENSOR_BAT */

#ifdef SENSOR_DHT22
  publishSensorDht22();
#endif
  deepSleep();

}
