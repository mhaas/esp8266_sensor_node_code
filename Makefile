SKETCH = weather_station_mqtt.ino
ESP_ROOT = $(CURDIR)/ext/esp8266
ESP_LIBS_EXTRA = $(CURDIR)/ext/libs/

LIBS = $(ESP_LIBS)/ESP8266WiFi \
        $(ESP_LIBS)/Wire \
        $(ESP_LIBS_EXTRA)/imroy_pubsubclient \
        $(ESP_LIBS_EXTRA)/Adafruit_Sensor \
        $(ESP_LIBS_EXTRA)/Adafruit_BMP085_Unified \
        $(ESP_LIBS_EXTRA)/Adafruit_SI1145_Library \
        $(ESP_LIBS_EXTRA)/DHT-sensor-library
        
BUILD_ROOT = dist/

include ext/makeEspArduino/makeEspArduino.mk

