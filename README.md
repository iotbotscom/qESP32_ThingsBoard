qESP qESP32 IOT Board ThingsBoard Demo
------------------------------

![qESP32 WiFi and Bluetooth ESP32 IOT Board](https://cdn.shopify.com/s/files/1/0269/3100/3464/products/iotbotscom-qesp32-iot-arduino-wifi-esp32_1024x1024@2x.jpg)

[*qESP32 WiFi and Bluetooth ESP32 IOT Board*](https://www.iot-bots.com/collections/qesp-iot/products/qesp32-esp32-diy-iot-development-kit)

![qESP32 IOT Board ThingsBoard Demo](https://github.com/iotbotscom/qESP32_ThingsBoard/blob/main/pics/qESP-qESP32-esp32-wifi-thingsboard-cloud.png)

------------------------------

The newest qESP32 DIY WiFi Enabled IOT Board is coming to help hobbyists, makers and all DIYers creates low power cloud connected applications in a minute. Designed to be used with long lasting power Lithium CR123A battery, qESP32 WiFi IOT board, having everything onboard, allows you to build local sensors data monitoring, security and alarm systems and other IOT solutions. 

- Efficient Power Management: CR123A battery with DC/DC step-up converter;
- Ultimate sensors set: motion, temperature, humidity, pressure, light;
- Enhanced indication: PWM controller with five RGB LEDs & sound alert;
- Developer HW extensions: I2C&Power JST connector and Add-On Boards interface;
- Open source tools and environments: Arduino IDE & Esperssif SDK compatible.

Built-in Features:
------------------------------

 - "Small & Сute": it is just 1,72" (44 mm) round 4-layers board;
 - Wireless Included: Feature-rich ESP32 SOC with integrated Wi-Fi and Bluetooth connectivity;
 - Security Alert & Motion detection: LIS3DH accelerometer;
 - Local Condition monitoring: BME280 Temperature, Humidity, Pressure and TSL2591 Light sensors;
 - Funny visual indication: PWM controller with five RGB LEDs around and one status LED;
 - Audible alert capability: externally driven magnetic buzzer;
 - Simple and sufficient user controls: User and Reset buttons;
 - Developer Extensions: I2C&Power 4-pin JST connector (SparkFun Qwiic and Adafruit STEMMA compatible) and Add-On Boards (qESP Bits) interface.

ThingBoard Demo:
------------------------------
The purpose of this Demo is show how to use qESP32 IOT Board for collecting and publishing data from sensors embedded : Motion, Light, Temperature, Humidity and Pressure.
Thingsboard cloud is used.

Hardware Setup (qESP / qESP32):
 - qESP / qESP32 / qButton IOT board;
 - ESP programmer:
    - option1 : qProg with USB cable;
    - option2 : any ESP chip programmer with jump wires and USB cable;
 - CR123A Lithium Battery.

Hardware Setup (qButton):
 - qButton IOT board;
 - USB Cable.

Arduino Setup:
 - "ESP32 Dev Module" to be choosen as board;
 - Arduino Libs to be installed:
   - Adafruit_BME280_Library : https://github.com/adafruit/Adafruit_BME280_Library
   - Adafruit-PWM-Servo-Driver-Library : https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
   - Adafruit_TSL2591_Library : https://github.com/adafruit/Adafruit_TSL2591_Library
   - ThingsBoard related: https://thingsboard.io/docs/samples/esp32/gpio-control-pico-kit-dht22-sensor/

Project Options:
 - You need to set these defines in user_config.h file:
   - QESP_IOT_BOARD_TYPE : CPU Board type, "qESP32" Default
   - QESP_IOT_BOARD_SLEEP_MODE_TYPE : Project Sleep Mode Option, "Always On" Default

ThingsBoard Demo Source code:
- https://github.com/iotbotscom/qESP32_ThingsBoard

