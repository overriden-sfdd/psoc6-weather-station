# PSoC 6 Wi-Fi BT pioneer kit MCU: Weather Station and Web Example
The project contains a sample of a working weather station. As well as an example of a working HTTPS server and realtime MQTT communication.

## Weather Station
The main component is [BMP280](https://www.adafruit.com/product/2651) which provides barometric pressure and temperature.

The information from the sensor is obtained via I2C protocol. Then, it is shown on CY8CKIT-028-TFT display.

## Web Example
The HTTPS page stores the status of the board LED. The page runs at https://mysecurehttpserver:50007. Every time the LED changes its status, the HTTPS page gets updated.

The LED status change is requested via MQTT message. The MQTT is configured to have a broker IPv4 address at 10.5.1.205 at port 1883. The pub/sub topic is the same - ledstatus.

### Note

The libraries have been updated multiple times, and I have no time to maintain the project, so, most likely, the code will not compile. However, this project has been tested, and all the components are perfectly valid and working examples of how to setup a minimalistic weather & web project.

