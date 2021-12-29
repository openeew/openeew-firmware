# Firmware for OpenEEW seismic sensor
This firmware allows an OpenEEW ESP32 sensor to connect to an MQTT broker and send a JSON containing accelerations from the x,y and z axis every second.

## Setup Sensor and MQTT details
Add your sensor details in `src/secrets.h`:
- `user` This is a unique name of the sensor network owner
- `network` This is a 2 digit code which identifies the sensor network
- `station` This is a 5 digit code which identifies the sensor you are about to flash. Every sensor you flash must have a unique station code.

Add your MQTT broker details also in `src/secrets.h`:
- `MQTT_HOST` This is the network address of your MQTT host such as 192.168.0.10 or mqtt.yourserver.com
- `MQTT_PORT` This is the port of your MQTT host, typically 1883.

## Flash a new device
Connnect your sensor to your PC's USB, and add the USB port to `upload_port` in `platformio.ini`.

From the PlatformIO menu, choose `Upload` and wait for your device to flash. When done, you will be able to see the output from a serial monitor, such as the one available in PlatformIO.

## Connect sensor to internet
If you wish to use Ethernet simply plug in the cable before you turn on the sensor. If you wish to use Wifi, then wait until the LEDs flash yellow and open your SmartConfig app to send over your wifi credentials (must be 2.4Ghz wifi only). Apps you can use include (not all have been tested):
- [EspTouch iOS](https://apps.apple.com/us/app/espressif-esptouch/id1071176700)
- [EspTouch Android](https://play.google.com/store/apps/details?id=com.khoazero123.iot_esptouch_demo&hl=en&gl=US)

The sensor data will now be available from your MQTT broker at the topic `<user>/<network>/<deviceID>`, where `deviceID` is automatically created from the device's unique Wifi MAC address.

## Acknowledgements
This firmware is based off the excellent [ADXL355 library](https://github.com/markrad/ADXL355-CPP-SDK) by [MarkRad](https://github.com/markrad).
