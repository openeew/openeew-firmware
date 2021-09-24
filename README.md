# OpenEEW Sensor Firmware
The OpenEEW sensor features a high performance MEMS accelerometer and Ethernet or WiFi connectivity. It includes also a loud buzzer and 3 bright Neopixel LEDS for alarm functions. By including alarm functions, the owners of the locations where they are installed are more likely to value and look after the device.

## Firmware

This code allows an ESP32 device to send 3 axis accelerometer readings to a remote MQTT endpoint from its accelerometer to an MQTT endpoint.

We have provided 2 versions for convenience depending on your infrastructure:
- [AWS](https://github.com/openeew/openeew-firmware/tree/main/AWS_IoT)
- [Watson IBM](https://github.com/openeew/openeew-firmware/tree/main/WatsonIoT)
- [Local MQTT Broker](https://github.com/openeew/openeew-firmware/tree/main/localnet)


Learn more about using [MQTT to communicate with the firmware](FIRMWARE.md)

### Contributors

<a href="https://github.com/openeew/openeew-firmware/graphs/contributors">
  <img src="https://contributors-img.web.app/image?repo=openeew/openeew-firmware" />
</a>
___

Enjoy! Give us [feedback](https://github.com/openeew/openeew-firmware/issues) if you have suggestions on how to improve this information.

## Contributing and Developer information

The community welcomes your involvement and contributions to this project. Please read the OpenEEW [contributing](https://github.com/openeew/openeew/blob/master/CONTRIBUTING.md) document for details on our code of conduct, and the process for submitting pull requests to the community.

## License

The OpenEEW sensor is licensed under the Apache Software License, Version 2. Separate third party code objects invoked within this code pattern are licensed by their respective providers pursuant to their own separate licenses. Contributions are subject to the [Developer Certificate of Origin, Version 1.1 (DCO)](https://developercertificate.org/) and the [Apache Software License, Version 2](http://www.apache.org/licenses/LICENSE-2.0.txt).
