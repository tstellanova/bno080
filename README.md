# bno080

A rust embedded-hal driver for the Hillcrest Laboratories / Bosch Sensortec 
BNO080 9DOF AHRS

This sensor device combines a triple axis accelerometer, 
gyroscope, and magnetometer, and contains an ARM Cortex M0+ 
for fusing all the sensor data into a meaningful rotation vector. 

For reference, see the original 
[Hillcrest BNO080 driver](https://github.com/tstellanova/bno080-driver) implemented in C. 

## Status

- [x] Basic i2c interface support
- [x] Some tests for decoding and encoding methods
- [x] Basic SHTP protocol support
- [ ] SPI support (only stubs at this point)
- [ ] CI
- [ ] Usage examples
- [ ] Support for calibration 
- [ ] Support for tare
- [ ] Support for external barometer (eg bmp280)
- [ ] Support for external light sensor




