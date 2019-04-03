# Wz

A crowd-source mobile weather station based on esp32 and me3616

## Requirements

### Hardware chips

- MCU: ESP32-DEVKITC-V3
- NBiot: ME3616

### Sensors

- BME280: temperature & humidity & pressure sensor
- MAX44009: illuminance sensor
- GYML8511: ultraviolet sensor
- GP2Y1010AU: pm2.5 dust sensor

### esp-idf components

- [i2cdev](https://github.com/UncleRus/esp-idf-lib)
  - bmp280
- max44009 
- me3616
- bsp
