# Wz

A crowd-source mobile weather station based on esp32 and me3616.

we produced 2 versions:

- **Wz-Pro** (NBiot + LWM2M version): powerfull
- **Wz-mini** (WiFi + MQTT version): tiny and convenient

| components | NBiot + LWM2M | WiFi + MQTT    |
| ---------- | ------------- | -------------- |
| GPS        | me3616-G1A    | no             |
| oled       | no            | SSD1306 128x32 |
| battery    | 7.4V 1800mAh  | 3.7v  100mAh   |
| GP2Y1014AU | yes           | no             |

## Requirements

### Hardware chips

- MCU: ESP32-WROOM-32D
- NBiot: ME3616 G1A

### Sensors

- BME280: temperature & humidity & pressure sensor
- MAX44009: illuminance sensor
- GYML8511: ultraviolet sensor
- GP2Y1014AU: pm2.5 dust sensor

### esp-idf components

- [tarablessd1306](https://github.com/TaraHoleInIt/tarablessd1306)
- [i2cdev](https://github.com/UncleRus/esp-idf-lib)
  - bmp280
- bsp (tcp, used in wifi branch)
- max44009 (by spartazhc)
- me3616 (by spartazhc)



## Settings

### Wz-Pro GPIO define (nbiot version)

| Component | Usage | GPIO pin   | Mode     |
| --------- | -------- | ------ | ---- |
| NB-iot    | RST      | 5 |   output|
|           | PWR | 17 | output |
|           | RXD | 4 | UART |
|           | TXD | 16 | UART |
| BME280 | SDA | 18 | I2C |
|           | SCL | 19 | I2C |
| MAX44009 | SDA | 18 | I2C |
|           | SCL | 19 | I2C |
|           | INTR | 2 | input, INTR_NEGEDGE |
| ML8511 | EN | 15 | output |
|           | ADC | 35 | adc |
|GP2Y1014AU|LED|25|output|
||ADC|34|adc|

### Wz-mini GPIO define (WIFI verison)
| Component | Usage | GPIO pin   | Mode     |
| --------- | -------- | ------ | ---- |
| BME280 | SDA | 18 | I2C |
|           | SCL | 19 | I2C |
| MAX44009 | SDA | 18 | I2C |
|           | SCL | 19 | I2C |
| ML8511 | EN | 15 | output |
|           | ADC | 35 | adc |

