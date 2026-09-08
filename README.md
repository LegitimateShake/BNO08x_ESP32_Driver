# BNO086 Driver for ESP32 - ESP IDF v6.x.x

This repo includes driver code for the BNO086. 
It should also work for the BNO085, as they are mostly the same chip.
The motivation behind this was to write a driver that supports multiple BNO08x at the same time, as this is sadly not the case with the manufacturers driver code.
The driver is written in **C++** currently only supports communication over **I2C**.

## Nessecary pin connections:
- **SCL** (eigther internal or external pullup resistor required)
- **SDA** (eigther internal or external pullup resistor required)
- **H_INT** (nessecary for BNO08x interrupts)
- **RST**   (nessecary for resetting the BNO08x)

## Includes

> `#include <bno086.h>`

## Environment-Sensor support

The driver currently supports pressure, humidity and temperature environment sensors, as they were the only ones I had access to.

## Implementations

### `begin()`
Before any communication with the sensor can be established one must first initialize the BNO08x object.
To do so, `begin()` must be called. After that, any implementation can be used.

---

### `config_...()`
Implementations that start with `config_...()` can be used to configure the behaviour of any sensor of the BNO08x.
To create a new sensor config the following method can be called:
> `create_sensor_config()`

To later enable the sensor, based on the created configuration, call:
> `enable_sensor()`

To disable the sensor again, call the following method with the same configuration struct:
> `disable_sensor()`

---

### `read_...()`
Implementations that start with `read_...()` return data from the sensor. In most cases, a struct must be supplied, that the sensor data can be written to. These functions will always return the newest sensor data that was not yet read by the user. Additionally, the user can set a timeout. The timeout indicates the amount of time the function will block to wait for new sensor data. The default is `100ms`. A timeout of `0ms` will result in non blocking behaviour, as the function will only check for new data once. When multiple sensors are read at the same time it is recommended to only set a timeout for the fastest sensor and set the timeout of all other sensors to zero.

---

### `frs_...()`

Implementations that start with frs_...() write data to or read data from the flash record system of the BNO08x.
Any configurations that are taken here will persist through power cycles.

## Status codes

Most methods will return a status code after completion. This code indicates if the function call was successful or not. Success is indicated by the code `bno_err_t::OK`. To check the return code of any function call the following non member method can be used:
> `bno_check_sc()`

By default, this function will block indefinitely if a status code other than `bno_err_t::OK` is returned.
Additionally, an error message will be logged over the ESP's serial interface.

## Data Types

This driver utilizes several custom data types. All types, as well as all status codes are defined in `bno_types.h`.

