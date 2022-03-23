# LIS3DH

Toit driver for the LIS3DH accelerometer.

A full description of the LIS3DH sensor can be found at the [LIS3DH product page.][prodpage]

## Usage

A simple usage example.

```
import gpio
import i2c
import lis3dh

main:
  bus := i2c.Bus
    --sda=gpio.Pin 21
    --scl=gpio.Pin 22

  device := bus.device lis3dh.I2C_ADDRESS
  sensor := lis3dh.Lis3dh device

  sensor.enable
  print sensor.read_acceleration
```

See the `examples` folder for more examples.

## References

Documentation for the LIS3DH: https://www.st.com/en/mems-and-sensors/lis3dh.html#documentation
The Datasheet is [DS6839](https://www.st.com/resource/en/datasheet/lis3dh.pdf).

## Features and bugs

Please file feature requests and bugs at the [issue tracker][tracker].

[tracker]: https://github.com/nilwes/LIS3DH/issues
[prodpage]: https://www.st.com/en/mems-and-sensors/lis3dh.html#documentation
[datasheet]: https://www.st.com/resource/en/datasheet/lis3dh.pdf
