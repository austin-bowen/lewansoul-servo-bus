# lewansoul-servo-bus

This is a Python 3 library implementing the [LewanSoul Servo Bus Communication Protocol](https://images-na.ssl-images-amazon.com/images/I/71WyZDfQwkL.pdf),
which is used by the following products:

- [Hiwonder bus servos](https://www.hiwonder.hk/collections/bus-servo)
- [Hiwonder xArm 1S robotic arm](https://www.hiwonder.hk/products/xarm-hiwonder-intelligent-bus-servo-robotic-arm-for-programming)
- [Hiwonder xArm 2.0 robotic arm](https://www.hiwonder.hk/products/xarm2-0-hiwonder-new-intelligent-robotic-arm-support-scratch-python-assemble-programmable-robotic-kit)

This library supports all the commands necessary to control the servo positions, as well as commands to set parameters
and read servo states such as position, temperature, and supply voltage.


### Quick Example

```python3
from lewansoul_servo_bus import ServoBus

servo_bus = ServoBus('/dev/ttyUSB0')

# Move servo with ID 1 to 90 degrees in 1.0 seconds
servo_bus.move_time_write(1, 90, 1.0)

# Move servo with ID 2 to 180 degrees in 2.0 seconds
servo_2 = servo_bus.get_servo(2)
servo_2.move_time_write(180, 2.0)
```


## Hardware

To talk to the servos, you will need a way to interface the computer with the servo bus. Most serial interfaces are
full-duplex, but the servo bus is half-duplex. The following circuit provides a simple way to convert full-duplex
signals to half-duplex:

![USB to LewanSoul Servo Bus](images/usb-to-lewansoul-servo-bus.svg)

### Notes
- If you want to isolate the computer power system from the servo power system, check out
[this](images/usb-to-lewansoul-servo-bus-isolated.svg) schematic.
- Notice that this configuration will cause all data sent by the computer to be "echoed" back to the computer.
`ServoBus` discards echoed data by default, but that behavior can be disabled with `ServoBus(discard_echo=False)`.
- This library is for _direct_ communication with the bus servos; it is _not_ intended for communication with the
servos through e.g. the Hiwonder Serial Bus Servo Controller. This allows for faster communication (115200 baud vs
9600), access to _all_ the protocol functions, and less hardware.


## Software

1. You may want to create a Python 3 virtualenv at the root of this repository.
1. Run:
   1. `pip install -r requirements.txt`
   1. `cd src/python`
   1. `python3 lewansoul_servo_bus.py`
1. Of course, you can also import `lewansoul_servo_bus.py` in your own project and use the `ServoBus` class within to
   control your servos. Just make sure the `src/python` directory is in your `PYTHONPATH`.

### Example Usage

```python3
from lewansoul_servo_bus import ServoBus

def main():
    with ServoBus('/dev/ttyUSB0') as servo_bus:
        pan_servo = servo_bus.get_servo(1)
        tilt_servo = servo_bus.get_servo(2)

        # Do things with pan/tilt_servo...
```