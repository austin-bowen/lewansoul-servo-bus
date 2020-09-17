# hiwonder-xarm-python3
Control a [Hiwonder xArm robot arm](https://www.hiwonder.hk/products/xarm-hiwonder-6dof-bus-servo-robotic-arm-based-on-scratch-arduino-programmable-robotic-arm)
using Python 3.


## Hardware
In addition to a Hiwonder xArm, you will need an Arduino-compatible microcontroller with a USB serial interface,
and at least one UART interface.  The microcontroller is used to convert the serial commands from the computer into
half-duplex UART commands sent to the xArm's servo bus.  I used a [Seeeduino XIAO](https://www.seeedstudio.com/Seeeduino-XIAO-Arduino-Microcontroller-SAMD21-Cortex-M0+-p-4426.html)
when writing this library.

The microcontroller-to-xArm wiring is very simple:  Just connect a suitable resistor (e.g. 4.7k) between the
UART TX and RX pins, and connect the UART RX pin to the xArm servo bus' signal wire.  The whole hardware diagram looks
like this:

```
              +-------------------+
              |           UART TX>|----+
              |  Arduino          |    |
              |  Compatible       |    # 4.7k
              |  Microcontroller  |    # Resistor
              |                   |    |
|Computer>----|<USB       UART RX>|----+----<Servo Bus Signal Wire|
              |               Gnd>|---------<Servo Bus Ground Wire|
              +-------------------+
```

Where `UART TX` is pin 6 and `UART RX` is pin 7 on the Seeeduino XIAO.


## Software
1. If you are not using a Seeeduino XIAO, you may need to edit the `UART_TX_PIN` setting in the xarm.ino file
   to match your microcontroller.
1. Use the Arduino IDE to load the xarm.ino file into your microcontroller.
1. You may wish to create a Python 3 virtualenv at the root of this repository.
1. Run:
   1. `pip install -r requirements.txt`
   1. `cd src/python`
   1. `python3 xarm.py`
1. Of course you can also import xarm.py in your own project and use the Xarm class within to control your arm.
