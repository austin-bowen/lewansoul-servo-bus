"""
Library for controlling bus servos using the `LewanSoul Bus Servo Communication
Protocol <https://images-na.ssl-images-amazon.com/images/I/71WyZDfQwkL.pdf>`_.
"""

import struct
import time
from multiprocessing import RLock
from typing import Union, NamedTuple, Literal, Tuple, List, Optional

import serial

# General servo constants
MIN_ID = 0
MAX_ID = 253
BROADCAST_ID = 254
MIN_ANGLE_DEGREES = 0
MAX_ANGLE_DEGREES = 240

# Packet stuff
_PACKET_HEADER = b'\x55\x55'
_1_SIGNED_CHAR_STRUCT = struct.Struct('<b')
_1_SIGNED_SHORT_STRUCT = struct.Struct('<h')
_1_UNSIGNED_CHAR_1_UNSIGNED_SHORT_STRUCT = struct.Struct('<bxh')
_2_UNSIGNED_SHORTS_STRUCT = struct.Struct('<HH')

# Servo command numbers
_SERVO_MOVE_TIME_WRITE = 1
_SERVO_MOVE_TIME_READ = 2
_SERVO_MOVE_TIME_WAIT_WRITE = 7
_SERVO_MOVE_TIME_WAIT_READ = 8
_SERVO_MOVE_START = 11
_SERVO_MOVE_STOP = 12
_SERVO_ID_WRITE = 13
_SERVO_ID_READ = 14
_SERVO_ANGLE_OFFSET_ADJUST = 17
_SERVO_ANGLE_OFFSET_WRITE = 18
_SERVO_ANGLE_OFFSET_READ = 19
_SERVO_ANGLE_LIMIT_WRITE = 20
_SERVO_ANGLE_LIMIT_READ = 21
_SERVO_VIN_LIMIT_WRITE = 22
_SERVO_VIN_LIMIT_READ = 23
_SERVO_TEMP_MAX_LIMIT_WRITE = 24
_SERVO_TEMP_MAX_LIMIT_READ = 25
_SERVO_TEMP_READ = 26
_SERVO_VIN_READ = 27
_SERVO_POS_READ = 28
_SERVO_OR_MOTOR_MODE_WRITE = 29
_SERVO_OR_MOTOR_MODE_READ = 30
_SERVO_LOAD_OR_UNLOAD_WRITE = 31
_SERVO_LOAD_OR_UNLOAD_READ = 32
_SERVO_LED_CTRL_WRITE = 33
_SERVO_LED_CTRL_READ = 34
_SERVO_LED_ERROR_WRITE = 35
_SERVO_LED_ERROR_READ = 36

# Custom types
Real = Union[float, int]


class _ServoPacket(NamedTuple):
    servo_id: int
    command: int
    parameters: bytes


class Servo:
    """Represents a specific servo on a servo bus."""

    def __init__(self, id_: int, bus: 'ServoBus', name: str = None) -> None:
        """
        This is not meant to be instantiated directly. Use
        `ServoBus.get_servo(...)` instead.
        """

        self.id = id_
        self.bus = bus
        self.name = name

    def __str__(self) -> str:
        name = self.name or 'Servo'
        return f'{name} (ID {self.id})'

    def move_time_write(self, *args, **kwargs) -> None:
        self.bus.move_time_write(self.id, *args, **kwargs)

    def move_time_wait_write(self, *args, **kwargs) -> None:
        self.bus.move_time_wait_write(self.id, *args, **kwargs)

    def move_time_read(self) -> Tuple[float, float]:
        return self.bus.move_time_read(self.id)

    def move_time_wait_read(self) -> Tuple[float, float]:
        return self.bus.move_time_wait_read(self.id)

    def move_speed_write(self, *args, **kwargs) -> None:
        self.bus.move_speed_write(self.id, *args, **kwargs)

    def velocity_read(self, *args, **kwargs) -> float:
        return self.bus.velocity_read(self.id, *args, **kwargs)[0]

    def move_start(self) -> None:
        self.bus.move_start(self.id)

    def move_stop(self) -> None:
        self.bus.move_stop(self.id)

    def id_write(self, new_id: int) -> None:
        self.bus.id_write(self.id, new_id)
        self.id = new_id

    def angle_offset_adjust(self, *args, **kwargs) -> None:
        self.bus.angle_offset_adjust(self.id, *args, **kwargs)

    def angle_offset_write(self) -> None:
        self.bus.angle_offset_write(self.id)

    def angle_offset_read(self) -> float:
        return self.bus.angle_offset_read(self.id)

    def angle_limit_write(self, *args, **kwargs) -> None:
        self.bus.angle_limit_write(self.id, *args, **kwargs)

    def angle_limit_read(self) -> Tuple[float, float]:
        return self.bus.angle_limit_read(self.id)

    def vin_limit_write(self, *args, **kwargs) -> None:
        self.bus.vin_limit_write(self.id, *args, **kwargs)

    def vin_limit_read(self) -> Tuple[float, float]:
        return self.bus.vin_limit_read(self.id)

    def temp_max_limit_write(self, *args, **kwargs) -> None:
        return self.bus.temp_max_limit_write(self.id, *args, **kwargs)

    def temp_max_limit_read(self, *args, **kwargs) -> float:
        return self.bus.temp_max_limit_read(self.id, *args, **kwargs)

    def temp_read(self, *args, **kwargs) -> float:
        return self.bus.temp_read(self.id, *args, **kwargs)

    def vin_read(self) -> float:
        return self.bus.vin_read(self.id)

    def pos_read(self) -> float:
        return self.bus.pos_read(self.id)

    def mode_write(self, *args, **kwargs) -> None:
        return self.bus.mode_write(self.id, *args, **kwargs)

    def mode_read(self) -> Tuple[str, Optional[int]]:
        return self.bus.mode_read(self.id)

    def set_powered(self, powered: bool) -> None:
        return self.bus.set_powered(self.id, powered)

    def is_powered(self) -> bool:
        return self.bus.is_powered(self.id)

    def led_ctrl_write(self, *args, **kwargs) -> None:
        return self.bus.led_ctrl_write(self.id, *args, **kwargs)

    def led_ctrl_read(self) -> bool:
        return self.bus.led_ctrl_read(self.id)

    def led_error_write(self, *args, **kwargs) -> None:
        return self.bus.led_error_write(self.id, *args, **kwargs)

    def led_error_read(self) -> Tuple[bool, bool, bool]:
        return self.bus.led_error_read(self.id)


class ServoBus:
    """
    Controls bus servos using the LewanSoul Bus Servo Communication Protocol.

    Example usage::

        # Create a new ServoBus instance using a "with ..." statement,
        # which by default will power on all the servos.
        with ServoBus(...) as servo_bus:
            # Move the servo with ID 2 to the 120 degree position over 1 second
            servo_bus.move_time_write(2, 120, 1)

            # Wait a second for the servo to get in position before leaving the
            # "with ..." statement, which by default will power off all the
            # servos before closing the serial connection.
            time.sleep(1)
    """

    def __init__(
            self,
            port: Optional[str] = None,
            timeout: float = 1.0,
            baudrate: int = 115200,
            serial_conn=None,
            on_enter_power_on: bool = False,
            on_exit_power_off: bool = True,
            discard_echo: bool = True,
            verify_checksum: bool = True
    ) -> None:
        """
        :param port: The serial port to connect to.
            Ignored if `serial_conn` is given.
        :param baudrate: The baud rate to use. Hiwonder servos use 115200.
            Ignored if `serial_conn` is given.
        :param timeout: How long to wait for a response from the controller
            before timing out. None means never timeout.
            Ignored if `serial_conn` is given.
        :param serial_conn: Serial connection object to use. Useful if you
            already have a connection, or if you want to wrap this protocol in
            your own protocol. Must have at least `read(...)` and `write(...)`
            methods.
        :param on_enter_power_on: If the servos should be powered on when
            entering a "with ..." statement.
        :param on_exit_power_off: If the servos should be powered off when
            exiting a "with ..." statement.
        :param discard_echo: Set to True (the default) if all bytes sent are
            echoed back by the hardware. If this is True, then after this sends
            some number of bytes to the servos, it will automatically
            read/discard that same number of bytes from the receive buffer.
        :param verify_checksum: If the checksum byte of received packets should
            be checked.
        """

        self.on_enter_power_on = on_enter_power_on
        self.on_exit_power_off = on_exit_power_off
        self.discard_echo = discard_echo
        self.verify_checksum = verify_checksum

        if serial_conn:
            self._serial_conn = serial_conn
            self._close_on_exit = False
        else:
            self._serial_conn = serial.Serial(
                port=port, baudrate=baudrate, timeout=timeout)
            self._close_on_exit = True

        self._serial_conn_lock = RLock()

    def __enter__(self):
        if self.on_enter_power_on:
            self.set_powered(BROADCAST_ID, True)

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        try:
            if self.on_exit_power_off:
                self.set_powered(BROADCAST_ID, False)
        finally:
            if self._close_on_exit:
                with self._serial_conn_lock:
                    self.serial_conn.close()

    @property
    def serial_conn(self):
        return self._serial_conn

    def _send_packet(self, servo_id: int, command: int,
                     parameters: Union[bytearray, bytes] = None) -> None:
        # The LewanSoul servo command packet format is as follows:
        #
        #     | Header: byte[2] = 0x55 0x55 | ID: byte | Length: byte |
        #     | Command: byte | Parameter: byte[0..n] | Checksum: byte |
        #
        # - ID is the servo ID.
        # - Length is the number of bytes being sent after and including this
        #   byte.
        # - Checksum is the LSB of ~(ID + Length + Command + Parameter0 +
        #   ...ParameterN).

        if servo_id < MIN_ID or servo_id > BROADCAST_ID:
            raise ValueError(
                f'servo_id must be in range [{MIN_ID}, {BROADCAST_ID}]; '
                f'got {servo_id}.')
        if command < 0 or command > 255:
            raise ValueError(
                f'command must be in range [0, 255]; got {command}.')

        if parameters is None:
            parameters = b''

        servo_packet = bytearray(_PACKET_HEADER)
        servo_packet.append(servo_id)
        length = 3 + len(parameters)
        servo_packet.append(length)
        servo_packet.append(command)
        if parameters:
            servo_packet.extend(parameters)

        checksum = _calculate_checksum(servo_id, length, command, parameters)
        servo_packet.append(checksum)

        with self._serial_conn_lock:
            # Clear the input buffer
            try:
                self.serial_conn.reset_input_buffer()
            except AttributeError:
                pass

            # Send the packet
            self.serial_conn.write(servo_packet)

            # Discard echoed bytes
            if self.discard_echo:
                self.serial_conn.read(len(servo_packet))

    def _receive_packet(self) -> _ServoPacket:
        with self._serial_conn_lock:
            header = self.serial_conn.read(2)
            if header != _PACKET_HEADER:
                raise ServoBusError(
                    f'Expected header {repr(_PACKET_HEADER)}; '
                    f'received header {repr(header)}.')

            servo_id, length, command = self.serial_conn.read(3)
            param_count = length - 3
            parameters = self.serial_conn.read(param_count)
            checksum = self.serial_conn.read(1)[0]

        if self.verify_checksum:
            actual_checksum = _calculate_checksum(servo_id, length, command,
                                                  parameters)
            if checksum != actual_checksum:
                raise ServoBusError(
                    f'Checksum failed for received packet! '
                    f'Received checksum = {checksum}. '
                    f'Actual checksum = {actual_checksum}.'
                )

        return _ServoPacket(servo_id, command, parameters)

    def _send_and_receive_packet(
            self, servo_id: int,
            command: int,
            parameters: Union[bytearray, bytes] = None
    ) -> _ServoPacket:
        with self._serial_conn_lock:
            self._send_packet(servo_id, command, parameters=parameters)
            response = self._receive_packet()

        # Make sure received packet servo ID matches
        if response.servo_id != servo_id:
            raise ServoBusError(
                f'Received packet servo ID ({response.servo_id}) does not '
                f'match sent packet servo ID ({servo_id}).'
            )

        # Make sure received packet command matches
        if response.command != command:
            raise ServoBusError(
                f'Received packet command ({response.command}) does not '
                f'match sent packet command ({command}).'
            )

        return response

    def get_servo(self, servo_id: int, name: str = None) -> Servo:
        return Servo(servo_id, self, name=name)

    def _move_time_write(self, servo_id: int, angle_degrees: Real, time_s: Real,
                         command: int, wait: bool) -> None:
        """
        :param servo_id:
        :param angle_degrees: Should be in the range [0, 240] degrees; will be
            truncated if outside this range.
        :param time_s: Should be in the range [0, 30] seconds; will be truncated
            if outside this range.
        :param wait: Whether or not to wait time_s seconds after sending the
            command.
        :param command: Acceptable values are _SERVO_MOVE_TIME_WRITE, or
            _SERVO_MOVE_TIME_WAIT_WRITE.
        """

        if command not in {_SERVO_MOVE_TIME_WRITE, _SERVO_MOVE_TIME_WAIT_WRITE}:
            raise ValueError(
                f'Command must be either {_SERVO_MOVE_TIME_WRITE} or '
                f'{_SERVO_MOVE_TIME_WAIT_WRITE}; got {command}.'
            )

        angle_degrees = truncate_angle(angle_degrees)
        angle = _degrees_to_ticks(angle_degrees)

        time_s = min(max(0, time_s), 30)
        time_ms = int(round(time_s * 1000))

        params = _2_UNSIGNED_SHORTS_STRUCT.pack(angle, time_ms)
        self._send_packet(servo_id, command, params)

        if wait:
            time.sleep(time_s)

    def move_time_write(self, servo_id: int, angle_degrees: Real, time_s: Real,
                        wait: bool = False) -> None:
        """
        Tells the servo to start moving to the specified angle within the
        specified amount of time.

        :param servo_id:
        :param angle_degrees: Should be in the range [0, 240] degrees; will be
            truncated if outside this range.
        :param time_s: Should be in the range [0, 30] s; will be truncated if
            outside this range.
        :param wait: Whether or not to wait time_s seconds after sending the
            command.
        """

        return self._move_time_write(servo_id, angle_degrees, time_s,
                                     _SERVO_MOVE_TIME_WRITE, wait)

    def move_time_wait_write(self, servo_id: int, angle_degrees: Real,
                             time_s: Real) -> None:
        """
        Just like move_time_write(), except the servo will not start moving
        until move_start() is called.

        :param servo_id:
        :param angle_degrees: Should be in the range [0, 240] degrees; will be
            truncated if outside this range.
        :param time_s: Should be in the range [0, 30] s; will be truncated if
            outside this range.
        """

        return self._move_time_write(servo_id, angle_degrees, time_s,
                                     _SERVO_MOVE_TIME_WAIT_WRITE, False)

    def _move_time_read(
            self, servo_id: int, command: int
    ) -> Tuple[float, float]:
        """
        :param servo_id:
        :param command: Either _SERVO_MOVE_TIME_READ or
            _SERVO_MOVE_TIME_WAIT_READ.
        :return: A tuple of (angle_degrees, time_s).
        """

        if command not in {_SERVO_MOVE_TIME_READ, _SERVO_MOVE_TIME_WAIT_READ}:
            raise ValueError(
                f'Command must be either {_SERVO_MOVE_TIME_READ} or '
                f'{_SERVO_MOVE_TIME_WAIT_READ}; got {command}.'
            )

        response = self._send_and_receive_packet(servo_id, command)

        angle, time_ms = _2_UNSIGNED_SHORTS_STRUCT.unpack(response.parameters)

        angle_degrees = _ticks_to_degrees(angle)
        time_s = time_ms / 1000

        return angle_degrees, time_s

    def move_time_read(self, servo_id: int) -> Tuple[float, float]:
        """
        Returns the parameters set by the last call to move_time_write().

        :param servo_id:
        :return: A tuple of (angle_degrees, time_s).
        """

        return self._move_time_read(servo_id, command=_SERVO_MOVE_TIME_READ)

    def move_time_wait_read(self, servo_id: int) -> Tuple[float, float]:
        """
        Returns the parameters set by the last call to move_time_wait_write().

        :param servo_id:
        :return: A tuple of (angle_degrees, time_s).
        """

        return self._move_time_read(
            servo_id, command=_SERVO_MOVE_TIME_WAIT_READ)

    def move_speed_write(self, servo_id: int, angle_degrees: Real,
                         speed_dps: Real, wait: bool = False) -> None:
        """
        Tells the servo to go to the specified angle at a certain speed.

        :param servo_id:
        :param angle_degrees: Should be in the range [0, 240] degrees; will be
            truncated if outside this range.
        :param speed_dps: The speed in degrees-per-second that the servo should
            move to the target.
        :param wait: Whether or not to wait until the move is complete.
        """

        current_angle = self.pos_read(servo_id)
        error = abs(angle_degrees - current_angle)
        time_s = error / speed_dps

        self.move_time_write(servo_id, angle_degrees, time_s, wait=wait)

    def velocity_read(
            self, *servo_ids: int, period_s: Real = 0.1
    ) -> List[float]:
        """
        Determines the servo's current velocity by taking two position
        measurements period_s seconds apart.

        :param servo_ids: One or more servo IDs.
        :param period_s: The observation period -- how long to wait between
            position measurements.
        :return: The current velocity of the servo in degrees per second
            (may be negative).
        """

        measurements0 = [(time.monotonic(), self.pos_read(servo_id)) for
                         servo_id in servo_ids]
        time.sleep(period_s)
        measurements1 = [(time.monotonic(), self.pos_read(servo_id)) for
                         servo_id in servo_ids]

        velocities = []
        for measurement0, measurement1 in zip(measurements0, measurements1):
            time0, position0 = measurement0
            time1, position1 = measurement1
            velocities.append((position1 - position0) / (time1 - time0))

        return velocities

    def move_start(self, servo_id: int) -> None:
        """
        Tells the servo to start the motion specified by the last call to
        move_time_wait_write().
        """

        self._send_packet(servo_id, _SERVO_MOVE_START)

    def move_stop(self, servo_id: int) -> None:
        """Stop the servo at whatever its current angle is."""
        self._send_packet(servo_id, _SERVO_MOVE_STOP)

    def id_write(self, old_id: int, new_id: int) -> None:
        """Give the specified servo a new ID."""

        if old_id < MIN_ID or old_id > MAX_ID:
            raise ValueError(
                f'old_id must be in range [{MIN_ID}, {MAX_ID}]; got {old_id}.')
        if new_id < MIN_ID or new_id > MAX_ID:
            raise ValueError(
                f'new_id must be in range [{MIN_ID}, {MAX_ID}]; got {new_id}.')

        if new_id != old_id:
            self._send_packet(old_id, _SERVO_ID_WRITE, bytes((new_id,)))

    def angle_offset_adjust(self, servo_id: int, offset_degrees: Real,
                            write: bool = True) -> None:
        """
        Sets the servo's angle offset.

        :param servo_id:
        :param offset_degrees: Servo angle offset, in the range [-30, +30]
            degrees.
        :param write: If True, then angle_offset_write(servo_id) will be called
            after the offset adjustment has been made. Otherwise, the offset
            adjustment will be lost after the servo loses power.
        """

        if offset_degrees < -30 or offset_degrees > 30:
            raise ValueError(
                f'offset_degrees must be in range [-30, 30]; '
                f'got {offset_degrees}.')

        offset = int(round(offset_degrees * 125 / 30))
        params = _1_SIGNED_CHAR_STRUCT.pack(offset)
        self._send_packet(servo_id, _SERVO_ANGLE_OFFSET_ADJUST, params)

        if write:
            self.angle_offset_write(servo_id)

    def angle_offset_write(self, servo_id: int) -> None:
        """
        Saves the offset adjust value set by angle_offset_adjust()
        so that it will persist after the servo loses power.
        """

        self._send_packet(servo_id, _SERVO_ANGLE_OFFSET_WRITE)

    def angle_offset_read(self, servo_id: int) -> float:
        """
        :return: The angle offset of the servo in degrees. Defaults to 0.
            Range is [-30, 30].
        """

        response = self._send_and_receive_packet(servo_id,
                                                 _SERVO_ANGLE_OFFSET_READ)
        offset = _1_SIGNED_CHAR_STRUCT.unpack(response.parameters)[0]
        return offset * 30 / 125

    def angle_limit_write(self, servo_id: int, min_angle_degrees: Real,
                          max_angle_degrees: Real) -> None:
        """
        Sets the minimum and maximum servo angles allowed.
        This setting will persist after the servo loses power.

        :param servo_id:
        :param min_angle_degrees: Minimum servo angle, in the range [0, 240]
            degrees. Will be truncated if out of range. Must be lower than
            max_angle_degrees.
        :param max_angle_degrees: Maximum servo angle, in the range [0, 240]
            degrees. Will be truncated if out of range. Must be higher than
            min_angle_degrees.
        """

        min_angle_degrees = truncate_angle(min_angle_degrees)
        max_angle_degrees = truncate_angle(max_angle_degrees)

        min_angle = _degrees_to_ticks(min_angle_degrees)
        max_angle = _degrees_to_ticks(max_angle_degrees)

        if min_angle >= max_angle:
            raise ValueError(
                f'min_angle_degrees must be less than max_angle_degrees; '
                f'got min_angle_degrees={min_angle_degrees} '
                f'(==> min_angle={min_angle}) '
                f'and max_angle_degrees={max_angle_degrees} '
                f'(==> max_angle={max_angle}).')

        params = _2_UNSIGNED_SHORTS_STRUCT.pack(min_angle, max_angle)
        self._send_packet(servo_id, _SERVO_ANGLE_LIMIT_WRITE, params)

    def angle_limit_read(self, servo_id: int) -> Tuple[float, float]:
        """
        Gets the angle limits as set by angle_limit_write().

        :return: The angle limits as a tuple of
            (min_angle_degrees, max_angle_degrees).
        """

        response = self._send_and_receive_packet(servo_id,
                                                 _SERVO_ANGLE_LIMIT_READ)

        min_angle, max_angle = _2_UNSIGNED_SHORTS_STRUCT.unpack(
            response.parameters)

        min_angle_degrees = _ticks_to_degrees(min_angle)
        max_angle_degrees = _ticks_to_degrees(max_angle)

        return min_angle_degrees, max_angle_degrees

    def vin_limit_write(self, servo_id: int, min_voltage: Real,
                        max_voltage: Real) -> None:
        """
        Sets the minimum and maximum supply voltages allowed to power the
        servo. If the supply voltage goes outside the specified range,
        the servos will be powered off. This setting will persist after a
        servo loses power.

        :param servo_id:
        :param min_voltage: Should be in the range [4.5, 12] volts; anything out
            of range will be truncated. This should be lower than max_voltage.
        :param max_voltage: Should be in the range [4.5, 12] volts; anything out
            of range will be truncated. This should be higher than min_voltage.
        """

        def scrub_voltage(v: Real) -> int:
            """Converts to mV and limits to range [4500, 12000]."""
            v = int(round(v * 1000))
            return min(max(4500, v), 12000)

        min_voltage_mv = scrub_voltage(min_voltage)
        max_voltage_mv = scrub_voltage(max_voltage)

        if min_voltage_mv > max_voltage_mv:
            raise ValueError(
                f'min_voltage must be less than max_voltage; '
                f'got min_voltage={min_voltage} '
                f'(==> min_voltage_mv={min_voltage_mv}) '
                f'and max_voltage={max_voltage} '
                f'(==> max_voltage_mv={max_voltage_mv}).')

        params = _2_UNSIGNED_SHORTS_STRUCT.pack(min_voltage_mv, max_voltage_mv)
        self._send_packet(servo_id, _SERVO_VIN_LIMIT_WRITE, params)

    def vin_limit_read(self, servo_id: int) -> Tuple[float, float]:
        """
        Gets the input voltage limits as set by vin_limit_write().

        :param servo_id:
        :return: A tuple of the (min_voltage, max_voltage) settings in Volts.
        """

        response = self._send_and_receive_packet(servo_id,
                                                 _SERVO_VIN_LIMIT_READ)

        min_voltage_mv, max_voltage_mv = _2_UNSIGNED_SHORTS_STRUCT.unpack(
            response.parameters)
        min_voltage = min_voltage_mv / 1000
        max_voltage = max_voltage_mv / 1000

        return min_voltage, max_voltage

    def temp_max_limit_write(self, servo_id: int, temp: Real,
                             units: Literal['C', 'F'] = 'F') -> None:
        """
        Sets the maximum allowed temperature at which the servo will operate.
        If the servo temperature exceeds this limit, then the motor will be
        powered off. This setting persists after a servo loses power.

        :param servo_id:
        :param temp: Max allowed temperature. Should be in range [50, 100] C or
            [122, 212] F. Will be truncated if out of range. Default is 85 C or
            185 F.
        :param units: Use 'C' for Celsius or 'F' for Fahrenheit.
        """

        units = _validate_temp_units(units)

        if units == 'F':
            temp = _fahrenheit_to_celsius(temp)

        temp = int(round(temp))
        temp = min(max(50, temp), 100)

        self._send_packet(servo_id, _SERVO_TEMP_MAX_LIMIT_WRITE, bytes((temp,)))

    def temp_max_limit_read(self, servo_id: int,
                            units: Literal['C', 'F'] = 'F') -> float:
        """
        Gets the maximum temperature limit s set by temp_max_limit_write().

        :param servo_id:
        :param units: Use 'C' for Celsius or 'F' for Fahrenheit.
        :return: The max allowed temperature, in degrees Celsius or Fahrenheit.
        """

        units = _validate_temp_units(units)

        response = self._send_and_receive_packet(servo_id,
                                                 _SERVO_TEMP_MAX_LIMIT_READ)

        temp = float(response.parameters[0])
        if units == 'F':
            temp = _celsius_to_fahrenheit(temp)

        return temp

    def temp_read(self, servo_id: int, units: Literal['C', 'F'] = 'F') -> float:
        """
        Reads the temperature of the servo and returns in units of either
        Celsius or Fahrenheit.

        :param servo_id:
        :param units: Use 'C' for Celsius or 'F' for Fahrenheit.
        """

        units = _validate_temp_units(units)

        response = self._send_and_receive_packet(servo_id, _SERVO_TEMP_READ)

        # This is initially in Celsius
        temp = float(response.parameters[0])

        # Convert to Fahrenheit?
        if units.upper() == 'F':
            temp = _celsius_to_fahrenheit(temp)

        return temp

    def vin_read(self, servo_id: int) -> float:
        """Gets the input voltage to the servo."""

        response = self._send_and_receive_packet(servo_id, _SERVO_VIN_READ)

        vin_mv = _1_SIGNED_SHORT_STRUCT.unpack(response.parameters)[0]

        return vin_mv / 1000

    def pos_read(self, servo_id: int) -> float:
        """Gets the servo angle, in degrees. May be negative."""

        response = self._send_and_receive_packet(servo_id, _SERVO_POS_READ)

        angle = _1_SIGNED_SHORT_STRUCT.unpack(response.parameters)[0]

        return _ticks_to_degrees(angle)

    def mode_write(self, servo_id: int, mode: Literal['motor', 'servo'],
                   speed: Real = None) -> None:
        """
        Sets the servo mode to either 'motor' -- where it rotates
        continuously at a certain speed -- or 'servo' -- where it holds to a
        specific position.

        :param servo_id:
        :param mode: Either 'motor' or 'servo'. If 'motor', then speed must be
            specified.
        :param speed: In the range [-1000, 1000]. Will be truncated if out of
            range. Ignored if mode is 'servo'.
        """

        if mode.lower() not in {'motor', 'servo'}:
            raise ValueError(
                f'mode must be either "motor" or "servo"; got "{mode}".')

        mode = mode.lower()
        if mode == 'motor':
            if speed is None:
                raise ValueError(f'speed must be specified if mode is "motor".')

            speed = int(round(speed))
            speed = min(max(-1000, speed), 1000)
        else:
            speed = 0

        params = _1_UNSIGNED_CHAR_1_UNSIGNED_SHORT_STRUCT.pack(
            1 if mode == 'motor' else 0, speed)
        self._send_packet(servo_id, _SERVO_OR_MOTOR_MODE_WRITE, params)

    def mode_read(self, servo_id: int) -> Tuple[str, Optional[int]]:
        """
        Gets the mode the servo is currently set to, based on the last call
        to mode_write().

        :param servo_id:
        :return: A tuple of ('motor' or 'servo', speed or None).
        """

        response = self._send_and_receive_packet(servo_id,
                                                 _SERVO_OR_MOTOR_MODE_READ)

        mode, speed = _1_UNSIGNED_CHAR_1_UNSIGNED_SHORT_STRUCT.unpack(
            response.parameters)

        if mode == 0:
            mode = 'servo'
            speed = None
        elif mode == 1:
            mode = 'motor'
            speed = int(speed)
        else:
            raise ValueError(f'Received unknown mode: {mode}')

        return mode, speed

    def set_powered(self, servo_id: int, powered: bool) -> None:
        """
        Sets whether or not the servo is powered and attempting to hold its
        position.
        """

        self._send_packet(servo_id, _SERVO_LOAD_OR_UNLOAD_WRITE,
                          b'\x01' if powered else b'\x00')

    def is_powered(self, servo_id: int) -> bool:
        """
        Gets whether or not the servo is powered and attempting to hold its
        position.
        """

        if servo_id < MIN_ID or servo_id > MAX_ID:
            raise ValueError(
                f'servo_id must be in range [{MIN_ID}, {MAX_ID}]; '
                f'got {servo_id}.')

        response = self._send_and_receive_packet(servo_id,
                                                 _SERVO_LOAD_OR_UNLOAD_READ)
        return bool(response.parameters[0])

    def led_ctrl_write(self, servo_id: int, state: bool) -> None:
        """
        Sets whether or not the LED should be on when there are no errors.

        :param servo_id:
        :param state: Whether or not the LED should be on when there are no
            errors.
        """

        self._send_packet(servo_id, _SERVO_LED_CTRL_WRITE,
                          b'\x00' if state else b'\x01')

    def led_ctrl_read(self, servo_id: int) -> bool:
        """
        Gets whether or not the LED is on when there are no errors.

        :param servo_id:
        :return: Whether or not the LED is on when there are no errors.
        """

        response = self._send_and_receive_packet(servo_id, _SERVO_LED_CTRL_READ)
        return response.parameters == b'\x00'

    def led_error_write(self, servo_id: int, stalled: bool, over_voltage: bool,
                        over_temp: bool) -> None:
        """
        Sets which error conditions will cause the LED to indicate an error has
        occurred.

        :param servo_id:
        :param stalled:
        :param over_voltage:
        :param over_temp:
        """

        params = (stalled << 2) | (over_voltage << 1) | over_temp
        params = bytes((params,))
        self._send_packet(servo_id, _SERVO_LED_ERROR_WRITE, params)

    def led_error_read(self, servo_id: int) -> Tuple[bool, bool, bool]:
        """
        Returns which error conditions will cause the LED to indicate an error
        has occurred.

        Note: This is NOT the error condition that the servo is currently
        experiencing. There appears to be no way to get that information.

        :param servo_id:
        :return: A tuple of booleans (stalled, over_voltage, over_temp).
        """

        result = self._send_and_receive_packet(servo_id, _SERVO_LED_ERROR_READ)
        params = result.parameters[0]

        stalled = bool(params & 4)
        over_voltage = bool(params & 2)
        over_temp = bool(params & 1)

        return stalled, over_voltage, over_temp


class ServoBusError(Exception):
    pass


def truncate_angle(angle_degrees: Real) -> Real:
    """:return: The angle, truncated to be in the range [0, 240] degrees."""

    return min(max(MIN_ANGLE_DEGREES, angle_degrees), MAX_ANGLE_DEGREES)


def _calculate_checksum(servo_id: int, length: int, command: int,
                        parameters: Union[bytearray, bytes]) -> int:
    checksum = servo_id + length + command + sum(parameters)
    checksum = ~checksum & 0xFF
    return checksum


def _celsius_to_fahrenheit(temp: Real) -> float:
    return (temp * 9 / 5) + 32


def _fahrenheit_to_celsius(temp: Real) -> float:
    return (temp - 32) * 5 / 9


def _degrees_to_ticks(degrees: Real) -> int:
    return int(float(degrees * 1000 / MAX_ANGLE_DEGREES))


def _ticks_to_degrees(ticks: int) -> float:
    return ticks * MAX_ANGLE_DEGREES / 1000


def _validate_temp_units(units: str) -> str:
    if units.upper() not in {'C', 'F'}:
        raise ValueError(f'Units must be either "C" or "F"; got "{units}".')

    return units.upper()
