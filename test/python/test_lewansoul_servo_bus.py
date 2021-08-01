import unittest
from typing import Optional, Union

import lewansoul_servo_bus


class ServoTest(unittest.TestCase):
    ID = 123
    SERVO_BUS = lewansoul_servo_bus.ServoBus()
    NAME = 'claw'

    def setUp(self) -> None:
        self.servo = lewansoul_servo_bus.Servo(
            self.ID, self.SERVO_BUS, name=self.NAME)

    def test_init(self):
        self.assertEqual(self.ID, self.servo.id)
        self.assertIs(self.SERVO_BUS, self.servo.bus)
        self.assertEqual(self.NAME, self.servo.name)

    def test_str__with_name(self):
        name = 'wrist'

        servo = lewansoul_servo_bus.Servo(self.ID, self.SERVO_BUS, name=name)

        self.assertEqual(f'{name} (ID {self.ID})', str(servo))

    def test_str__without_name(self):
        servo = lewansoul_servo_bus.Servo(self.ID, self.SERVO_BUS)

        self.assertEqual(f'Servo (ID {self.ID})', str(servo))


class ServoBusTest(unittest.TestCase):
    SERVO_ID = 123

    def assert_data_written(self, data: bytes):
        self.assertEqual(data, self.servo_bus.serial_conn.data_written)

    def setup_servo_bus(self, serial_conn_read_data: bytes) -> None:
        """
        Creates a MockSerial object with the given read_data, and sets
        `self.servo_bus` to a new ServoBus instance using the MockSerial
        instance as the servo_conn.
        """

        serial_conn = MockSerial(serial_conn_read_data)
        self.servo_bus = lewansoul_servo_bus.ServoBus(
            serial_conn=serial_conn, discard_echo=False)

    def test_get_servo(self):
        servo_bus = lewansoul_servo_bus.ServoBus()
        name = 'gripper'

        servo = servo_bus.get_servo(self.SERVO_ID, name=name)

        self.assertEqual(self.SERVO_ID, servo.id)
        self.assertIs(servo_bus, servo.bus)
        self.assertEqual(name, servo.name)

    def test_move_time_write(self):
        self.setup_servo_bus(b'')

        self.servo_bus.move_time_write(123, 180, 2.5)

        self.assert_data_written(b'UU{\x07\x01\xee\x02\xc4\t\xbf')

    def test_move_time_read(self):
        self.setup_servo_bus(b'UU{\x07\x02\xee\x02\xc4\t\xbe')

        angle_degrees, time_s = self.servo_bus.move_time_read(123)

        self.assert_data_written(b'UU{\x03\x02\x7f')
        self.assertEqual(180, angle_degrees)
        self.assertAlmostEqual(2.5, time_s)


class MockSerial:
    """Mocks a Serial connection object for testing."""

    def __init__(self, read_data: bytes) -> None:
        self.read_data = read_data
        self.read_data_index = 0
        self.data_written = bytearray()

    def read(self, size: int = 1) -> bytes:
        i = self.read_data_index
        self.read_data_index += size
        return self.read_data[i:i + size]

    def write(self, data: Union[bytearray, bytes]) -> Optional[int]:
        self.data_written.extend(data)
        return len(data)
