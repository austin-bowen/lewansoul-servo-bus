import unittest
from typing import Optional, Union

import lewansoul_servo_bus


class ServoTest(unittest.TestCase):
    ID = 123
    SERVO_BUS = object()
    NAME = 'claw'

    def setUp(self) -> None:
        self.servo = lewansoul_servo_bus.Servo(
            self.ID, self.SERVO_BUS, name=self.NAME)

    def test_init(self):
        self.assertEqual(self.servo.id, self.ID)
        self.assertIs(self.servo.bus, self.SERVO_BUS)
        self.assertEqual(self.servo.name, self.NAME)

    def test_str__with_name(self):
        name = 'wrist'

        servo = lewansoul_servo_bus.Servo(self.ID, self.SERVO_BUS, name=name)

        self.assertEqual(str(servo), f'{name} (ID {self.ID})')

    def test_str__without_name(self):
        servo = lewansoul_servo_bus.Servo(self.ID, self.SERVO_BUS)

        self.assertEqual(str(servo), f'Servo (ID {self.ID})')


class ServoBusTest(unittest.TestCase):
    ...


class MockSerial:
    """Mocks a Serial connection object for testing."""

    def __init__(self, read_data: bytes) -> None:
        self.read_data = read_data
        self.read_data_index = 0
        self.write_data = bytearray()

    def read(self, size: int = 1) -> bytes:
        i = self.read_data_index
        self.read_data_index += size
        return self.read_data[i:i + size]

    def write(self, data: Union[bytearray, bytes]) -> Optional[int]:
        self.write_data.extend(data)
        return len(data)
