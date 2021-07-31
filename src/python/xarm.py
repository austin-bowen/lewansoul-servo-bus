from typing import Optional, Tuple

from lewansoul_servo_bus import BROADCAST_ID, truncate_angle, Servo, ServoBus

# xArm servo IDs
XARM_GRIPPER_ID = 1
XARM_WRIST_ID = 2
XARM_OUTER_ELBOW_ID = 3
XARM_INNER_ELBOW_ID = 4
XARM_SHOULDER_ID = 5
XARM_BASE_ID = 6
XARM_SERVO_IDS = (
    XARM_GRIPPER_ID, XARM_WRIST_ID, XARM_OUTER_ELBOW_ID, XARM_INNER_ELBOW_ID, XARM_SHOULDER_ID, XARM_BASE_ID)


class Xarm:
    def __init__(
            self,
            servo_bus: ServoBus,
            name: Optional[str] = None,
            gripper_id: int = XARM_GRIPPER_ID,
            wrist_id: int = XARM_WRIST_ID,
            outer_elbow_id: int = XARM_OUTER_ELBOW_ID,
            inner_elbow_id: int = XARM_INNER_ELBOW_ID,
            shoulder_id: int = XARM_SHOULDER_ID,
            base_id: int = XARM_BASE_ID
    ) -> None:
        self.servo_bus = servo_bus
        self.name = name

        self.gripper = servo_bus.get_servo(gripper_id, name='gripper')
        self.wrist = servo_bus.get_servo(wrist_id, name='wrist')
        self.outer_elbow = servo_bus.get_servo(outer_elbow_id, name='outer elbow')
        self.inner_elbow = servo_bus.get_servo(inner_elbow_id, name='inner elbow')
        self.shoulder = servo_bus.get_servo(shoulder_id, name='shoulder')
        self.base = servo_bus.get_servo(base_id, name='base')

        self._servos = (self.gripper, self.wrist, self.outer_elbow, self.inner_elbow, self.shoulder, self.base)

    def __str__(self) -> str:
        return self.name or 'xArm'

    @property
    def servos(self) -> Tuple[Servo, Servo, Servo, Servo, Servo, Servo]:
        return self._servos


def control(servo_bus: ServoBus):
    print('Enter commands in the format (ID, angle [deg], time [s]):')

    servo_bus.set_powered(BROADCAST_ID, True)

    while True:
        try:
            command = input()
            servo_id, angle, time_s = [a.strip() for a in command.split(',')]
            servo_id, angle, time_s = int(servo_id), float(angle), float(time_s)
        except KeyboardInterrupt:
            print()
            return
        except Exception as e:
            print('Error:', e)
            continue

        servo_bus.move_time_write(servo_id, angle, time_s)
        # servo_bus.move_speed_write(servo_id, angle, time_s)

    servo_bus.set_powered(BROADCAST_ID, False)


def test(arm: Xarm) -> int:
    print('\nTesting pos_read and move_time_write...')

    error = False
    errors = 0
    delta = 20
    move_time = 1

    for servo in arm.servos:
        print(f'- {servo}:')

        # Get the servo position
        print('  - pos_read() -> ', end='', flush=True)
        try:
            degrees = servo.pos_read()
            print(degrees)
        except Exception as e:
            print(f'Error: {e}')
            error = True
            errors += 1
            continue

        # Determine new target position
        degrees = truncate_angle(degrees)
        target = (degrees - delta) if degrees > 120 else (degrees + delta)

        # Move the servo to the target position
        print(f'  - move_time_write({target}, {move_time}, wait=True) -> ', end='', flush=True)
        try:
            print(servo.move_time_write(target, move_time, wait=True))
        except Exception as e:
            print(f'Error: {e}')
            error = True
            errors += 1
            continue

        # Get the new servo position
        print('  - pos_read() -> ', end='', flush=True)
        try:
            new_degrees = servo.pos_read()
            print(new_degrees)
        except Exception as e:
            print(f'Error: {e}')
            error = True
            errors += 1
            continue

        # Servo does not appear to have moved?
        if abs(degrees - new_degrees) < 1:
            print(f'  - Error: Servo does not appear to have moved.')
            error = True
            errors += 1

        # Move the servo back to its original location
        print(f'  - move_time_write({degrees}, {move_time}, wait=True) -> ', end='', flush=True)
        try:
            print(servo.move_time_write(degrees, move_time, wait=True))
        except Exception as e:
            print(f'Error: {e}')
            error = True
            errors += 1

    print('- FAIL' if error else '- Pass')

    return errors


def watch_xarm_state(arm: Xarm) -> None:
    for servo in arm.servos:
        servo.set_powered(False)

    try:
        while True:
            positions = [servo.pos_read() for servo in arm.servos]
            positions = (int(round(p)) for p in positions)

            vs = [servo.velocity_read() for servo in arm.servos]
            vs = (int(round(v)) for v in vs)

            print()
            print('Servo:\t' + '\t'.join(map(str, XARM_SERVO_IDS)))
            print('Pos. :\t' + '\t'.join(map(str, positions)))
            print('Vel. :\t' + '\t'.join(map(str, vs)))
    except KeyboardInterrupt:
        print()


def main() -> None:
    # Get serial port
    try:
        serial_port = input('Enter serial port [/dev/ttyUSB0]: ')
    except KeyboardInterrupt:
        print()
        return

    if not serial_port:
        serial_port = '/dev/ttyUSB0'

    # Get choice of what to do
    print()
    print('Options:')
    print('1. Test - Automatically move each joint a small amount to verify functionality')
    print('2. Control - Use keyboard commands to control the arm')
    print('3. Watch State - Show various states of the arm in real time')
    print()

    try:
        choice = input('Choice: ')
    except KeyboardInterrupt:
        print()
        return

    if not choice:
        print('No choice given.')
        return

    choice = int(choice.strip())

    # Connect to the servo bus
    print()
    with ServoBus(serial_port_regexp=serial_port) as servo_bus:
        # Get an instance of an arm
        arm = Xarm(servo_bus)

        # Test the arm?
        if choice == 1:
            test(arm)

        # Control the arm?
        elif choice == 2:
            control(arm.servo_bus)

        # Watch the arm?
        elif choice == 3:
            watch_xarm_state(arm)

        # Invalid choice?
        else:
            print(f'Invalid choice!')


if __name__ == '__main__':
    main()
