from typing import Optional

from lewansoul_servo_bus import ServoBus

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

        self.servos = (self.gripper, self.wrist, self.outer_elbow, self.inner_elbow, self.shoulder, self.base)

    def __str__(self) -> str:
        return self.name or super().__str__()
