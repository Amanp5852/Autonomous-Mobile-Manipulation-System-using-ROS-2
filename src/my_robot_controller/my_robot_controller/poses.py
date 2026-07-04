from dataclasses import dataclass

from .config import GRIPPER_OPEN, GRIPPER_CLOSED


# ------------------------------------------------------------
# Arm Pose Dataclass
# ------------------------------------------------------------

@dataclass
class ArmPose:

    joint0: float
    joint1: float
    joint2: float
    joint3: float
    joint4: float
    joint5: float

    def as_list(self):
        return [
            self.joint0,
            self.joint1,
            self.joint2,
            self.joint3,
            self.joint4,
            self.joint5
        ]

# ------------------------------------------------------------
# Home Pose
# ------------------------------------------------------------

HOME_POSE = ArmPose(

    joint0=0.0,
    joint1=0.0,
    joint2=0.0,
    joint3=0.0,
    joint4=0.0,
    joint5=0.0

)

TRANSPORT_POSE = ArmPose(

    joint0 = 0.0,

    joint1 = 0.0,

    joint2 = 0.0,

    joint3 = 0.0,

    joint4 = GRIPPER_CLOSED,

    joint5 = GRIPPER_CLOSED

)


# ------------------------------------------------------------
# Pickup Pose
# ------------------------------------------------------------

GREEN_PICKUP_POSE = ArmPose(
    joint0=0.95,
    joint1=0.90,
    joint2=-0.80,
    joint3=-1.16,
    joint4=0.04,
    joint5=0.04
)

RED_PICKUP_POSE = ArmPose(
    joint0=0.95,
    joint1=0.90,
    joint2=-0.80,
    joint3=-1.16,
    joint4=0.04,
    joint5=0.04
)

BLUE_PICKUP_POSE = ArmPose(
    joint0=0.95,
    joint1=0.90,
    joint2=-0.80,
    joint3=-1.16,
    joint4=0.04,
    joint5=0.04
)


# ------------------------------------------------------------
# Place Pose
# ------------------------------------------------------------

GREEN_PLACE_POSE = ArmPose(
    joint0=0.60,
    joint1=0.60,
    joint2=-0.35,
    joint3=-1.10,
    joint4=0.0,
    joint5=0.0
)

RED_PLACE_POSE = ArmPose(
    joint0=0.60,
    joint1=0.60,
    joint2=-0.35,
    joint3=-1.10,
    joint4=0.0,
    joint5=0.0
)

BLUE_PLACE_POSE = ArmPose(
    joint0=0.60,
    joint1=0.60,
    joint2=-0.35,
    joint3=-1.10,
    joint4=0.0,
    joint5=0.0
)
