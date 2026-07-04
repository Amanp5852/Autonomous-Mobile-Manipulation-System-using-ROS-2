#!/usr/bin/env python3

from enum import Enum, auto
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Bool

from .poses import *
from .config import *

# ------------------------------------------------------------
# Manipulation State Machine
# ------------------------------------------------------------

class ManipulationState(Enum):

    IDLE = auto()

    MOVE_JOINT0 = auto()

    MOVE_JOINT3 = auto()

    MOVE_JOINT2 = auto()

    MOVE_JOINT1 = auto()

    MOVE_GRIPPER = auto()

    RETURN_JOINT1 = auto()
    RETURN_JOINT2 = auto()
    RETURN_JOINT3 = auto()
    RETURN_JOINT0 = auto()

    COMPLETE = auto()

# ------------------------------------------------------------
# Current Joint Values
# ------------------------------------------------------------

@dataclass
class CurrentJointState:

    joint0: float = 0.0
    joint1: float = 0.0
    joint2: float = 0.0
    joint3: float = 0.0
    joint4: float = 0.0
    joint5: float = 0.0

class ManipulationNode(Node):

    def __init__(self):

        super().__init__("manipulation_node")

        self.get_logger().info(
            "Manipulation Node Started"
        )

        self.current = CurrentJointState()

        self.joint_state_received = False

        self.target_pose = None

        # Current manipulation command
        self.current_command = None

        # True while returning to HOME pose
        self.returning_home = False

        self.gripper_close_time = None

        self.state = ManipulationState.IDLE

        self.joint0_pub = self.create_publisher(
            Float64,
            "/joint0/cmd_pos",
            10
        )

        self.joint1_pub = self.create_publisher(
            Float64,
            "/joint1/cmd_pos",
            10
        )

        self.joint2_pub = self.create_publisher(
            Float64,
            "/joint2/cmd_pos",
            10
        )

        self.joint3_pub = self.create_publisher(
            Float64,
            "/joint3/cmd_pos",
            10
        )

        self.joint4_pub = self.create_publisher(
            Float64,
            "/joint4/cmd_pos",
            10
        )

        self.joint5_pub = self.create_publisher(
            Float64,
            "/joint5/cmd_pos",
            10
        )

        self.status_pub = self.create_publisher(
            String,
            "/manipulation_status",
            10
        )

        self.done_pub = self.create_publisher(
            Bool,
            "/manipulation_done",
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10
        )
        
        self.command_sub = self.create_subscription(
            String,
            "/manipulation_command",
            self.command_callback,
            10
        )

        self.timer = self.create_timer(

            1.0 / CONTROL_RATE,

            self.control_loop

        )


    # ------------------------------------------------------------
    # Joint State Callback
    # ------------------------------------------------------------

    def joint_state_callback(self, msg):

        joint_map = dict(zip(msg.name, msg.position))
        

        self.current.joint0 = joint_map.get(
            "arm_base_forearm_joint",
            self.current.joint0
        )

        self.current.joint1 = joint_map.get(
            "forearm_hand_joint",
            self.current.joint1
        )

        self.current.joint2 = joint_map.get(
            "hand_gripper_arm_joint",
            self.current.joint2
        )

        self.current.joint3 = joint_map.get(
            "gripper_arm_base_joint",
            self.current.joint3
        )

        self.current.joint4 = joint_map.get(
            "gripper_base_finger_right_joint",
            self.current.joint4
        )

        self.current.joint5 = joint_map.get(
            "gripper_base_finger_left_joint",
            self.current.joint5
        )

        self.joint_state_received = True

    # ------------------------------------------------------------
    # Manipulation Command Callback
    # ------------------------------------------------------------

    def command_callback(self, msg):

        command = msg.data.upper()

        if self.state != ManipulationState.IDLE:

            self.get_logger().warn(

                "Manipulator Busy"

            )

            return

        self.get_logger().info(

            f"Received Command : {command}"

        )

        if command == "HOME":

            self.move_to_pose(HOME_POSE)

        elif command.endswith("_PICKUP"):

            self.current_command = command

            self.returning_home = False

            self.move_to_pose(command)

        elif command.endswith("_PLACE"):

            self.current_command = command

            self.returning_home = False

            self.move_to_pose(command)

        else:

            self.get_logger().warn(

                f"Unknown Command : {command}"

            )

    # ------------------------------------------------------------
    # Publish Status
    # ------------------------------------------------------------

    def publish_status(self, text):

        msg = String()

        msg.data = text

        self.status_pub.publish(msg)

    # ------------------------------------------------------------
    # Publish Done
    # ------------------------------------------------------------

    def publish_done(self, done):

        msg = Bool()

        msg.data = done

        self.done_pub.publish(msg)

    # ------------------------------------------------------------
    # Joint Publishers
    # ------------------------------------------------------------

    def publish_joint0(self, value):

        msg = Float64()

        msg.data = value

        self.joint0_pub.publish(msg)


    def publish_joint1(self, value):

        msg = Float64()

        msg.data = value

        self.joint1_pub.publish(msg)


    def publish_joint2(self, value):

        msg = Float64()

        msg.data = value

        self.joint2_pub.publish(msg)


    def publish_joint3(self, value):

        msg = Float64()

        msg.data = value

        self.joint3_pub.publish(msg)


    def publish_joint4(self, value):

        msg = Float64()

        msg.data = value

        self.joint4_pub.publish(msg)


    def publish_joint5(self, value):

        msg = Float64()

        msg.data = value

        self.joint5_pub.publish(msg)

    # ------------------------------------------------------------
    # Joint Reached
    # ------------------------------------------------------------

    def joint_reached(self, current, target):

        return abs(current - target) < JOINT_TOLERANCE
    
    # ------------------------------------------------------------
    # Move To Pose
    # ------------------------------------------------------------

    def move_to_pose(self, pose):

        pose_table = {

            "HOME": HOME_POSE,

            "GREEN_PICKUP": GREEN_PICKUP_POSE,
            "RED_PICKUP": RED_PICKUP_POSE,
            "BLUE_PICKUP": BLUE_PICKUP_POSE,

            "GREEN_PLACE": GREEN_PLACE_POSE,
            "RED_PLACE": RED_PLACE_POSE,
            "BLUE_PLACE": BLUE_PLACE_POSE,

        }

        self.target_pose = pose_table[pose]

        self.publish_done(False)

        self.publish_status("START")

        self.get_logger().info(

            f"Starting Pose : {pose}"

        )

        self.state = ManipulationState.MOVE_JOINT0

    # ------------------------------------------------------------
    # Control Loop
    # ------------------------------------------------------------

    def control_loop(self):

        if not self.joint_state_received:

            return

        if self.target_pose is None:

            return
        
        
        # ------------------------------------------------------------
        # Joint 0
        # ------------------------------------------------------------

        if self.state == ManipulationState.MOVE_JOINT0:

            self.publish_status("MOVE_JOINT0")

            self.publish_joint0(
                self.target_pose.joint0
            )

            if self.joint_reached(

                self.current.joint0,

                self.target_pose.joint0

            ):

                self.state = ManipulationState.MOVE_JOINT3

                self.get_logger().info(
                    "Joint0 Reached"
                )
            
            self.get_logger().info(
                f"Joint0 Current={self.current.joint0:.3f} Target={self.target_pose.joint0:.3f}"
            )


        # ------------------------------------------------------------
        # Joint 3
        # ------------------------------------------------------------

        elif self.state == ManipulationState.MOVE_JOINT3:

            self.publish_status("MOVE_JOINT3")

            self.publish_joint3(
                self.target_pose.joint3
            )
           
            if self.joint_reached(

                self.current.joint3,

                self.target_pose.joint3

            ):

                self.state = ManipulationState.MOVE_JOINT2

                self.get_logger().info(
                    "Joint3 Reached"
                )

            self.get_logger().info(
                f"Joint3 Current={self.current.joint3:.3f} Target={self.target_pose.joint3:.3f}"
            )

        # ------------------------------------------------------------
        # Joint 2
        # ------------------------------------------------------------

        elif self.state == ManipulationState.MOVE_JOINT2:

            self.publish_status("MOVE_JOINT2")

            self.publish_joint2(
                self.target_pose.joint2
            )

            if self.joint_reached(

                self.current.joint2,

                self.target_pose.joint2

            ):

                self.state = ManipulationState.MOVE_JOINT1

                self.get_logger().info(
                    "Joint2 Reached"
                )

            self.get_logger().info(
                f"Joint2 Current={self.current.joint2:.3f} Target={self.target_pose.joint2:.3f}"
            )


        # ------------------------------------------------------------
        # Joint 1
        # ------------------------------------------------------------

        elif self.state == ManipulationState.MOVE_JOINT1:

            self.publish_status("MOVE_JOINT1")

            self.publish_joint1(
                self.target_pose.joint1
            )

            if self.joint_reached(

                self.current.joint1,

                self.target_pose.joint1

            ):

                
                self.get_logger().info(
                    "Joint1 Reached"
                )
                
                self.state = ManipulationState.MOVE_GRIPPER


            self.get_logger().info(
                f"Joint1 Current={self.current.joint1:.3f} Target={self.target_pose.joint1:.3f}"
            )

        elif self.state == ManipulationState.MOVE_GRIPPER:

            self.publish_status("MOVE_GRIPPER")

            self.publish_joint4(
                self.target_pose.joint4
            )

            self.publish_joint5(
                self.target_pose.joint5
            )    
        
        # ------------------------------------------------------------
        # Gripper
        # ------------------------------------------------------------

            if (

                self.joint_reached(

                    self.current.joint4,

                    self.target_pose.joint4

                )

                and

                self.joint_reached(

                    self.current.joint5,

                    self.target_pose.joint5

                )

            ):

                self.get_logger().info(

                    "Gripper Reached"

                )

                if self.gripper_close_time is None:

                    self.gripper_close_time = self.get_clock().now()

                    return
                
                elapsed = (
                    self.get_clock().now() - self.gripper_close_time
                ).nanoseconds / 1e9

                if elapsed < 0.5:

                    return

                self.gripper_close_time = None

                # --------------------------------------------
                # First pass
                # Return arm to HOME
                # --------------------------------------------

                if not self.returning_home:

                    self.get_logger().info(
                        "Returning To HOME Pose"
                    )

                    self.returning_home = True

                    if self.current_command == "PICKUP":

                        self.target_pose = ArmPose(

                            joint0=TRANSPORT_POSE.joint0,
                            joint1=TRANSPORT_POSE.joint1,
                            joint2=TRANSPORT_POSE.joint2,
                            joint3=TRANSPORT_POSE.joint3,
                            joint4=self.current.joint4,
                            joint5=self.current.joint5

                        )

                    else:

                        self.target_pose = ArmPose(

                            joint0=HOME_POSE.joint0,
                            joint1=HOME_POSE.joint1,
                            joint2=HOME_POSE.joint2,
                            joint3=HOME_POSE.joint3,
                            joint4=self.current.joint4,
                            joint5=self.current.joint5

                        )

                    self.state = ManipulationState.RETURN_JOINT1

                   
        # ------------------------------------------------------------
        # Return Joint 1
        # ------------------------------------------------------------

        elif self.state == ManipulationState.RETURN_JOINT1:

            self.publish_status("RETURN_JOINT1")

            self.publish_joint1(
                self.target_pose.joint1
            )

            if self.joint_reached(

                self.current.joint1,

                self.target_pose.joint1

            ):

                self.state = ManipulationState.RETURN_JOINT2

                self.get_logger().info(
                    "Return Joint1 Reached"
                )

            self.get_logger().info(
                f"Joint1 Current={self.current.joint1:.3f} "
                f"Target={self.target_pose.joint1:.3f}"
            )

        # ------------------------------------------------------------
        # Return Joint 2
        # ------------------------------------------------------------

        elif self.state == ManipulationState.RETURN_JOINT2:

            self.publish_status("RETURN_JOINT2")

            self.publish_joint2(
                self.target_pose.joint2
            )

            if self.joint_reached(

                self.current.joint2,

                self.target_pose.joint2

            ):

                self.state = ManipulationState.RETURN_JOINT3

                self.get_logger().info(
                    "Return Joint2 Reached"
                )

            self.get_logger().info(
                f"Joint2 Current={self.current.joint2:.3f} "
                f"Target={self.target_pose.joint2:.3f}"
            )

        # ------------------------------------------------------------
        # Return Joint 3
        # ------------------------------------------------------------

        elif self.state == ManipulationState.RETURN_JOINT3:

            self.publish_status("RETURN_JOINT3")

            self.publish_joint3(
                self.target_pose.joint3
            )

            if self.joint_reached(

                self.current.joint3,

                self.target_pose.joint3

            ):

                self.state = ManipulationState.RETURN_JOINT0

                self.get_logger().info(
                    "Return Joint3 Reached"
                )

            self.get_logger().info(
                f"Joint3 Current={self.current.joint3:.3f} "
                f"Target={self.target_pose.joint3:.3f}"
            )

        # ------------------------------------------------------------
        # Return Joint 0
        # ------------------------------------------------------------

        elif self.state == ManipulationState.RETURN_JOINT0:

            self.publish_status("RETURN_JOINT0")

            self.publish_joint0(
                self.target_pose.joint0
            )

            if self.joint_reached(

                self.current.joint0,

                self.target_pose.joint0

            ):

                self.state = ManipulationState.COMPLETE

                self.get_logger().info(
                    "Return Joint0 Reached"
                )

            self.get_logger().info(
                f"Joint0 Current={self.current.joint0:.3f} "
                f"Target={self.target_pose.joint0:.3f}"
            )
        # ------------------------------------------------------------
        # Complete
        # ------------------------------------------------------------

        elif self.state == ManipulationState.COMPLETE:

            self.publish_status("DONE")

            self.publish_done(True)
        
            self.returning_home = False

            self.current_command = None

            self.target_pose = None

            self.state = ManipulationState.IDLE

            self.publish_status("IDLE")

            self.get_logger().info(
                "Manipulation Complete"
            )


def main(args=None):

    rclpy.init(args=args)

    node = ManipulationNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":

    main()