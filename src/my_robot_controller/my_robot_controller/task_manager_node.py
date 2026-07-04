#!/usr/bin/env python3

from enum import Enum, auto

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Bool

from geometry_msgs.msg import PoseStamped

from .location import *
from .config import *
import math

# ------------------------------------------------------------
# Task Manager State Machine
# ------------------------------------------------------------

class TaskState(Enum):

    START = auto()

    GO_TO_INSPECTION = auto()

    WAIT_FOR_NAVIGATION = auto()

    WAIT_FOR_COLOR = auto()

    GO_TO_PICKUP = auto()

    WAIT_FOR_PICKUP_NAVIGATION = auto()

    PICK_OBJECT = auto()

    WAIT_FOR_PICKUP = auto()

    GO_TO_DROP = auto()

    WAIT_FOR_DROP_NAVIGATION = auto()

    PLACE_OBJECT = auto()

    WAIT_FOR_PLACE = auto()

    RETURN_TO_INSPECTION = auto()

    WAIT_FOR_RETURN = auto()

    NEXT_INSPECTION = auto()

    MISSION_COMPLETE = auto()


# ------------------------------------------------------------
# Task Manager Node
# ------------------------------------------------------------

class TaskManagerNode(Node):

    def __init__(self):

        super().__init__("task_manager_node")

        self.get_logger().info(
            "Task Manager Started"
        )

        # ------------------------------------------------------------
        # Mission Variables
        # ------------------------------------------------------------

        self.state = TaskState.START

        self.current_color = None

        self.navigation_done = False

        self.manipulation_done = False

        self.color_detected = False

        # Completed Objects
        self.completed_colors = set()

        # ------------------------------------------------------------
        # Inspection Sequence
        # ------------------------------------------------------------

        self.inspection_sequence = [

            INSPECTION_CENTER

        ]

        self.inspection_index = 0

        # ------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------

        self.goal_pub = self.create_publisher(

            PoseStamped,

            "/goal_pose",

            10

        )

        self.manipulation_pub = self.create_publisher(

            String,

            "/manipulation_command",

            10

        )

        self.status_pub = self.create_publisher(

            String,

            "/task_status",

            10

        )

        # ------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------

        self.color_sub = self.create_subscription(

            String,

            "/detected_color",

            self.color_callback,

            10

        )

        self.navigation_sub = self.create_subscription(

            Bool,

            "/navigation_done",

            self.navigation_callback,

            10

        )

        self.manipulation_sub = self.create_subscription(

            Bool,

            "/manipulation_done",

            self.manipulation_callback,

            10

        )

        # ------------------------------------------------------------
        # Timer
        # ------------------------------------------------------------

        self.timer = self.create_timer(

            1.0 / CONTROL_RATE,

            self.control_loop

        )

        self.get_logger().info(

            "Mission Ready"

        )

    # ------------------------------------------------------------
    # Color Callback
    # ------------------------------------------------------------

    def color_callback(self, msg):

        color = msg.data.lower()

        if color == "none":

            return

        if self.color_detected:

            return

        if color in self.completed_colors:

            return

        self.current_color = color

        self.color_detected = True

        self.get_logger().info(

            f"Detected Color : {self.current_color}"

        )

    # ------------------------------------------------------------
    # Navigation Callback
    # ------------------------------------------------------------

    def navigation_callback(self, msg):

        self.navigation_done = msg.data

        if self.navigation_done:

            self.get_logger().info(

                "Navigation Complete"

            )

    # ------------------------------------------------------------
    # Manipulation Callback
    # ------------------------------------------------------------

    def manipulation_callback(self, msg):

        self.manipulation_done = msg.data

        if self.manipulation_done:

            self.get_logger().info(

                "Manipulation Complete"

            )

    # ------------------------------------------------------------
    # Publish Status
    # ------------------------------------------------------------

    def publish_status(self, text):

        msg = String()

        msg.data = text

        self.status_pub.publish(msg)
    
    # ------------------------------------------------------------
    # Publish Navigation Goal
    # ------------------------------------------------------------

    def publish_navigation_goal(self, location, yaw=0.0):

        msg = PoseStamped()

        msg.header.frame_id = "odom"

        msg.pose.position.x = location.x

        msg.pose.position.y = location.y

        msg.pose.position.z = 0.0

        msg.pose.orientation.x = 0.0

        msg.pose.orientation.y = 0.0

        msg.pose.orientation.z = math.sin(
            yaw / 2.0
        )

        msg.pose.orientation.w = math.cos(
            yaw / 2.0
        )

        self.goal_pub.publish(msg)

        self.get_logger().info(

            f"Navigation Goal -> "
            f"({location.x:.3f}, "
            f"{location.y:.3f}, "
            f"{yaw:.3f})"

        )
    
    # ------------------------------------------------------------
    # Publish Manipulation Command
    # ------------------------------------------------------------

    def publish_manipulation_command(self, command):

        msg = String()

        msg.data = command

        self.manipulation_pub.publish(msg)

    # ------------------------------------------------------------
    # Go To Inspection Point
    # ------------------------------------------------------------

    def go_to_inspection(self):

        location = self.inspection_sequence[
            self.inspection_index
        ]

        self.publish_navigation_goal(
            location,
            0.0
        )

        self.current_color = None

        self.color_detected = False

        self.navigation_done = False

        self.state = TaskState.WAIT_FOR_NAVIGATION

        self.get_logger().info(

            f"Going To Inspection Point {self.inspection_index + 1}"

        )
    
    # ------------------------------------------------------------
    # Go To Pickup
    # ------------------------------------------------------------

    def go_to_pickup(self):

        location = PICKUP_LOCATIONS[
            self.current_color
        ]

        self.publish_navigation_goal(
            location,
            0.0
        )

        self.navigation_done = False

        self.state = TaskState.WAIT_FOR_PICKUP_NAVIGATION

        self.get_logger().info(

            f"Going To {self.current_color.upper()} Pickup"

        )

    # ------------------------------------------------------------
    # Go To Drop
    # ------------------------------------------------------------

    def go_to_drop(self):

        location = DROP_LOCATIONS[
            self.current_color
        ]

        self.publish_navigation_goal(
            location,
            math.pi
        )

        self.navigation_done = False

        self.state = TaskState.WAIT_FOR_DROP_NAVIGATION

        self.get_logger().info(

            f"Going To {self.current_color.upper()} Bin"

        )

    # ------------------------------------------------------------
    # Return To Inspection
    # ------------------------------------------------------------

    def return_to_inspection(self):

        self.publish_navigation_goal(

            INSPECTION_CENTER,

            0.0

        )

        self.navigation_done = False

        self.state = TaskState.WAIT_FOR_RETURN

        self.get_logger().info(

            "Returning To Inspection Center"

        )
    
    # ------------------------------------------------------------
    # Pickup Object
    # ------------------------------------------------------------

    def pickup_object(self):

        command = f"{self.current_color.upper()}_PICKUP"

        self.publish_manipulation_command(
            command
        )

        self.manipulation_done = False

        self.state = TaskState.WAIT_FOR_PICKUP
    
    # ------------------------------------------------------------
    # Place Object
    # ------------------------------------------------------------

    def place_object(self):

        command = f"{self.current_color.upper()}_PLACE"

        self.publish_manipulation_command(
            command
        )

        self.manipulation_done = False

        self.state = TaskState.WAIT_FOR_PLACE

    # ------------------------------------------------------------
    # Next Inspection
    # ------------------------------------------------------------

    def next_inspection(self):

        self.current_color = None

        self.color_detected = False

        if len(self.completed_colors) >= 3:

            self.state = TaskState.MISSION_COMPLETE

        else:

            self.state = TaskState.GO_TO_INSPECTION

        # ------------------------------------------------------------
    # Control Loop
    # ------------------------------------------------------------

    def control_loop(self):

        self.publish_status(

            self.state.name

        )

        # ------------------------------------------------------------
        # START
        # ------------------------------------------------------------

        if self.state == TaskState.START:

            self.state = TaskState.GO_TO_INSPECTION

        # ------------------------------------------------------------
        # GO TO INSPECTION
        # ------------------------------------------------------------

        elif self.state == TaskState.GO_TO_INSPECTION:

            self.go_to_inspection()

        # ------------------------------------------------------------
        # WAIT FOR INSPECTION NAVIGATION
        # ------------------------------------------------------------

        elif self.state == TaskState.WAIT_FOR_NAVIGATION:

            if self.navigation_done:

                self.navigation_done = False

                self.current_color = None

                self.color_detected = False

                self.state = TaskState.WAIT_FOR_COLOR

        # ------------------------------------------------------------
        # WAIT FOR COLOR
        # ------------------------------------------------------------

        elif self.state == TaskState.WAIT_FOR_COLOR:

            if self.color_detected:

                self.color_detected = False

                self.state = TaskState.GO_TO_PICKUP

            
            self.get_logger().info(
                "Waiting for color detection..."
            )

        # ------------------------------------------------------------
        # GO TO PICKUP
        # ------------------------------------------------------------

        elif self.state == TaskState.GO_TO_PICKUP:

            self.go_to_pickup()

        # ------------------------------------------------------------
        # WAIT FOR PICKUP NAVIGATION
        # ------------------------------------------------------------

        elif self.state == TaskState.WAIT_FOR_PICKUP_NAVIGATION:

            if self.navigation_done:

                self.navigation_done = False

                self.state = TaskState.PICK_OBJECT

        # ------------------------------------------------------------
        # PICK OBJECT
        # ------------------------------------------------------------

        elif self.state == TaskState.PICK_OBJECT:

            self.pickup_object()

        # ------------------------------------------------------------
        # WAIT FOR PICKUP
        # ------------------------------------------------------------

        elif self.state == TaskState.WAIT_FOR_PICKUP:

            if self.manipulation_done:

                self.manipulation_done = False

                self.state = TaskState.GO_TO_DROP

        # ------------------------------------------------------------
        # GO TO DROP
        # ------------------------------------------------------------

        elif self.state == TaskState.GO_TO_DROP:

            self.go_to_drop()

        # ------------------------------------------------------------
        # WAIT FOR DROP NAVIGATION
        # ------------------------------------------------------------

        elif self.state == TaskState.WAIT_FOR_DROP_NAVIGATION:

            if self.navigation_done:

                self.navigation_done = False

                self.state = TaskState.PLACE_OBJECT

        # ------------------------------------------------------------
        # PLACE OBJECT
        # ------------------------------------------------------------

        elif self.state == TaskState.PLACE_OBJECT:

            self.place_object()

        # ------------------------------------------------------------
        # WAIT FOR PLACE
        # ------------------------------------------------------------

        elif self.state == TaskState.WAIT_FOR_PLACE:

            if self.manipulation_done:

                self.manipulation_done = False

                self.completed_colors.add(
                    self.current_color
                )

                self.state = TaskState.RETURN_TO_INSPECTION

        # ------------------------------------------------------------
        # RETURN TO INSPECTION CENTER
        # ------------------------------------------------------------

        elif self.state == TaskState.RETURN_TO_INSPECTION:

            self.return_to_inspection()

        # ------------------------------------------------------------
        # WAIT FOR RETURN
        # ------------------------------------------------------------

        elif self.state == TaskState.WAIT_FOR_RETURN:

            if self.navigation_done:

                self.navigation_done = False

                self.state = TaskState.NEXT_INSPECTION

        # ------------------------------------------------------------
        # NEXT INSPECTION
        # ------------------------------------------------------------

        elif self.state == TaskState.NEXT_INSPECTION:

            self.next_inspection()

        # ------------------------------------------------------------
        # MISSION COMPLETE
        # ------------------------------------------------------------

        elif self.state == TaskState.MISSION_COMPLETE:

            self.get_logger().info(

                "Mission Complete"

            )

            self.publish_status(

                "MISSION_COMPLETE"

            )

            self.timer.cancel()

# ------------------------------------------------------------
# Main
# ------------------------------------------------------------

def main(args=None):

    rclpy.init(args=args)

    node = TaskManagerNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":

    main()