#!/usr/bin/env python3

import math
from enum import Enum, auto
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Bool

from tf_transformations import euler_from_quaternion
from my_robot_controller.config import *

# ------------------------------------------------------------
# Navigation State Machine
# ------------------------------------------------------------

class NavigationState(Enum):

    IDLE = auto()

    COMPUTE_HEADING = auto()

    ROTATE_TO_GOAL = auto()

    MOVE_TO_GOAL = auto()

    FINAL_ALIGNMENT = auto()

    GOAL_REACHED = auto()


# ------------------------------------------------------------
# Dataclasses
# ------------------------------------------------------------

@dataclass
class RobotPose:

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class NavigationGoal:

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


# ------------------------------------------------------------
# Navigation Node
# ------------------------------------------------------------

class NavigationNode(Node):

    def __init__(self):

        super().__init__("navigation_node")

        self.get_logger().info("Navigation Node Started")


        # ------------------------------------------------------------
        # PI Controller Variables
        # ------------------------------------------------------------

        self.heading_integral = 0.0

        self.dt = 1.0 / CONTROL_RATE

        # -----------------------------------------
        # Robot Pose
        # -----------------------------------------

        self.robot_pose = RobotPose()

        # -----------------------------------------
        # Final Navigation Goal
        # -----------------------------------------

        self.final_goal = NavigationGoal()


        # -----------------------------------------
        # Active Navigation Goal
        # -----------------------------------------

        self.active_goal = NavigationGoal()


        # -----------------------------------------
        # Waypoint List
        # -----------------------------------------

        self.waypoints = []


        # -----------------------------------------
        # Current Waypoint Index
        # -----------------------------------------

        self.waypoint_index = 0


        # -----------------------------------------
        # Total Waypoints
        # -----------------------------------------

        self.total_waypoints = 0
  
        # -----------------------------------------
        # Desired Heading
        # -----------------------------------------

        self.goal_heading = 0.0

        # -----------------------------------------
        # Navigation State
        # -----------------------------------------

        self.state = NavigationState.IDLE


        # -----------------------------------------
        # Goal Availability
        # -----------------------------------------

        self.goal_received = False


        # -----------------------------------------
        # Publishers
        # -----------------------------------------

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )


        self.status_pub = self.create_publisher(
            String,
            "/robot_navigation_status",
            10
        )

        self.goal_reached_pub = self.create_publisher(
            Bool,
            "/goal_reached",
            10
        )

        self.navigation_done_pub = self.create_publisher(
            Bool,
            "/navigation_done",
            10
        ) 

              
    
        # -----------------------------------------
        # Subscribers
        # -----------------------------------------

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )


        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_pose_callback,
            10
        )


        # -----------------------------------------
        # Timer
        # -----------------------------------------

        self.timer = self.create_timer(
            1.0 / CONTROL_RATE,
            self.control_loop
        )


        self.get_logger().info("Waiting for Navigation Goal...")

    # ------------------------------------------------------------
    # ODOM CALLBACK
    # ------------------------------------------------------------

    def odom_callback(self, msg: Odometry):

        self.robot_pose.x = msg.pose.pose.position.x
        self.robot_pose.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        quaternion = [
            q.x,
            q.y,
            q.z,
            q.w
        ]

        (_, _, yaw) = euler_from_quaternion(quaternion)

        self.robot_pose.yaw = yaw
    
    # ------------------------------------------------------------
    # GOAL CALLBACK
    # ------------------------------------------------------------

    def goal_pose_callback(self, msg: PoseStamped):

        self.final_goal.x = msg.pose.position.x
        self.final_goal.y = msg.pose.position.y

        q = msg.pose.orientation

        quaternion = [
            q.x,
            q.y,
            q.z,
            q.w
        ]

        (_, _, self.final_goal.yaw) = euler_from_quaternion(
            quaternion
        )

        self.get_logger().info(

            f"Final Goal : "
            f"({self.final_goal.x:.3f}, "
            f"{self.final_goal.y:.3f})"

        )

        self.generate_waypoints()

        self.load_waypoint()
            
    # ------------------------------------------------------------
    # NORMALIZE ANGLE
    # ------------------------------------------------------------

    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle


    # ------------------------------------------------------------
    # DISTANCE TO GOAL
    # ------------------------------------------------------------

    def distance_to_goal(self):

        dx = self.active_goal.x - self.robot_pose.x
        dy = self.active_goal.y - self.robot_pose.y

        return math.sqrt(dx**2 + dy**2)


    # ------------------------------------------------------------
    # HEADING TO GOAL
    # ------------------------------------------------------------

    def compute_goal_heading(self):

        dx = self.active_goal.x - self.robot_pose.x
        dy = self.active_goal.y - self.robot_pose.y

        self.goal_heading = math.atan2(dy, dx)


    # ------------------------------------------------------------
    # PUBLISH VELOCITY
    # ------------------------------------------------------------

    def publish_velocity(
            self,
            linear=0.0,
            angular=0.0):

        msg = Twist()

        msg.linear.x = linear
        msg.angular.z = angular

        self.cmd_vel_pub.publish(msg)


    # ------------------------------------------------------------
    # STOP ROBOT
    # ------------------------------------------------------------

    def stop_robot(self):

        self.publish_velocity(
            linear=0.0,
            angular=0.0
        )


    # ------------------------------------------------------------
    # PUBLISH STATUS
    # ------------------------------------------------------------

    def publish_status(self, text):

        msg = String()

        msg.data = text

        self.status_pub.publish(msg)

    # ------------------------------------------------------------
    # Publish Goal Reached
    # ------------------------------------------------------------

    def publish_goal_reached(self, reached):

        msg = Bool()

        msg.data = reached

        self.goal_reached_pub.publish(msg)
    
    # ------------------------------------------------------------
    # POSITION REACHED
    # ------------------------------------------------------------

    def position_reached(self):

        return self.distance_to_goal() < POSITION_TOLERANCE
    
    def publish_navigation_done(self, done):

        msg = Bool()

        msg.data = done

        self.navigation_done_pub.publish(msg)


    # ------------------------------------------------------------
    # ANGLE REACHED
    # ------------------------------------------------------------

    def angle_reached(self, desired_angle):

        error = self.normalize_angle(
            desired_angle -
            self.robot_pose.yaw
        )

        return abs(error) < ANGLE_TOLERANCE
    
    # ------------------------------------------------------------
    # GENERATE WAYPOINTS
    # ------------------------------------------------------------

    # ------------------------------------------------------------
    # GENERATE WAYPOINTS
    # ------------------------------------------------------------

    def generate_waypoints(self):

        self.waypoints.clear()

        self.waypoint_index = 0

        self.total_waypoints = 0

        current_x = self.robot_pose.x
        current_y = self.robot_pose.y

        target_x = self.final_goal.x
        target_y = self.final_goal.y
        target_yaw = self.final_goal.yaw

        same_x = abs(current_x - target_x) < POSITION_TOLERANCE
        same_y = abs(current_y - target_y) < POSITION_TOLERANCE

        # --------------------------------------------------------
        # Already at Goal
        # --------------------------------------------------------

        if same_x and same_y:

            self.add_waypoint(
                target_x,
                target_y,
                target_yaw
            )

        # --------------------------------------------------------
        # Only X needs to change
        # --------------------------------------------------------

        elif same_y:

            self.add_waypoint(
                target_x,
                target_y,
                target_yaw
            )

        # --------------------------------------------------------
        # Only Y needs to change
        # --------------------------------------------------------

        elif same_x:

            self.add_waypoint(
                target_x,
                target_y,
                target_yaw
            )

        # --------------------------------------------------------
        # Both X and Y need to change
        # --------------------------------------------------------

        else:

            if MOVE_Y_FIRST:

                #
                # Move along Y first
                #

                if target_y > current_y:
                    intermediate_yaw = math.pi / 2
                else:
                    intermediate_yaw = -math.pi / 2

                self.add_waypoint(
                    current_x,
                    target_y,
                    intermediate_yaw
                )

                self.add_waypoint(
                    target_x,
                    target_y,
                    target_yaw
                )

            else:

                #
                # Move along X first
                #

                if target_x > current_x:
                    intermediate_yaw = 0.0
                else:
                    intermediate_yaw = math.pi

                self.add_waypoint(
                    target_x,
                    current_y,
                    intermediate_yaw
                )

                self.add_waypoint(
                    target_x,
                    target_y,
                    target_yaw
                )

        self.total_waypoints = len(self.waypoints)

        self.get_logger().info(
            f"Generated {self.total_waypoints} waypoint(s)"
        )

    # ------------------------------------------------------------
    # LOAD WAYPOINT
    # ------------------------------------------------------------

    def load_waypoint(self):

        if self.waypoint_index >= self.total_waypoints:

            return

        waypoint = self.waypoints[
            self.waypoint_index
        ]

        self.set_active_goal(
            waypoint
        )

        self.get_logger().info(

            f"Waypoint "
            f"{self.waypoint_index + 1}"
            f"/"
            f"{self.total_waypoints}"

        )

    # ------------------------------------------------------------
    # ADVANCE WAYPOINT
    # ------------------------------------------------------------

    def advance_waypoint(self):

        self.waypoint_index += 1

        return self.waypoint_index < self.total_waypoints

    # ------------------------------------------------------------
    # ADD WAYPOINT
    # ------------------------------------------------------------

    def add_waypoint(self, x, y, yaw):

        self.waypoints.append(

            NavigationGoal(

                x=x,
                y=y,
                yaw=yaw

            )

        )
    
    # ------------------------------------------------------------
    # SET ACTIVE GOAL
    # ------------------------------------------------------------

    def set_active_goal(self, goal: NavigationGoal):

        self.active_goal = NavigationGoal(
            x=goal.x,
            y=goal.y,
            yaw=goal.yaw
        )

        self.goal_received = True

        # Reset PI Controller
        self.heading_integral = 0.0

        self.state = NavigationState.COMPUTE_HEADING

        self.publish_goal_reached(False)

        self.publish_navigation_done(False)

        self.get_logger().info(

            f"Active Goal -> "
            f"({goal.x:.3f}, {goal.y:.3f})"

        )
    
    # ------------------------------------------------------------
    # ROTATE TO GOAL
    # ------------------------------------------------------------

    def rotate_to_goal(self):

        error = self.normalize_angle(
            self.goal_heading -
            self.robot_pose.yaw
        )

        angular_speed = KP_ANGULAR * error

        angular_speed = max(
            -MAX_ANGULAR_SPEED,
            min(MAX_ANGULAR_SPEED, angular_speed)
        )

        self.publish_velocity(
            linear=0.0,
            angular=angular_speed
        )

        if abs(error) < ANGLE_TOLERANCE:

            self.stop_robot()

            self.state = NavigationState.MOVE_TO_GOAL

            self.get_logger().info("Heading Correct")


    # ------------------------------------------------------------
    # MOVE TO GOAL
    # ------------------------------------------------------------

    def move_to_goal(self):

        distance = self.distance_to_goal()

        self.compute_goal_heading()

        heading_error = self.normalize_angle(
            self.goal_heading -
            self.robot_pose.yaw
        )

        linear_speed = KP_LINEAR * distance
        angular_speed = KP_ANGULAR * heading_error

        linear_speed = min(
            MAX_LINEAR_SPEED,
            linear_speed
        )

        angular_speed = max(
            -MAX_ANGULAR_SPEED,
            min(MAX_ANGULAR_SPEED, angular_speed)
        )

        self.publish_velocity(
            linear=linear_speed,
            angular=angular_speed
        )

        if self.position_reached():

            self.stop_robot()

            self.state = NavigationState.FINAL_ALIGNMENT

            self.get_logger().info("Position Reached")


    # ------------------------------------------------------------
    # FINAL ALIGNMENT
    # ------------------------------------------------------------

    def final_alignment(self):

        error = self.normalize_angle(
            self.active_goal.yaw -
            self.robot_pose.yaw
        )

        # ------------------------------------------------------------
        # PI Controller
        # ------------------------------------------------------------

        self.heading_integral += error * self.dt

        # Anti-windup
        self.heading_integral = max(
            -0.5,
            min(0.5, self.heading_integral)
        )

        angular_speed = (
            KP_ANGULAR * error +
            KI_ANGULAR * self.heading_integral
        )

        # ------------------------------------------------------------
        # Speed Limits
        # ------------------------------------------------------------

        angular_speed = max(
            -MAX_ANGULAR_SPEED,
            min(MAX_ANGULAR_SPEED, angular_speed)
        )

        # Maintain minimum angular speed until within tolerance
        if (
            abs(error) > ANGLE_TOLERANCE and
            abs(angular_speed) < MIN_ANGULAR_SPEED
        ):

            angular_speed = (
                MIN_ANGULAR_SPEED
                if angular_speed > 0.0
                else -MIN_ANGULAR_SPEED
            )

        self.publish_velocity(
            linear=0.0,
            angular=angular_speed
        )

        if abs(error) < ANGLE_TOLERANCE:

            self.stop_robot()

            self.state = NavigationState.GOAL_REACHED

            self.get_logger().info("Final Heading Reached")
    
    # ------------------------------------------------------------
    # CONTROL LOOP
    # ------------------------------------------------------------

    def control_loop(self):

        if not self.goal_received:
            return


        if self.state == NavigationState.IDLE:

            self.publish_status("IDLE")


        elif self.state == NavigationState.COMPUTE_HEADING:

            self.publish_status("COMPUTE_HEADING")

            self.compute_goal_heading()

            self.state = NavigationState.ROTATE_TO_GOAL


        elif self.state == NavigationState.ROTATE_TO_GOAL:

            self.publish_status("ROTATE_TO_GOAL")

            self.rotate_to_goal()


        elif self.state == NavigationState.MOVE_TO_GOAL:

            self.publish_status("MOVE_TO_GOAL")

            self.move_to_goal()


        elif self.state == NavigationState.FINAL_ALIGNMENT:

            self.publish_status("FINAL_ALIGNMENT")

            self.final_alignment()


        elif self.state == NavigationState.GOAL_REACHED:

            self.publish_status("GOAL_REACHED")

            self.stop_robot()

            self.heading_integral = 0.0

            #
            # More waypoints?
            #

            if self.advance_waypoint():

                self.load_waypoint()

                return

            #
            # Final waypoint completed
            #

            self.publish_navigation_done(True)

            self.publish_goal_reached(True)

            self.goal_received = False

            self.state = NavigationState.IDLE

            self.get_logger().info("Navigation Complete")

def main(args=None):

    rclpy.init(args=args)

    node = NavigationNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.stop_robot()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":

    main()