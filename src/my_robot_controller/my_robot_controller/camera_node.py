#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge
from .config import *

# ------------------------------------------------------------
# Camera Node
# ------------------------------------------------------------

class CameraNode(Node):

    def __init__(self):

        super().__init__("camera_node")

        self.get_logger().info(
            "Camera Node Started"
        )

        self.bridge = CvBridge()

        self.last_detected_color = "none"

        self.color_pub = self.create_publisher(
            String,
            "/detected_color",
            10
        )

        self.status_pub = self.create_publisher(
            String,
            "/vision_status",
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            10
        )

        self.get_logger().info(

            "Waiting for Camera Images..."

        )

        # ------------------------------------------------------------
    # Image Callback
    # ------------------------------------------------------------

    def image_callback(self, msg):

        self.publish_status("PROCESSING")

        try:

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )

        except Exception as e:

            self.get_logger().error(str(e))

            return

        detected_color = self.detect_color(frame)

        if (
            detected_color != "none"
        ):

            self.last_detected_color = detected_color

            self.publish_color(detected_color)

        # ------------------------------------------------------------
    # Detect Color
    # ------------------------------------------------------------

    def detect_color(self, frame):

        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV
        )

        red_mask1 = cv2.inRange(
            hsv,
            LOWER_RED1,
            UPPER_RED1
        )

        red_mask2 = cv2.inRange(
            hsv,
            LOWER_RED2,
            UPPER_RED2
        )

        red_mask = red_mask1 + red_mask2

        green_mask = cv2.inRange(
            hsv,
            LOWER_GREEN,
            UPPER_GREEN
        )

        blue_mask = cv2.inRange(
            hsv,
            LOWER_BLUE,
            UPPER_BLUE
        )

        # ------------------------------------------------------------
        # Morphological Filtering
        # ------------------------------------------------------------

        kernel = np.ones((5, 5), np.uint8)

        red_mask = cv2.morphologyEx(
            red_mask,
            cv2.MORPH_OPEN,
            kernel
        )

        red_mask = cv2.morphologyEx(
            red_mask,
            cv2.MORPH_CLOSE,
            kernel
        )


        green_mask = cv2.morphologyEx(
            green_mask,
            cv2.MORPH_OPEN,
            kernel
        )

        green_mask = cv2.morphologyEx(
            green_mask,
            cv2.MORPH_CLOSE,
            kernel
        )


        blue_mask = cv2.morphologyEx(
            blue_mask,
            cv2.MORPH_OPEN,
            kernel
        )

        blue_mask = cv2.morphologyEx(
            blue_mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        red_area, red_contour = self.find_largest_area(red_mask)

        green_area, green_contour = self.find_largest_area(green_mask)

        blue_area, blue_contour = self.find_largest_area(blue_mask)

        detected_color = "none"

        largest_contour = None

        largest_area = 0


        if (

            red_area < MIN_CONTOUR_AREA

            and

            green_area < MIN_CONTOUR_AREA

            and

            blue_area < MIN_CONTOUR_AREA

        ):

            self.publish_status("NO_OBJECT")


        elif red_area > green_area and red_area > blue_area:

            detected_color = "red"

            largest_contour = red_contour

            largest_area = red_area


        elif green_area > blue_area:

            detected_color = "green"

            largest_contour = green_contour

            largest_area = green_area


        else:

            detected_color = "blue"

            largest_contour = blue_contour

            largest_area = blue_area
        
        if largest_contour is not None:

            cv2.drawContours(

                frame,

                [largest_contour],

                -1,

                (0,255,0),

                2

            )

            x,y,w,h = cv2.boundingRect(

                largest_contour

            )

            cv2.rectangle(

                frame,

                (x,y),

                (x+w,y+h),

                (255,0,0),

                2

            )

            cx = x + w//2

            cy = y + h//2

            cv2.circle(

                frame,

                (cx,cy),

                5,

                (0,0,255),

                -1

            )

            cv2.putText(

                frame,

                detected_color.upper(),

                (x,y-10),

                cv2.FONT_HERSHEY_SIMPLEX,

                0.7,

                (0,255,0),

                2

            )

            cv2.putText(

                frame,

                f"Area : {int(largest_area)}",

                (x,y+h+20),

                cv2.FONT_HERSHEY_SIMPLEX,

                0.6,

                (255,255,255),

                2

            )

            cv2.imshow(

            "Robot Camera",

            frame

            )

        cv2.waitKey(1)

        return detected_color
        
    # ------------------------------------------------------------
    # Find Largest Contour
    # ------------------------------------------------------------

    def find_largest_area(self, mask):

        contours, _ = cv2.findContours(

            mask,

            cv2.RETR_EXTERNAL,

            cv2.CHAIN_APPROX_SIMPLE

        )

        largest_area = 0

        largest_contour = None


        for contour in contours:

            area = cv2.contourArea(contour)

            if area > largest_area:

                largest_area = area

                largest_contour = contour


        return largest_area, largest_contour

    # ------------------------------------------------------------
    # Publish Detected Color
    # ------------------------------------------------------------

    def publish_color(self, color):

        msg = String()

        msg.data = color

        self.color_pub.publish(msg)

        self.get_logger().info(

            f"Detected Color : {color.upper()}"

        )
    
    # ------------------------------------------------------------
    # Publish Vision Status
    # ------------------------------------------------------------

    def publish_status(self, status):

        msg = String()

        msg.data = status

        self.status_pub.publish(msg)

    # ------------------------------------------------------------
    # Destroy Node
    # ------------------------------------------------------------

    def destroy_node(self):

        cv2.destroyAllWindows()

        super().destroy_node()

# ------------------------------------------------------------
# Main
# ------------------------------------------------------------

def main(args=None):

    rclpy.init(args=args)

    node = CameraNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":

    main()