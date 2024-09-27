import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image   
from cv_bridge import CvBridge
import numpy as np
import cv2

class DepthDistanceCalculator(Node):
    def __init__(self):
        super().__init__('depth_distance_calculator')

        # Create subscriptions to color image, depth image, and camera calibration
        self.subscription_image = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.subscription_depth = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10)
        self.bridge = CvBridge()

        self.color_image = None
        self.depth_image = None
        self.dist_coeffs = None
        self.distance = None

        self.publisher_ = self.create_publisher(Image, "/annotated_image", 10)
        
    def image_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if self.color_image is not None and self.distance is not None:
            self.display_annotated_image()

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")

        if self.depth_image is not None:
            self.calculate_distance()

    def calculate_distance(self):
        if self.depth_image is None:
            self.get_logger().warning('Depth image not available.')
            return

        # Get the image height and width
        height, width = self.depth_image.shape

        # Calculate the central point of the depth image
        center_x = width // 2
        center_y = height // 2

        # Extract the depth value at the central point
        self.distance = self.depth_image[center_y, center_x]*0.001

        # Check if the distance value is valid
        if not np.isfinite(self.distance):
            self.get_logger().warning('Invalid depth value at center point.')
            self.distance = None


    def display_annotated_image(self):
        # Make a copy of the color image to annotate
        annotated_image = self.color_image.copy()

        # Get the image dimensions
        height, width, _ = annotated_image.shape

        # Calculate the central point
        center_x = width // 2
        center_y = height // 2

        # Draw a red dot at the central point
        cv2.circle(annotated_image, (center_x, center_y), 5, (0, 0, 255), -1)

        # Add the distance text near the central point
        if self.distance is not None and np.isfinite(self.distance):
            distance_text = f"{self.distance:.2f} m"
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.6
            color = (255, 255, 255)  # White color
            thickness = 2
            text_position = (center_x - 50, center_y - 10)
            cv2.putText(annotated_image, distance_text, text_position, font, font_scale, color, thickness)

        # Display the annotated image in a window        
        image_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
        self.publisher_.publish(image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DepthDistanceCalculator()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
