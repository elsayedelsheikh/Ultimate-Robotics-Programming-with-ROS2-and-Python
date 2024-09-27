import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
from std_msgs.msg import String


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        
        # Create subscriber to video stream
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # The topic where the video stream is published
            self.image_callback,
            10)

        # Create publisher for the detection results
        self.detections_publisher = self.create_publisher(String, '/yolo/detections', 10)
        
        # Initialize CvBridge to convert between ROS Image messages and OpenCV images
        self.bridge = CvBridge()

        # Load YOLOv8 model (pre-trained on COCO dataset)
        self.model = YOLO('yolov8n.pt')  # You can replace with custom weights if needed

        self.get_logger().info('YOLO Node has been started.')

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Use YOLO to make predictions
        results = self.model(frame)

        # Draw bounding boxes and labels on the frame
        self.display_results(frame, results)

        # Publish the detected object classes as a string
        print("results: ", results)
        detections = self.format_detections(results)
        detection_msg = String()
        detection_msg.data = detections
        self.detections_publisher.publish(detection_msg)

    def display_results(self, frame, results):
        # Draw boxes and labels
        
        for result in results:
            boxes = result.boxes
            for box in boxes:
                # Extract bounding box coordinates
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                # Extract the class ID and confidence score
                class_id = int(box.cls[0])
                confidence = box.conf[0]

                # Get label name for the class
                label = self.model.names[class_id]

                # Draw bounding box and label on the image
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f'{label} {confidence:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Show the frame (for debugging)
        cv2.imshow('YOLOv8 Detections', frame)
        cv2.waitKey(1)

    def format_detections(self, results):
        # Format the detection results as a string for publishing
        detections = []
        for result in results:
            boxes = result.boxes
            for box in boxes:
                class_id = int(box.cls[0])
                label = self.model.names[class_id]
                detections.append(label)
        
        return ', '.join(detections) if detections else 'No objects detected'


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down YOLO node")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
