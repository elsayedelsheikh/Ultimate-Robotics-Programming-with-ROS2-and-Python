# msg/DetectedObject.msg
string class_name  # The name or label of the detected object (e.g., "person", "car").
float32 confidence  # Confidence score of the detection.
int32 center_x    # X coordinate of the bounding box center.
int32 center_y    # Y coordinate of the bounding box center.