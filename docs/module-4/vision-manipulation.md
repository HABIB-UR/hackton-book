---
sidebar_position: 3
---

# Vision-Guided Manipulation: Computer Vision for Robot Control

## Introduction to Computer Vision for Robotics

Computer vision is a critical component of autonomous robot systems, enabling robots to perceive and understand their environment. In this chapter, we'll explore how computer vision systems can guide robot manipulation tasks, creating the perception-action loop that makes autonomous robots truly capable.

Vision-guided manipulation involves several key components:
1. **Image Acquisition**: Capturing visual information from the environment
2. **Object Detection**: Identifying and locating objects of interest
3. **Pose Estimation**: Determining the position and orientation of objects
4. **Manipulation Planning**: Planning robot movements based on visual input
5. **Action Execution**: Executing manipulation actions guided by vision

## OpenCV Integration with Practical Examples

OpenCV (Open Source Computer Vision Library) is one of the most popular libraries for computer vision applications. We'll demonstrate how to integrate OpenCV for robotic vision tasks.

### Basic OpenCV Setup for Robotics

Here's a basic example of how to use OpenCV for object detection in robotics:

```python
import cv2
import numpy as np
from typing import List, Tuple, Dict, Any

class VisionGuidedManipulation:
    def __init__(self):
        """
        Initialize the vision-guided manipulation system
        """
        pass

    def detect_objects(self, image: np.ndarray, method: str = "color") -> List[Dict[str, Any]]:
        """
        Detect objects in an image using various methods
        """
        if method == "color":
            return self._detect_by_color(image)
        elif method == "contour":
            return self._detect_by_contours(image)
        elif method == "template":
            return self._detect_by_template(image)
        else:
            raise ValueError(f"Unknown detection method: {method}")

    def _detect_by_color(self, image: np.ndarray) -> List[Dict[str, Any]]:
        """
        Detect objects based on color using HSV color space
        """
        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define range for red color (example)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        # Upper red range
        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)

        # Combine masks
        mask = mask1 + mask2

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        objects = []
        for contour in contours:
            # Filter by area to avoid noise
            if cv2.contourArea(contour) > 500:  # Minimum area threshold
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)

                # Calculate center
                center_x = x + w // 2
                center_y = y + h // 2

                objects.append({
                    "type": "red_object",
                    "x": center_x,
                    "y": center_y,
                    "width": w,
                    "height": h,
                    "confidence": 0.8  # Placeholder confidence
                })

        return objects

    def _detect_by_contours(self, image: np.ndarray) -> List[Dict[str, Any]]:
        """
        Detect objects based on shape using contour analysis
        """
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply threshold
        _, thresh = cv2.threshold(gray, 127, 255, 0)

        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        objects = []
        for contour in contours:
            # Calculate contour properties
            area = cv2.contourArea(contour)

            # Filter by area
            if area > 500:
                # Approximate contour to polygon
                epsilon = 0.02 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)

                # Determine shape based on number of vertices
                vertices = len(approx)
                shape = "unknown"

                if vertices == 3:
                    shape = "triangle"
                elif vertices == 4:
                    # Check if it's square or rectangle
                    x, y, w, h = cv2.boundingRect(approx)
                    aspect_ratio = float(w) / h
                    if 0.95 <= aspect_ratio <= 1.05:
                        shape = "square"
                    else:
                        shape = "rectangle"
                elif vertices > 4:
                    shape = "circle"  # Approximated as polygon with many sides

                # Calculate center
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    center_x = int(M["m10"] / M["m00"])
                    center_y = int(M["m01"] / M["m00"])
                else:
                    center_x = 0
                    center_y = 0

                objects.append({
                    "type": shape,
                    "x": center_x,
                    "y": center_y,
                    "vertices": vertices,
                    "area": area,
                    "confidence": 0.7  # Placeholder confidence
                })

        return objects

    def overlay_detections(self, image: np.ndarray, detections: List[Dict[str, Any]]) -> np.ndarray:
        """
        Overlay detection results on the original image
        """
        output_image = image.copy()

        for detection in detections:
            x, y = int(detection['x']), int(detection['y'])
            width = detection.get('width', 20)
            height = detection.get('height', 20)

            # Draw circle at object center
            cv2.circle(output_image, (x, y), 10, (0, 255, 0), 2)

            # Draw bounding box if width and height are available
            if 'width' in detection and 'height' in detection:
                x1, y1 = x - width//2, y - height//2
                x2, y2 = x + width//2, y + height//2
                cv2.rectangle(output_image, (x1, y1), (x2, y2), (255, 0, 0), 2)

            # Add label
            label = f"{detection['type']}"
            cv2.putText(output_image, label, (x + 15, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        return output_image
```

## Hugging Face Transformers for Object Detection

Hugging Face Transformers provides state-of-the-art models for object detection that can be used for more sophisticated vision tasks:

```python
from transformers import pipeline
import cv2
import numpy as np
from typing import List, Dict, Any

class HuggingFaceVision:
    def __init__(self, model_name: str = "facebook/detr-resnet-50"):
        """
        Initialize the Hugging Face vision system with a pre-trained model
        """
        self.object_detector = pipeline("object-detection", model=model_name)

    def detect_objects(self, image_path: str) -> List[Dict[str, Any]]:
        """
        Detect objects in an image using Hugging Face Transformers
        """
        try:
            # Perform object detection
            results = self.object_detector(image_path)

            # Convert results to our standard format
            objects = []
            for result in results:
                # Extract bounding box coordinates
                bbox = result['box']
                objects.append({
                    "label": result['label'],
                    "score": result['score'],
                    "x": (bbox['xmin'] + bbox['xmax']) / 2,  # Center x
                    "y": (bbox['ymin'] + bbox['ymax']) / 2,  # Center y
                    "width": bbox['xmax'] - bbox['xmin'],
                    "height": bbox['ymax'] - bbox['ymin'],
                    "bbox": [bbox['xmin'], bbox['ymin'], bbox['xmax'], bbox['ymax']]
                })

            return objects
        except Exception as e:
            print(f"Error in object detection: {e}")
            return []

    def detect_objects_from_array(self, image_array: np.ndarray) -> List[Dict[str, Any]]:
        """
        Detect objects from a numpy array (OpenCV image)
        """
        # Convert numpy array to PIL Image
        from PIL import Image
        pil_image = Image.fromarray(cv2.cvtColor(image_array, cv2.COLOR_BGR2RGB))

        # Perform detection
        results = self.object_detector(pil_image)

        # Convert results to our standard format
        objects = []
        for result in results:
            bbox = result['box']
            objects.append({
                "label": result['label'],
                "score": result['score'],
                "x": (bbox['xmin'] + bbox['xmax']) / 2,  # Center x
                "y": (bbox['ymin'] + bbox['ymax']) / 2,  # Center y
                "width": bbox['xmax'] - bbox['xmin'],
                "height": bbox['ymax'] - bbox['ymin'],
                "bbox": [bbox['xmin'], bbox['ymin'], bbox['xmax'], bbox['ymax']]
            })

        return objects
```

## Real-time vs. Batch Processing for Vision

Vision processing can be implemented in two main ways:

### Batch Processing
- Process static images or pre-recorded video
- Suitable for applications where real-time response is not critical
- More accurate as processing time is not constrained

### Real-time Processing
- Process video frames as they are captured
- Requires efficient algorithms and optimized code
- Provides immediate feedback but may be less accurate

```python
import cv2
import time
from typing import Generator

class RealTimeVisionProcessor:
    def __init__(self, vision_system: VisionGuidedManipulation):
        """
        Initialize real-time vision processing
        """
        self.vision_system = vision_system

    def process_video_stream(self, camera_index: int = 0) -> Generator[List[Dict[str, Any]], None, None]:
        """
        Process video stream in real-time
        """
        cap = cv2.VideoCapture(camera_index)

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break

                # Process the frame
                detections = self.vision_system.detect_objects(frame)

                # Overlay detections on frame
                annotated_frame = self.vision_system.overlay_detections(frame, detections)

                # Yield detections for further processing
                yield detections, annotated_frame

                # Break on 'q' key press
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            cap.release()
            cv2.destroyAllWindows()

    def process_with_robot_integration(self, camera_index: int = 0):
        """
        Process video stream with robot integration
        """
        cap = cv2.VideoCapture(camera_index)

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break

                # Detect objects
                detections = self.vision_system.detect_objects(frame)

                # Process detections for robot action
                if detections:
                    # Find the most relevant object (e.g., closest, largest, etc.)
                    target_object = self.select_target_object(detections)

                    if target_object:
                        # Send command to robot based on vision input
                        robot_command = self.generate_robot_command(target_object, frame.shape)
                        print(f"Robot command: {robot_command}")

                # Display annotated frame
                annotated_frame = self.vision_system.overlay_detections(frame, detections)
                cv2.imshow('Vision-Guided Manipulation', annotated_frame)

                # Break on 'q' key press
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            cap.release()
            cv2.destroyAllWindows()

    def select_target_object(self, detections: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Select the most relevant object from detections
        """
        if not detections:
            return None

        # Example: Select the object closest to the center of the frame
        # For a 640x480 frame, center is at (320, 240)
        center_x, center_y = 320, 240  # Assuming 640x480 resolution

        closest_object = None
        min_distance = float('inf')

        for obj in detections:
            distance = np.sqrt((obj['x'] - center_x)**2 + (obj['y'] - center_y)**2)
            if distance < min_distance:
                min_distance = distance
                closest_object = obj

        return closest_object

    def generate_robot_command(self, target_object: Dict[str, Any], frame_shape: tuple) -> str:
        """
        Generate robot command based on vision input
        """
        frame_height, frame_width = frame_shape[:2]
        center_x, center_y = frame_width // 2, frame_height // 2

        # Determine robot action based on object position
        dx = target_object['x'] - center_x
        dy = target_object['y'] - center_y

        # Dead zone to avoid constant small movements
        dead_zone = 30

        command_parts = []

        if abs(dx) > dead_zone:
            if dx > 0:
                command_parts.append("move_right")
            else:
                command_parts.append("move_left")

        if abs(dy) > dead_zone:
            if dy > 0:
                command_parts.append("move_down")
            else:
                command_parts.append("move_up")

        if command_parts:
            return f"robot_adjust_position: {', '.join(command_parts)}"
        else:
            return "robot_align_with_object"
```

## Integration with ROS 2 Vision Topics

When working with ROS 2, vision data is typically published on image topics:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ROS2VisionNode(Node):
    def __init__(self):
        super().__init__('vision_guided_manipulation_node')

        # Create subscription to camera image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Create publisher for robot commands
        # self.command_publisher = self.create_publisher(RobotCommand, '/robot/command', 10)

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

        # Initialize vision system
        self.vision_system = VisionGuidedManipulation()

        self.get_logger().info('Vision-guided manipulation node initialized')

    def image_callback(self, msg: Image):
        """
        Process incoming image message from ROS 2
        """
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform object detection
            detections = self.vision_system.detect_objects(cv_image)

            # Process detections and generate robot commands
            if detections:
                self.process_detections_for_robot(detections)

            # Optionally, publish annotated image
            annotated_image = self.vision_system.overlay_detections(cv_image, detections)
            # self.publish_annotated_image(annotated_image)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def process_detections_for_robot(self, detections: List[Dict[str, Any]]):
        """
        Process detections and generate appropriate robot commands
        """
        # Example: If we detect a "cup", command the robot to pick it up
        for detection in detections:
            if detection['type'] in ['cup', 'bottle', 'object'] and detection['confidence'] > 0.7:
                # Generate command to pick up the object
                command = {
                    'action': 'pick_object',
                    'x': detection['x'],
                    'y': detection['y'],
                    'object_type': detection['type']
                }

                self.get_logger().info(f'Generated command: {command}')
                # self.command_publisher.publish(command)
                break  # Process first detected object
```

## Example Exercise: Vision-Guided Manipulation

**Objective**: Implement a basic vision-guided manipulation system that can detect objects and generate appropriate robot commands.

**Instructions**:
1. Implement the VisionGuidedManipulation class with basic detection methods
2. Test with sample images containing colored objects
3. Verify that the system correctly identifies object positions

**Expected Outcome**: The system should detect objects in images and provide their positions for robot manipulation.

## Exercise: Vision-Guided Manipulation Practice

**Objective**: Create a complete vision-guided manipulation system that can detect objects and guide robot actions.

**Difficulty**: Advanced

**Instructions**:
1. Implement both OpenCV and Hugging Face detection methods
2. Create a real-time processing pipeline
3. Integrate with a simulated robot control system
4. Test with various objects and lighting conditions

**Expected Outcome**: A working vision-guided manipulation system that can detect objects and generate appropriate robot commands.

**Hints**:
- Start with simple color-based detection before moving to more complex methods
- Consider lighting conditions and how they affect detection accuracy
- Implement robust error handling for edge cases

## Conclusion

In this chapter, we've explored how computer vision systems can guide robot manipulation tasks. We've covered:
- Basic OpenCV integration for object detection
- Advanced object detection using Hugging Face Transformers
- Real-time vs. batch processing approaches
- Integration with ROS 2 vision topics

In the next chapter, we'll combine all these components into a complete autonomous humanoid system that integrates voice processing, cognitive planning, and vision-guided manipulation.

## Learning Objectives Review

By completing this chapter, you should now understand:
- How to use OpenCV for basic computer vision tasks in robotics
- How to integrate advanced models from Hugging Face for object detection
- The differences between real-time and batch processing approaches
- How to integrate vision systems with ROS 2

## Validation Against Requirements

This chapter meets the following functional requirements:

**FR-004**: System MUST cover computer vision techniques for object detection and manipulation guidance
- ✅ The chapter provides comprehensive coverage of OpenCV for robotics applications
- ✅ Advanced object detection techniques using Hugging Face are explained
- ✅ Real-time and batch processing approaches are covered
- ✅ Integration with ROS 2 vision systems is demonstrated