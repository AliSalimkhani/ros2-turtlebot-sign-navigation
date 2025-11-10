# ROS 2 TurtleBot3 Sign Navigation

A simple sign navigation system for the TurtleBot3 Waffle Pi. It can detect 3 specific traffic signs (left turn, right turn, stop) using classic computer vision approaches. The project is developed with ROS 2 and simulated in Gazebo.

## How `sign_detector` Works

### Left / Right Turn Detection

The algorithm for determining the direction of the arrow follows these steps:

1.  **Color Segmentation:** It first isolates the blue background of the traffic sign using a defined HSV color range.
2.  **Contour Detection:** It finds the main contour of the blue sign to establish its boundaries and center point.
3.  **Arrow Isolation:** Within the blue sign's boundary, it isolates the white arrow shape.
4.  **Keypoint Analysis:** It finds the leftmost and rightmost points of the white arrow.
5.  **Direction Calculation:** It calculates the horizontal distance of these two points from the sign's vertical centerline.
    *   If the **leftmost point** is farther from the center than the rightmost point, the arrow is pointing **Left**.
    *   Otherwise, it is classified as pointing **Right**.

### Stop Sign Detection

The detection for the stop sign is simpler:

1.  **Color Segmentation:** It isolates the red color of the sign using two separate HSV ranges to cover the full red spectrum.
2.  **Contour Validation:** If a red contour with a sufficiently large area is found, it is classified as a "Stop" sign.

## System Flow

Once a sign is detected, the `sign_detector` node publishes a command (`'LEFT'`, `'RIGHT'`, or `'STOP'`) to the `/command` topic. The `controller_node` subscribes to this topic and executes a corresponding maneuver based on the received command.

## Detection Visualization

![Left Turn Detection](https://github.com/user-attachments/assets/1e9f955d-34ab-4827-b3ca-c5f5639e2520)
![Right Turn Detection](https://github.com/user-attachments/assets/cb22ff76-9612-4f0f-b7a1-97cace38a2c1)
![Stop Sign Detection](https://github.com/user-attachments/assets/055b3bf5-e55e-45e5-b52e-4efdb3a652ee)
