## EXP._ROB._Assignment_1
Group members
- Mohamed sameh mohamed El shab
- Waleed Elfieky


---

# ArUco Explorer Node

## Description

This node is designed to make the robot perform an initial scan of the environment in order to detect
all the ArUco markers around it. During the scan, the robot rotates in place and collects the IDs of
all detected markers.

Once all marker IDs have been identified, the robot processes the markers in ascending order. For
each marker, the robot rotates until the marker appears in the camera image and aligns itself so that
the marker is centered in the image. When the marker is centered, a new image is published on a custom
topic with a circle drawn around the detected marker.

<p align="center">
  <img src="/aruco_explorer_demo.png" width="800">
</p>

The same behaviour is repeated for all detected markers until the task is completed.


---

## Behavior Overview

The node works as a small state machine:

### Scanning
- The robot rotates in place for a fixed amount of time.
- Marker detections are received from the ArUco tracker.
- For each detected marker, the node stores:
  - the marker ID,
  - the marker position `(x, y)` in the `odom` frame,

### Go to Marker
- Markers are processed in ascending ID order.
- For each marker:
  - the rotation direction is computed **once** using the saved position,
  - the robot keeps rotating in that direction until the marker appears.
- When the marker is visible:
  - pixel-based control is used to center it in the image.
- Once centered:
  - a green circle is drawn on the image,
  - the next marker becomes the target.

### Done
- When all markers are processed, the robot stops.

---

## Control Logic


The control is split into two parts: **searching** (marker not visible) and **centering** (marker visible).

- **Searching / rotation direction**
  - After the scan, for the current target marker the node uses the saved marker position to compute a **desired yaw**.
  - The robot then rotates in the **shortest direction** toward that yaw.
  - While the marker is not visible, the robot keeps rotating at a constant angular speed (`search_angular_vel`).

- **Centering in the image**
  - When the target marker becomes visible in the camera image, the node computes the horizontal pixel error:
    - `err_x = marker_center_x - image_center_x`
  - The robot angular velocity is proportional to this error:
    - `w = -pixel_angular_gain * err_x`
  - The angular velocity is limited by `max_angular_vel`.
  - If `|err_x| < center_tolerance_px`, the marker is considered centered:
    - the robot stops,
    - an annotated image with a circle around the marker is published,
    - the next marker ID becomes the target.

---

## Topics

### Subscribed Topics
- `/aruco_detections`  
  Marker poses published by the ArUco tracker.
- `/camera/image`  
  Camera stream used for OpenCV ArUco detection.

### Published Topics
- `/cmd_vel`  
  Angular velocity commands.
- `/aruco_explorer/marker_image`  
  Image with a green circle drawn around the centered marker.

---

## Parameters

| Parameter | Description | Default |
|---------|------------|---------|
| `base_frame` | Robot base frame | `base_link` |
| `odom_frame` | Fixed reference frame | `odom` |
| `scan_angular_vel` | Angular velocity during scanning | `1.0` |
| `scan_duration` | Scanning duration (s) | `40.0` |
| `image_topic` | Camera image topic | `/camera/image` |
| `aruco_dictionary_id` | ArUco dictionary ID | `DICT_ARUCO_ORIGINAL` |
| `pixel_angular_gain` | Gain for centering control | `0.003` |
| `center_tolerance_px` | Pixel tolerance for centering | `10` |
| `search_angular_vel` | Angular velocity while searching | `0.6` |
| `max_angular_vel` | Maximum angular velocity | `0.8` |

---

## Requirements

### Environment and Simulation

- **ROS 2** installed and correctly sourced.
- A working **Gazebo simulation** is required to run the robot and sensors.
  
### Required Packages and Dependencies
 - you can install if you dont have  :
  ```bash
  sudo apt update
  sudo apt install -y \
  ros-<ros_distro>-rclcpp \
  ros-<ros_distro>-geometry-msgs \
  ros-<ros_distro>-sensor-msgs \
  ros-<ros_distro>-image-transport \
  ros-<ros_distro>-cv-bridge \
  ros-<ros_distro>-tf2-ros \
  ros-<ros_distro>-tf2-geometry-msgs
```
---
## How to Run

- First make sure you build correctly.
- start the simulation:
```bash
ros2 launch bme_gazebo_sensors spawn_robot.launch.py
```
- then launch the explorer node and the tracker using the launch file.
```bash
ros2 launch my_aruco_pkg aruco_explorer.launch.xml
```
