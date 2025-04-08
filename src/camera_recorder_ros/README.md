# Camera Recorder ROS Package

ROS 2 package for recording video streams from multiple USB cameras using GStreamer.

## Features

*   Record from multiple cameras simultaneously.
*   Configure camera resolution, framerate, and bitrate.
*   Timestamp video filenames with UTC time.
*   Optional timestamp overlay on video frames (requires GStreamer `textoverlay` element).
*   Control recording via keyboard input or ROS 2 services.
*   Outputs video files in MP4 format (using H.264 encoding by default).

## Nodes

This package provides two nodes:

### 1. `camera_recorder_node` (Keyboard Control)

This node allows you to start and stop recording for all configured cameras using keyboard input.

**Usage:**

1.  **Build the package:**
    ```bash
    colcon build --packages-select camera_recorder_ros --symlink-install
    ```
2.  **Source the workspace:**
    ```bash
    source install/setup.bash
    ```
3.  **Run the node:**
    ```bash
    ros2 run camera_recorder_ros camera_recorder_node
    ```
4.  **Control Recording:**
    *   Press `s` then `Enter` to start recording.
    *   Press `q` then `Enter` to stop recording and quit.

### 2. `camera_recorder_service_node` (Service Control)

This node allows you to start and stop recording for all configured cameras using ROS 2 services. It does not respond to keyboard input.

**Usage:**

1.  **Build the package:** (if not already done)
    ```bash
    colcon build --packages-select camera_recorder_ros --symlink-install
    ```
2.  **Source the workspace:**
    ```bash
    source install/setup.bash
    ```
3.  **Run the node:**
    ```bash
    ros2 run camera_recorder_ros camera_recorder_service_node
    ```
4.  **Control Recording (from another terminal):**
    *   **Start Recording (all cameras):**
        ```bash
        ros2 service call /start_recording std_srvs/srv/Trigger
        ```
    *   **Stop Recording:**
        ```bash
        ros2 service call /stop_recording std_srvs/srv/Trigger
        ```

## Configuration

Camera configurations (device path, name, resolution, etc.) are currently hardcoded within the Python node files (`camera_recorder_node.py` and `camera_recorder_service_node.py`).

Recording parameters can be set via ROS parameters when launching the nodes (e.g., `output_dir`, `timezone`, `encoder`).

**Example (setting output directory):**
```bash
ros2 run camera_recorder_ros camera_recorder_service_node --ros-args -p output_dir:=/path/to/my/recordings
```

## Dependencies

*   ROS 2 (Humble recommended)
*   GStreamer (including `gst-plugins-good`, `gst-plugins-bad`, `gst-plugins-ugly` for various elements like `v4l2src`, `nvh264enc` or `x264enc`, `mp4mux`, `textoverlay`)
*   `rclpy`
*   `std_srvs`

## TODO

*   Load camera configurations from a YAML file instead of hardcoding.
*   Add launch files for easier configuration and startup.
*   Improve error handling and reporting from camera processes.
*   Re-introduce custom service for selecting specific cameras in the service node (requires a separate interface package). 