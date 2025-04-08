#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import multiprocessing
import time
import os
import signal
from datetime import datetime, timezone

# Import the CameraRecorder class from the package
from camera_recorder_ros.camera_recorder import CameraRecorder

class CameraRecorderServiceNode(Node):
    """ROS2 node to control multiple camera recorders using only service calls."""
    
    def __init__(self):
        super().__init__('camera_recorder_service_node')
        
        # Declare parameters
        self.declare_parameter('output_dir', 'temp')
        self.declare_parameter('timezone', 'Asia/Shanghai')
        self.declare_parameter('verbose', False)
        self.declare_parameter('timestamp_enabled', True)
        self.declare_parameter('encoder', 'h264')
        
        # Get parameters
        self.output_dir = self.get_parameter('output_dir').value
        self.timezone = self.get_parameter('timezone').value
        self.verbose = self.get_parameter('verbose').value
        self.timestamp_enabled = self.get_parameter('timestamp_enabled').value
        self.encoder = self.get_parameter('encoder').value
        
        # Create services using Trigger for both
        self.start_srv = self.create_service(Trigger, 'start_recording', self.start_recording_callback)
        self.stop_srv = self.create_service(Trigger, 'stop_recording', self.stop_recording_callback)
        
        # Camera configs
        self.cam_configs = [
            {
                'device': '/dev/video0',
                'cam_name': 'layer0_1',
                'width': 1280,
                'height': 720,
                'framerate': '30/1',
                'bitrate': 8000000,
            },
            {
                'device': '/dev/video2',
                'cam_name': 'layer1_1',
                'width': 1280,
                'height': 720,
                'framerate': '30/1',
                'bitrate': 8000000,
            },
            {
                'device': '/dev/video4',
                'cam_name': 'layer2_1',
                'width': 1280,
                'height': 720,
                'framerate': '30/1',
                'bitrate': 8000000,
            },
            {
                'device': '/dev/video6',
                'cam_name': 'layer3_1',
                'width': 1280,
                'height': 720,
                'framerate': '30/1',
                'bitrate': 8000000,
            },
            {
                'device': '/dev/video8',
                'cam_name': 'layer0_2',
                'width': 1280,
                'height': 720,
                'framerate': '30/1',
                'bitrate': 8000000,
            },
            {
                'device': '/dev/video10',
                'cam_name': 'layer1_2',
                'width': 1280,
                'height': 720,
                'framerate': '30/1',
                'bitrate': 8000000,
            },
            {
                'device': '/dev/video12',
                'cam_name': 'layer2_2',
                'width': 1280,
                'height': 720,
                'framerate': '30/1',
                'bitrate': 8000000,
            },
            {
                'device': '/dev/video14',
                'cam_name': 'layer3_2',
                'width': 1280,
                'height': 720,
                'framerate': '30/1',
                'bitrate': 8000000,
            },
        ]
        
        # Initialize variables
        self.recorders = []
        self.recording_processes = []
        self.is_recording = False
        self.run_dir = None
        
        self.get_logger().info('Camera recorder service node initialized')
        self.get_logger().info('Use ROS2 services to control recording:')
        self.get_logger().info('  - /start_recording (Trigger): Start recording from ALL configured cameras')
        self.get_logger().info('  - /stop_recording (Trigger): Stop recording from all cameras')
    
    def start_recording_callback(self, request, response):
        """ROS2 service callback to start recording (using Trigger)."""
        if self.is_recording:
            response.success = False
            response.message = "Recording is already in progress"
            return response
        
        # Create a timestamped run directory
        timestamp_str = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        self.run_dir = os.path.join(self.output_dir, f"run_{timestamp_str}")
        
        try:
            os.makedirs(self.run_dir, exist_ok=True)
            self.get_logger().info(f"Created run directory: {self.run_dir}")
        except OSError as e:
            self.get_logger().error(f"Error creating run directory {self.run_dir}: {e}")
            response.success = False
            response.message = f"Failed to create output directory: {e}"
            return response
        
        # Start recording processes for ALL configured cameras
        self.recording_processes = []
        self.get_logger().info("Starting recording for all configured cameras...")
        
        for config in self.cam_configs:
            process = multiprocessing.Process(
                target=self._run_camera_process,
                args=(
                    config['device'],
                    config['cam_name'],
                    config['width'],
                    config['height'],
                    config['framerate'],
                    config['bitrate'],
                    self.run_dir,
                    self.timezone,
                    self.verbose,
                    self.timestamp_enabled,
                    self.encoder
                )
            )
            process.name = f"CamProcess-{config['cam_name']}"
            self.recording_processes.append(process)
            process.start()
            self.get_logger().info(f"Started recording for camera {config['cam_name']} (PID: {process.pid})")
        
        self.is_recording = True
        response.success = True
        response.message = f"Started recording for all cameras to {self.run_dir}"
        return response
    
    def stop_recording_callback(self, request, response):
        """ROS2 service callback to stop recording (using Trigger)."""
        if not self.is_recording:
            response.success = False
            response.message = "No recording in progress"
            return response
        
        self.get_logger().info("Stopping all recordings...")
        self._shutdown_processes()
        
        self.is_recording = False
        response.success = True
        response.message = "Recording stopped"
        return response
    
    def _run_camera_process(self, device, cam_name, width, height, framerate, bitrate, output_dir, timezone_name, verbose, timestamp, encoder):
        """Run a camera recorder in a separate process."""
        # Initialize process name
        multiprocessing.current_process().name = f"CamProcess-{cam_name}"
        
        recorder = CameraRecorder(
            device=device,
            cam_name=cam_name,
            width=width,
            height=height,
            framerate=framerate,
            bitrate=bitrate,
            output_dir=output_dir,
            timezone_name=timezone_name,
            verbose=verbose,
            timestamp=timestamp,
            encoder=encoder,
            auto_start=True  # Auto-start in this process since we want to record immediately
        )
        # Wrap run() in a try-except to catch potential initialization errors within the process
        try:
            recorder.run()
        except Exception as e:
            # Log error from within the process if possible (might not show up in main node log easily)
            print(f"[Process {cam_name}] Error during recording: {e}", flush=True)
            # Consider using a multiprocessing Queue or Pipe to send errors back to the main node if needed
    
    def _shutdown_processes(self):
        """Gracefully shut down all camera processes."""
        if not self.recording_processes:
            return
        
        # First send SIGINT to all processes
        for p in self.recording_processes:
            if p.is_alive():
                try:
                    self.get_logger().info(f"Sending SIGINT to {p.name} (PID: {p.pid})...")
                    # Ensure pid is not None before sending signal
                    if p.pid is not None:
                        os.kill(p.pid, signal.SIGINT)
                    else:
                        self.get_logger().warn(f"Process {p.name} has no PID, cannot send SIGINT.")
                except ProcessLookupError:
                    self.get_logger().warn(f"Process {p.name} (PID: {p.pid}) not found.")
                except Exception as e:
                    self.get_logger().error(f"Error sending SIGINT to {p.name}: {e}")
        
        # Wait for processes to shut down
        self.get_logger().info("Waiting for processes to shut down (5s timeout)...")
        
        # Wait with timeout
        deadline = time.time() + 5  # 5 second timeout
        for p in self.recording_processes:
            if p.is_alive():
                remaining = max(0.1, deadline - time.time())
                p.join(timeout=remaining)
        
        # Force terminate any remaining processes
        for p in self.recording_processes:
            if p.is_alive():
                self.get_logger().warn(f"Process {p.name} (PID: {p.pid}) still running - terminating forcefully")
                # Ensure pid is not None before terminating/killing
                if p.pid is not None:
                    try:
                        p.terminate()
                        p.join(timeout=1.0) # Give terminate a moment
                        
                        if p.is_alive():
                            self.get_logger().warn(f"Process {p.name} still alive after terminate - sending SIGKILL")
                            os.kill(p.pid, signal.SIGKILL)
                            p.join(timeout=0.5) # Short wait after kill
                    except ProcessLookupError:
                        self.get_logger().warn(f"Process {p.name} (PID: {p.pid}) not found during termination.")
                    except Exception as e:
                        self.get_logger().error(f"Error terminating/killing {p.name}: {e}")
                else:
                    self.get_logger().warn(f"Process {p.name} has no PID, cannot terminate/kill.")
        
        self.recording_processes = []
        self.get_logger().info("All camera processes have been shut down")

def main(args=None):
    rclpy.init(args=args)
    node = CameraRecorderServiceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received, shutting down...")
    except Exception as e:
        if node:
            node.get_logger().error(f"Unhandled exception in main: {e}", exc_info=True)
        else:
            print(f"Unhandled exception before node initialization: {e}")
    finally:
        # Clean up
        if node and hasattr(node, 'is_recording') and node.is_recording:
            print("Ensuring recording processes are stopped...")
            node._shutdown_processes()
        if node:
            print("Destroying node...")
            node.destroy_node()
        if rclpy.ok():
            print("Shutting down rclpy...")
            rclpy.shutdown()
        print("Shutdown complete.")

if __name__ == '__main__':
    main() 