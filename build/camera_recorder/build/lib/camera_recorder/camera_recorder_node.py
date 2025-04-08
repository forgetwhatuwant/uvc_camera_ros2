#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import multiprocessing
import time
import os
import signal
from datetime import datetime, timezone
import gi
import threading
import sys
import termios
import tty

# Import the CameraRecorder class from the package
from camera_recorder.camera_recorder import CameraRecorder

class CameraRecorderNode(Node):
    """ROS2 node to control multiple camera recorders."""
    
    def __init__(self):
        super().__init__('camera_recorder_node')
        
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
        
        # Create services
        self.start_srv = self.create_service(Trigger, 'start_recording', self.start_recording_callback)
        self.stop_srv = self.create_service(Trigger, 'stop_recording', self.stop_recording_callback)
        
        # Camera configs - same as in the original script
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
        self.running = True  # Flag to control keyboard listener thread
        
        # Initialize camera recorders
        self.initialize_cameras()
        
        # Start keyboard input thread for direct controls
        self.keyboard_thread = threading.Thread(target=self.keyboard_control_loop)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        self.get_logger().info('Camera recorder node initialized')
        self.get_logger().info('Press "r" to start recording, "s" to stop recording, "q" to quit')
    
    def initialize_cameras(self):
        """Initialize camera recorders without starting recording."""
        # Create output directory if it doesn't exist
        try:
            os.makedirs(self.output_dir, exist_ok=True)
            self.get_logger().info(f"Output directory: {self.output_dir}")
        except OSError as e:
            self.get_logger().error(f"Error creating directory {self.output_dir}: {e}")
            return
            
        self.get_logger().info('Initializing cameras...')
        
        for config in self.cam_configs:
            # Create recorder but don't start recording yet
            recorder = CameraRecorder(
                device=config['device'],
                cam_name=config['cam_name'],
                width=config['width'],
                height=config['height'],
                framerate=config['framerate'],
                bitrate=config['bitrate'],
                output_dir=self.output_dir,  # This will be updated when recording starts
                timezone_name=self.timezone,
                verbose=self.verbose,
                timestamp=self.timestamp_enabled,
                encoder=self.encoder,
                auto_start=False  # Don't auto-start recording
            )
            self.recorders.append(recorder)
            self.get_logger().info(f"Initialized camera {config['cam_name']} on {config['device']}")
    
    def getch(self):
        """Get a single character from stdin without waiting for enter."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
    def keyboard_control_loop(self):
        """Thread to handle keyboard controls."""
        self.get_logger().info("Keyboard control thread started")
        
        while self.running:
            try:
                key = self.getch()
                
                if key == 'r' or key == 'R':
                    # Start recording
                    if not self.is_recording:
                        self.get_logger().info("Keyboard command: Start recording")
                        request = Trigger.Request()
                        response = self.start_recording_callback(request, Trigger.Response())
                        self.get_logger().info(f"Response: {response.message}")
                    else:
                        self.get_logger().info("Recording already in progress")
                
                elif key == 's' or key == 'S':
                    # Stop recording
                    if self.is_recording:
                        self.get_logger().info("Keyboard command: Stop recording")
                        request = Trigger.Request()
                        response = self.stop_recording_callback(request, Trigger.Response())
                        self.get_logger().info(f"Response: {response.message}")
                    else:
                        self.get_logger().info("No recording in progress")
                
                elif key == 'q' or key == 'Q':
                    # Quit the application
                    self.get_logger().info("Keyboard command: Quit")
                    self.running = False
                    
                    # Clean up and stop node
                    if self.is_recording:
                        self._shutdown_processes()
                    
                    # Signal the main thread to exit
                    rclpy.get_global_executor().wake()
            
            except Exception as e:
                self.get_logger().error(f"Error in keyboard control loop: {e}")
                # Brief pause to avoid tight loop in case of error
                time.sleep(0.1)
    
    def start_recording_callback(self, request, response):
        """ROS2 service callback to start recording."""
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
        
        # Start recording processes
        self.recording_processes = []
        
        for i, config in enumerate(self.cam_configs):
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
        response.message = f"Started recording to {self.run_dir}"
        return response
    
    def stop_recording_callback(self, request, response):
        """ROS2 service callback to stop recording."""
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
        recorder.run()
    
    def _shutdown_processes(self):
        """Gracefully shut down all camera processes."""
        if not self.recording_processes:
            return
        
        # First send SIGINT to all processes
        for p in self.recording_processes:
            if p.is_alive():
                try:
                    self.get_logger().info(f"Sending SIGINT to {p.name} (PID: {p.pid})...")
                    os.kill(p.pid, signal.SIGINT)
                except Exception as e:
                    self.get_logger().error(f"Error sending signal to {p.name}: {e}")
        
        # Wait for processes to shut down
        self.get_logger().info("Waiting for processes to shut down (5s timeout)...")
        
        # Wait with timeout
        deadline = time.time() + 5  # 5 second timeout
        for p in self.recording_processes:
            remaining = max(0.1, deadline - time.time())
            p.join(timeout=remaining)
        
        # Force terminate any remaining processes
        for p in self.recording_processes:
            if p.is_alive():
                self.get_logger().warn(f"Process {p.name} (PID: {p.pid}) still running - terminating forcefully")
                p.terminate()
                p.join(timeout=1.0)
                
                # Extra check for stubborn processes
                if p.is_alive():
                    self.get_logger().warn(f"Process {p.name} refusing to terminate - sending SIGKILL")
                    try:
                        os.kill(p.pid, signal.SIGKILL)
                    except:
                        pass
        
        self.recording_processes = []
        self.get_logger().info("All camera processes have been shut down")

def main(args=None):
    rclpy.init(args=args)
    node = CameraRecorderNode()
    
    try:
        # Use a multithreaded executor to handle both the main thread and keyboard input
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.running = False  # Stop keyboard thread
        if node.is_recording:
            node._shutdown_processes()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 