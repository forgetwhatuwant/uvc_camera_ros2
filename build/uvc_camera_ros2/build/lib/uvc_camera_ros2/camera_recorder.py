#!/usr/bin/env python3

from datetime import timezone, datetime, timedelta
import multiprocessing
import gi
import sys
import signal
import os
import argparse

# Attempt to import zoneinfo for timezone conversion
try:
    import zoneinfo
    _ZONEINFO_AVAILABLE = True
except ImportError:
    _ZONEINFO_AVAILABLE = False
    print("Warning: 'zoneinfo' module not found or 'tzdata' package not installed.", file=sys.stderr)
    print("         Timezone conversion will not be available.", file=sys.stderr)

gi.require_version('Gst', '1.0')
gi.require_version('GstApp', '1.0')
from gi.repository import Gst, GstApp, GLib


class CameraRecorder:
    """Class to record video from a camera with timestamp tracking."""

    def __init__(self, 
                device="/dev/video0", 
                cam_name="cam0",
                width=1280, 
                height=720, 
                framerate="30/1",
                bitrate=8000000,
                output_dir="temp",
                timezone_name="Asia/Shanghai",
                verbose=False,
                timestamp=True,
                encoder="h264",
                auto_start=False):  # Add auto_start parameter to control if recording starts immediately
        """Initialize camera recorder with the given parameters."""
        # Configuration
        self.device = device
        self.cam_name = cam_name
        self.width = width
        self.height = height
        self.framerate = framerate
        self.bitrate = bitrate
        self.output_dir = output_dir
        self.verbose = verbose
        self.timestamp = timestamp  # Store timestamp setting
        self.encoder = encoder      # Store encoder type (h264 or h265)
        self.auto_start = auto_start  # Whether to start recording immediately
        
        # State variables
        self.main_loop = None
        self.pipeline = None
        self.bus = None
        self.first_frame_processed = False
        self.pipeline_start_system_time = None
        self.filename = None
        self.timestamp_file = None  # File handle for timestamp data
        self.frame_count = 0  # Counter to track frame indices
        
        # Set up timezone if available
        if _ZONEINFO_AVAILABLE:
            self.tz_local = zoneinfo.ZoneInfo(timezone_name)
            print(f"[{self.cam_name}] Using timezone: {timezone_name}")
        else:
            self.tz_local = None
            
        # Build pipeline description
        self.pipeline_desc = self._build_pipeline_description()
        
        # Initialize GStreamer
        Gst.init(sys.argv[1:] if len(sys.argv) > 1 else None)

    def _build_pipeline_description(self):
        """Build GStreamer pipeline string based on configuration."""
        # Choose the encoder element based on encoder setting
        if self.encoder == "h265":
            encoder_element = f"nvv4l2h265enc bitrate={self.bitrate}"
            parser_element = "h265parse"
        else:  # Default to h264
            encoder_element = f"nvv4l2h264enc bitrate={self.bitrate}"
            parser_element = "h264parse"
            
        return (
            f"v4l2src device={self.device} "
            f"! image/jpeg,format=MJPG,width={self.width},height={self.height},framerate={self.framerate} "
            f"! nvv4l2decoder mjpeg=1 "
            f"! nvvidconv "
            f"! video/x-raw(memory:NVMM),format=I420 " # Raw video in NVMM memory after conversion
            f"! tee name=t " # Split the stream
            # Branch 1: Encoding and File Saving
            f"t. ! queue ! {encoder_element} ! {parser_element} ! qtmux ! filesink name=fsink async=false sync=false "
            # Branch 2: Timestamp Extraction
            f"t. ! queue ! appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false " # Get raw frames here
        )

    def on_new_sample(self, appsink):
        """Callback function executed when appsink receives a new sample."""
        sample = appsink.emit("pull-sample")
        if sample:
            buffer = sample.get_buffer()
            if buffer:
                pts = buffer.pts  # Presentation Timestamp in nanoseconds

                # --- Process first frame ---
                if not self.first_frame_processed and pts != Gst.CLOCK_TIME_NONE:
                    self.first_frame_processed = True
                    first_frame_pts = pts  # Store PTS of the first frame
                    self.frame_count = 0  # Reset frame counter

                    # Always print first frame information regardless of verbose setting
                    print(f"[{self.cam_name}] " + "-" * 40)
                    print(f"[{self.cam_name}] *** First frame PROCESSED by appsink! ***")
                    # Print the system time recorded when pipeline started
                    if self.pipeline_start_system_time:
                        print(f"[{self.cam_name}]     Pipeline Start Time (UTC): {self.pipeline_start_system_time.isoformat()}")
                        # Also print local timezone if possible
                        if self.tz_local:
                            print(f"[{self.cam_name}]                             ( {self.pipeline_start_system_time.astimezone(self.tz_local).isoformat()} )")
                    else:
                        # This case should ideally not happen if the code flow is correct
                        print(f"[{self.cam_name}]     Pipeline Start Time: (Not recorded!)")
                    print(f"[{self.cam_name}]     First Frame PTS: {first_frame_pts} ns ({first_frame_pts / Gst.SECOND:.6f} s)")
                    print(f"[{self.cam_name}] " + "-" * 40)
                    # Skip regular printing for this first frame below

                    # Write first frame to timestamp file if enabled
                    if self.timestamp and self.timestamp_file:
                        frame_walltime = self.pipeline_start_system_time + timedelta(seconds=first_frame_pts / Gst.SECOND)
                        frame_unix_time = frame_walltime.timestamp()
                        self.timestamp_file.write(f"{self.frame_count} {frame_unix_time:.6f}\n")
                        self.timestamp_file.flush()  # Ensure data is written to disk
                        self.frame_count += 1

                # --- Process subsequent frames ---
                elif self.first_frame_processed:
                    if pts != Gst.CLOCK_TIME_NONE:
                        # Calculate walltime based on pipeline start time + PTS
                        pts_seconds = pts / Gst.SECOND  # Convert nanoseconds to seconds
                        frame_walltime = self.pipeline_start_system_time + timedelta(seconds=pts_seconds)
                        # Calculate Unix timestamp (seconds since epoch)
                        frame_unix_time = frame_walltime.timestamp()
                        
                        # Write frame data to timestamp file if enabled
                        if self.timestamp and self.timestamp_file:
                            self.timestamp_file.write(f"{self.frame_count} {frame_unix_time:.6f}\n")
                            # Flush periodically to ensure data is written even if program crashes
                            if self.frame_count % 30 == 0:  # Flush every 30 frames
                                self.timestamp_file.flush()
                            self.frame_count += 1
                        
                        # Only print per-frame timestamps if verbose mode is enabled
                        if self.verbose:
                            print(f"[{self.cam_name}] Frame PTS: {pts} ns ({pts_seconds:.6f} s) - Walltime: {frame_walltime.isoformat()} UTC (Unix: {frame_unix_time:.6f})", end="")
                            # Also print local timezone time if available
                            if self.tz_local:
                                print(f" ({frame_walltime.astimezone(self.tz_local).isoformat()})")
                            else:
                                print()  # Just add newline if timezone not available
                    elif self.verbose:
                        print(f"[{self.cam_name}] Frame PTS: (Invalid/None)")

            return Gst.FlowReturn.OK
        else:
            # This can happen during shutdown
            pass
        return Gst.FlowReturn.ERROR

    def on_bus_message(self, bus, message):
        """Callback for messages from the pipeline bus."""
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"[{self.cam_name}] !!! Pipeline Error: {err} ({debug})", file=sys.stderr)
            if self.main_loop and self.main_loop.is_running():
                self.main_loop.quit()
        elif t == Gst.MessageType.EOS:
            print(f"[{self.cam_name}] --- End-of-stream reached ---")
            if self.main_loop and self.main_loop.is_running():
                self.main_loop.quit()
        elif t == Gst.MessageType.WARNING:
            err, debug = message.parse_warning()
            print(f"[{self.cam_name}] ### Pipeline Warning: {err} ({debug})", file=sys.stderr)
        elif t == Gst.MessageType.STATE_CHANGED:
            old_state, new_state, pending_state = message.parse_state_changed()
            # Only care about messages from the pipeline itself
            if message.src == self.pipeline:
                print(f"[{self.cam_name}] Pipeline state changed from {old_state.value_nick} to {new_state.value_nick}")

        return True  # Important: Keep listening to messages

    def quit_main_loop_if_running(self):
        """Quit the main loop if it's running."""
        if self.main_loop and self.main_loop.is_running():
            print(f"[{self.cam_name}] Quitting main loop now.", file=sys.stderr)
            self.main_loop.quit()
        # Ensure this function doesn't repeat if called via timeout_add_seconds
        return GLib.SOURCE_REMOVE

    def signal_handler(self, sig, frame):
        """Handles SIGINT/SIGTERM for graceful shutdown."""
        try:
            # Use simple print since we're only on Linux and not worried about reentrant 
            # calls as much (still using stderr for safety)
            print(f"\n[{self.cam_name}] Caught signal {sig}, sending EOS signal to pipeline...", file=sys.stderr)
            
            if self.pipeline:
                # Try sending EOS first for graceful muxer finalization
                self.pipeline.send_event(Gst.Event.new_eos())
                # Give some time for EOS to propagate before quitting loop directly
                GLib.timeout_add_seconds(1, self.quit_main_loop_if_running)
            else:
                if self.main_loop and self.main_loop.is_running():
                    self.main_loop.quit()
        except Exception as e:
            print(f"Error in signal handler: {e}", file=sys.stderr)

    def cleanup(self):
        """Stops the pipeline and cleans up resources."""
        try:
            if self.timestamp_file:
                try:
                    self.timestamp_file.flush()
                    self.timestamp_file.close()
                    print(f"[{self.cam_name}] Timestamp file closed. Wrote {self.frame_count} frames.", file=sys.stderr)
                except Exception as e:
                    print(f"[{self.cam_name}] Error closing timestamp file: {e}", file=sys.stderr)
                self.timestamp_file = None
                
            if self.bus:
                try:
                    self.bus.remove_signal_watch()
                except Exception as e:
                    print(f"[{self.cam_name}] Error removing bus signal watch: {e}", file=sys.stderr)
                self.bus = None  # Allow GC
                
            if self.pipeline:
                try:
                    print(f"[{self.cam_name}] Setting pipeline to NULL state...", file=sys.stderr)
                    # It's good practice to wait briefly for state change if possible,
                    # but setting to NULL should be synchronous usually.
                    self.pipeline.set_state(Gst.State.NULL)
                    print(f"[{self.cam_name}] Pipeline stopped.", file=sys.stderr)
                except Exception as e:
                    print(f"[{self.cam_name}] Error stopping pipeline: {e}", file=sys.stderr)
                self.pipeline = None  # Allow GC
        except Exception as e:
            print(f"[{self.cam_name}] Error during cleanup: {e}", file=sys.stderr)

    def setup_pipeline(self):
        """Set up the GStreamer pipeline without starting it."""
        # Create the main loop
        self.main_loop = GLib.MainLoop()

        # Set up signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        # --- Create Output Directory ---
        try:
            os.makedirs(self.output_dir, exist_ok=True)
            print(f"[{self.cam_name}] Output directory: '{self.output_dir}'")
        except OSError as e:
            print(f"[{self.cam_name}] Error creating directory {self.output_dir}: {e}", file=sys.stderr)
            return False

        # --- Generate Filenames ---
        timestamp_str = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        # Include camera name in the filename
        base_filename = f"{self.cam_name}_video{self.encoder}_{timestamp_str}"
        self.filename = os.path.join(self.output_dir, f"{base_filename}.mp4")
        print(f"[{self.cam_name}] Saving video to: {self.filename}")
        
        # Create timestamp file if enabled
        if self.timestamp:
            timestamp_filename = os.path.join(self.output_dir, f"{base_filename}.txt")
            try:
                self.timestamp_file = open(timestamp_filename, 'w')
                print(f"[{self.cam_name}] Saving timestamps to: {timestamp_filename}")
                # Write header with format information
                self.timestamp_file.write("# Frame timestamps for {}\n".format(base_filename))
                self.timestamp_file.write("# Format: <frame_index> <unix_timestamp_seconds>\n")
                self.timestamp_file.write("# Recording started at: {} UTC\n".format(
                    datetime.now(timezone.utc).isoformat()))
                self.timestamp_file.flush()
            except Exception as e:
                print(f"[{self.cam_name}] Warning: Could not create timestamp file: {e}", file=sys.stderr)
                self.timestamp_file = None
        
        # --- Parse Pipeline ---
        try:
            self.pipeline = Gst.parse_launch(self.pipeline_desc)
        except GLib.Error as e:
            print(f"[{self.cam_name}] Error parsing pipeline: {e}", file=sys.stderr)
            return False

        # --- Get Elements ---
        appsink = self.pipeline.get_by_name('sink')
        filesink = self.pipeline.get_by_name('fsink')

        if not self.pipeline:
            print(f"[{self.cam_name}] Error: Failed to create pipeline.", file=sys.stderr)
            return False
        if not appsink:
            print(f"[{self.cam_name}] Error: Could not find appsink element 'sink'.", file=sys.stderr)
            return False
        if not filesink:
            print(f"[{self.cam_name}] Error: Could not find filesink element 'fsink'.", file=sys.stderr)
            return False

        # --- Configure Elements ---
        filesink.set_property('location', self.filename)

        # --- Reset State Variables ---
        self.first_frame_processed = False  # Reset flag for this run
        self.pipeline_start_system_time = None  # Reset time for this run

        # Connect the 'new-sample' signal
        appsink.connect("new-sample", self.on_new_sample)

        # --- Setup Bus Watch ---
        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus.connect("message", self.on_bus_message)
        
        return True

    def start_recording(self):
        """Start the recording pipeline."""
        if not self.pipeline:
            success = self.setup_pipeline()
            if not success:
                return False
        
        # --- Start Pipeline ---
        print(f"[{self.cam_name}] Starting pipeline...")
        # Record time just before setting state to PLAYING
        self.pipeline_start_system_time = datetime.now(timezone.utc)
        print(f"[{self.cam_name}] Recorded pipeline start time (UTC): {self.pipeline_start_system_time.isoformat()}")
        if self.tz_local:
            print(f"[{self.cam_name}]                             (Local): {self.pipeline_start_system_time.astimezone(self.tz_local).isoformat()}")

        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            print(f"[{self.cam_name}] !!! Unable to set the pipeline to the playing state.", file=sys.stderr)
            self.cleanup()
            return False
        elif ret == Gst.StateChangeReturn.NO_PREROLL:
            print(f"[{self.cam_name}] ### Pipeline is live and does not need PREROLL...")
        elif ret == Gst.StateChangeReturn.ASYNC:
            print(f"[{self.cam_name}] ### Pipeline is PREROLLING asynchronously (will transition to PLAYING later)...")
        
        return True

    def run(self):
        """Set up and run the recording pipeline."""
        # Initialize GStreamer if not already done
        if not Gst.is_initialized():
            Gst.init(sys.argv[1:] if len(sys.argv) > 1 else None)
        
        # Set up the pipeline
        success = self.setup_pipeline()
        if not success:
            return
        
        # If auto_start is enabled, start recording immediately
        if self.auto_start:
            success = self.start_recording()
            if not success:
                return

        try:
            # Run the main loop
            print(f"[{self.cam_name}] Pipeline running... Press Ctrl+C to stop.")
            self.main_loop.run()
        except KeyboardInterrupt:
            print(f"\n[{self.cam_name}] Ctrl+C detected, stopping pipeline gracefully...")
        finally:
            self.cleanup()


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description='Record video from two cameras with timestamp tracking')
    parser.add_argument('--output', '-o', default='temp', help='Output directory (default: temp)')
    parser.add_argument('--timezone', '-tz', default='Asia/Shanghai', help='Timezone (default: Asia/Shanghai)')
    parser.add_argument('--verbose', '-v', action='store_true', help='Enable verbose output with per-frame timestamps')
    parser.add_argument('--no-timestamp', action='store_true', help='Disable saving frame timestamps to text file')
    parser.add_argument('--encoder', '-e', choices=['h264', 'h265'], default='h264', help='Video encoder to use (default: h264)')
    return parser.parse_args()


def run_camera_process(device, cam_name, width, height, framerate, bitrate, output_dir, timezone_name, verbose, timestamp, encoder):
    """Run a camera recorder in a separate process."""
    # Initialize separate process name for better monitoring
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
        auto_start=True  # Auto-start when using the process directly
    )
    recorder.run()


def main():
    """Main function to create and run camera recorders."""
    args = parse_arguments()
    
    # Create a timestamped run directory inside the output directory
    timestamp_str = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    run_dir = os.path.join(args.output, f"run_{timestamp_str}")
    
    try:
        os.makedirs(run_dir, exist_ok=True)
        print(f"Created run directory: {run_dir}")
    except OSError as e:
        print(f"Error creating run directory {run_dir}: {e}", file=sys.stderr)
        sys.exit(1)
    
    # Fixed configuration for cameras
    cam_configs = [
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
        
    
    print("Starting camera recording...")
    if args.verbose:
        print("Verbose mode enabled: Per-frame timestamps will be printed.")
    timestamp_enabled = not args.no_timestamp
    if timestamp_enabled:
        print("[INFO] Timestamp files will be created for each video.")
    print(f"[INFO] Using {args.encoder.upper()} encoder for video compression.")
    
    # Create and start processes for each camera
    processes = []
    for config in cam_configs:
        process = multiprocessing.Process(
            target=run_camera_process,
            args=(
                config['device'],
                config['cam_name'],
                config['width'],
                config['height'],
                config['framerate'],
                config['bitrate'],
                run_dir,  # Use the run directory instead of args.output directly
                args.timezone,
                args.verbose,
                timestamp_enabled,
                args.encoder
            )
        )
        processes.append(process)
        process.start()
        print(f"Started process for camera {config['cam_name']} on {config['device']} (PID: {process.pid})")
    
    # Improved shutdown handling
    def shutdown_processes(sig=None, frame=None):
        """Gracefully shut down all camera processes."""
        try:
            if sig:
                print(f"\nMain process received signal {sig}, shutting down gracefully...", file=sys.stderr)
            else:
                print("\nShutting down all camera processes...", file=sys.stderr)
            
            # First send SIGINT to all processes to trigger their own graceful shutdown
            for p in processes:
                if p.is_alive():
                    try:
                        print(f"Sending SIGINT to {p.name} (PID: {p.pid})...", file=sys.stderr)
                        # Linux-only: send SIGINT (same as Ctrl+C)
                        import os
                        import signal as sig_module
                        os.kill(p.pid, sig_module.SIGINT)
                    except Exception as e:
                        print(f"Error sending signal to {p.name}: {e}", file=sys.stderr)
            
            # Give processes time to shut down gracefully
            import time
            print("Waiting for processes to shut down (5s timeout)...", file=sys.stderr)
            
            # Wait for each process with timeout
            deadline = time.time() + 5  # 5 second timeout
            for p in processes:
                remaining = max(0.1, deadline - time.time())  # At least 0.1s wait, up to deadline
                p.join(timeout=remaining)
            
            # Force terminate any remaining processes
            for p in processes:
                if p.is_alive():
                    print(f"Process {p.name} (PID: {p.pid}) still running - terminating forcefully", file=sys.stderr)
                    p.terminate()
                    p.join(timeout=1.0)  # Short timeout for termination
                    
                    # Extra check for stubborn processes
                    if p.is_alive():
                        print(f"Process {p.name} (PID: {p.pid}) refusing to terminate - sending SIGKILL", file=sys.stderr)
                        try:
                            import os
                            import signal as sig_module
                            os.kill(p.pid, sig_module.SIGKILL)
                        except:
                            pass
            
            print("All camera processes have been shut down.", file=sys.stderr)
            if sig:  # Only exit if called as signal handler
                sys.exit(0)
        except Exception as e:
            print(f"Error during shutdown: {e}", file=sys.stderr)
    
    # Register signal handlers
    signal.signal(signal.SIGINT, shutdown_processes)
    signal.signal(signal.SIGTERM, shutdown_processes)
    
    # Wait for all processes to complete
    try:
        # Use a loop with timeout to allow checking for issues
        while any(p.is_alive() for p in processes):
            # Wait a bit and then check if any process died unexpectedly
            time.sleep(1.0)
            for p in processes:
                if not p.is_alive() and p.exitcode != 0:
                    print(f"WARNING: Process {p.name} exited unexpectedly with code {p.exitcode}", file=sys.stderr)
    except KeyboardInterrupt:
        # This will be caught by the signal handler above
        pass
    finally:
        # Ensure shutdown happens even if we get an unexpected exception
        shutdown_processes()
        
    print("Main process exiting.")


if __name__ == '__main__':
    # Import time here for use in main
    import time
    
    # No need for multiprocessing start method setting on Linux
    main()