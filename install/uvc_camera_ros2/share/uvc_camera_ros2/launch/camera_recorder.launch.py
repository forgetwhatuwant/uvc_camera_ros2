from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    """Generate a launch description for the camera recorder node."""
    
    # Declare launch arguments
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='temp',
        description='Directory where recordings will be saved'
    )
    
    timezone_arg = DeclareLaunchArgument(
        'timezone',
        default_value='Asia/Shanghai',
        description='Timezone for timestamp files'
    )
    
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='False',
        description='Enable verbose output'
    )
    
    timestamp_enabled_arg = DeclareLaunchArgument(
        'timestamp_enabled',
        default_value='True',
        description='Enable timestamp files'
    )
    
    encoder_arg = DeclareLaunchArgument(
        'encoder',
        default_value='h264',
        description='Video encoder to use (h264 or h265)'
    )
    
    # Define the camera recorder node
    camera_recorder_node = Node(
        package='camera_recorder',  # Must match package name
        executable='camera_recorder_node.py',
        name='camera_recorder_node',
        output='screen',
        parameters=[{
            'output_dir': LaunchConfiguration('output_dir'),
            'timezone': LaunchConfiguration('timezone'),
            'verbose': LaunchConfiguration('verbose'),
            'timestamp_enabled': LaunchConfiguration('timestamp_enabled'),
            'encoder': LaunchConfiguration('encoder'),
        }],
        # Add a reminder about keyboard controls
        prefix=[
            'echo "\nUse keyboard controls: r=Start recording, s=Stop recording, q=Quit\n"; '
        ],
    )
    
    # Return the launch description
    return LaunchDescription([
        output_dir_arg,
        timezone_arg,
        verbose_arg,
        timestamp_enabled_arg,
        encoder_arg,
        camera_recorder_node,
    ]) 