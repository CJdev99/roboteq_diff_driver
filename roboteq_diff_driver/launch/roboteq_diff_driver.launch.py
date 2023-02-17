import launch
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    parameters_file_path = PathJoinSubstitution([FindPackageShare('my_package'), 'config', 'my_params.yaml'])

    # Declare launch arguments
    pub_odom_tf = DeclareLaunchArgument('pub_odom_tf', default_value='true', description='Whether to publish the odometry transform')
    odom_frame = DeclareLaunchArgument('odom_frame', default_value='odom', description='The name of the odom frame')
    base_frame = DeclareLaunchArgument('base_frame', default_value='base_link', description='The name of the base frame')
    cmdvel_topic = DeclareLaunchArgument('cmdvel_topic', default_value='cmd_vel', description='The name of the cmdvel topic')
    odom_topic = DeclareLaunchArgument('odom_topic', default_value='odom', description='The name of the odom topic')
    port = DeclareLaunchArgument('port', default_value='/dev/ttyACM0', description='The serial port to use')
    baud = DeclareLaunchArgument('baud', default_value='115200', description='The baud rate to use')
    open_loop = DeclareLaunchArgument('open_loop', default_value='false', description='Whether to use open loop mode')
    wheel_circumference = DeclareLaunchArgument('wheel_circumference', default_value='0.3192', description='The wheel circumference in meters')
    track_width = DeclareLaunchArgument('track_width', default_value='0.4318', description='The track width in meters')
    encoder_ppr = DeclareLaunchArgument('encoder_ppr', default_value='900', description='The encoder pulses per revolution')
    encoder_cpr = DeclareLaunchArgument('encoder_cpr', default_value='3600', description='The encoder counts per revolution')
    max_amps = DeclareLaunchArgument('max_amps', default_value='5.0', description='The maximum motor current in amps')
    max_rpm = DeclareLaunchArgument('max_rpm', default_value='100', description='The maximum motor RPM')

    # Set environment variables
    ld_library_path = SetEnvironmentVariable('LD_LIBRARY_PATH', ':/opt/ros/melodic/lib')

    # Define nodes
    my_node = Node(
        package='my_package',
        executable='my_node',
        parameters=[parameters_file_path],
        remappings=[('cmd_vel', cmdvel_topic), ('odom', odom_topic)],
        output='screen'
    )

    # Specify the order of execution
    return launch.LaunchDescription([
        pub_odom_tf,
        odom_frame,
        base_frame,
        cmdvel_topic,
        odom_topic,
        port,
        baud,
        open_loop,
        wheel_circumference,
        track_width,
        encoder_ppr,
        encoder_cpr,
        max_amps,
        max_rpm,
        ld_library_path,
        my_node,
    ])
