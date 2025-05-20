from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Generate robot_description from xacro
    urdf_file = PathJoinSubstitution([
        FindPackageShare("drone_description"),
        "urdf",
        "drone_system.urdf.xacro"
    ])
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", urdf_file]),
        value_type=str
    )

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # Dummy Joint State Publisher (for passive arm/braces)
        Node(
            package='drone_bringup',
            executable='dummy_joint_state_publisher',
            output='screen'
        ), 

        # PID Controller Node
        Node(
            package='drone_bringup',
            executable='pid_control_node',
            output='screen'
        ),

        # Toggle Mechanism Service Node (handles /column_to_slider/effort_command)
        Node(
            package='drone_bringup',
            executable='toggle_mechanism_service',
            output='screen'
        ),

        # GUI for Mechanism Control 
        Node(
            package='drone_bringup',
            executable='mechanism_gui',
            output='screen'
        ),

        # Delay before spawning entity into Gazebo
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description', '-entity', 'drone'],
                    output='screen'
                )
            ]
        ),

        # Delay before applying initial joint efforts
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='drone_bringup',
                    executable='init_joint_positions',
                    output='screen'
                )
            ]
        ),
    ])
