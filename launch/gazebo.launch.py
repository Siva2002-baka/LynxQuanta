import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock')

    pkg_lynx    = get_package_share_directory('lynx_quanta')
    urdf        = os.path.join(pkg_lynx, 'urdf', 'm20_with_arm', 'm20_with_arm_v2.urdf')
    ctrl_yaml   = os.path.join(pkg_lynx, 'config', 'm20_with_arm_controller.yaml')

    robot_desc = ParameterValue(
        Command(['xacro ', urdf, ' ', 'ros2_control_yaml:=', ctrl_yaml]),
        value_type=str)

    # ── Gazebo Harmonic ───────────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    gz_spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name',  'm20_with_arm', '-allow_renaming', 'true',
            '-x', '1.0', '-y', '1.0', '-z', '0.6',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── Bridge ────────────────────────────────────────────────────────────────
    gz_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='gz_ros2_bridge', output='screen',
        arguments=[
            # ROS → GZ
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # GZ → ROS  (infrastructure)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── Robot State Publisher ─────────────────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{
            'use_sim_time':      use_sim_time,
            'robot_description': robot_desc,
            'publish_frequency': 30.0,
            'ignore_timestamp':  True,
        }],
        remappings=[('/cmd_vel_nav', '/cmd_vel')],
    )

    # ── ros2_control spawners ─────────────────────────────────────────────────
    joint_state_broadcaster = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': use_sim_time}], output='screen',
    )
    leg_pose_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=['leg_pose_controller'],
        parameters=[{'use_sim_time': use_sim_time}], output='screen',
    )
    wheel_velocity_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=['wheel_velocity_controller'], output='screen',
    )

    brain = Node(
        package='lynx_quanta', executable='lynx_brain', name='lynx_brain',
        output='screen', parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── Assemble ─────────────────────────────────────────────────────────────
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(gz_sim)
    ld.add_action(gz_spawn)
    ld.add_action(gz_bridge)
    ld.add_action(rsp)
    ld.add_action(joint_state_broadcaster)
    ld.add_action(leg_pose_controller)
    ld.add_action(wheel_velocity_controller)
    ld.add_action(brain)
    return ld