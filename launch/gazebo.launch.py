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
    urdf        = os.path.join(pkg_lynx, 'urdf', 'm20_with_arm', 'm20_with_piper.urdf')
    ctrl_yaml   = os.path.join(pkg_lynx, 'config', 'm20_with_piper_controller.yaml')

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
            "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
            '/camera_front/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_front/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/camera_front/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_front/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/camera_rear/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_rear/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/camera_rear/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_rear/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/lidar_front/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/lidar_rear/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            "/lidar_front@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/lidar_rear@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
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

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'
            ])

        ]),
        launch_arguments={
            'slam_params_file': PathJoinSubstitution(
                [
                    FindPackageShare('lynx_quanta'), 'config', 'mapper_params_online_async.yaml',
                ]
            ),
            'use_sim_time': 'true',

        }.items()
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

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",

    )

    depth_visualizer_node = Node(
        package="lynx_quanta",
        executable="depth_visualizer",
        name="depth_visualizer",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )


    dual_lidar_merger_node = Node(
        package='lynx_quanta',
        executable='lidar_merger',
        name='lidar_merger',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,

            'target_frame': 'base_link',

            'front_topic': '/lidar_front/points',
            'rear_topic': '/lidar_rear/points',

            'merged_cloud_topic': '/lidar_merged_points',
            'merged_scan_topic': '/scan',

            'min_height': -0.02,
            'max_height': 0.35,

            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.00349,

            'range_min': 0.15,
            'range_max': 50.0,

            'publish_rate': 10.0,
        }],
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
    ld.add_action(arm_controller_spawner)
    ld.add_action(slam_toolbox)
    ld.add_action(depth_visualizer_node)
    ld.add_action(dual_lidar_merger_node)
    ld.add_action(brain)
    return ld