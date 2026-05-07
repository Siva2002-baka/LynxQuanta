import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf = os.path.join(get_package_share_directory('lynx_quanta'), 'urdf', 'm20_with_arm', 'm20_with_arm.urdf')
    control_yaml_file = os.path.join(get_package_share_directory('lynx_quanta'), 'config', 'm20_with_arm_controller.yaml')
    robot_desc = ParameterValue(Command(
        [
            'xacro ', 
            # 'cat ',
            urdf, 
            ' ', 
            'ros2_control_yaml:=', control_yaml_file,
            ]
        ),value_type=str)

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    # world_file = os.path.join(get_package_share_directory('turtlebot'), 'worlds', 'maze_2_6x5.sdf')
    # world_file = os.path.join(get_package_share_directory('gazebo_worlds'), 'worlds', 'house.world')

    # Include the gz sim launch file  
    gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args" :  f'-r empty.sdf' #{world_file}' #'-r empty.sdf'
        }.items()
    )
    # Spawn Rover Robot
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "m20_with_arm",
            "-allow_renaming", "true",
            "-x", "1.0",
            "-y", "1.0",
            "-z", "0.6",
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
                # ROS -> GZ
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",

            # GZ -> ROS
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            # "/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        ],
        parameters=[
            {'use_sim_time': use_sim_time},
            ],
    )

    # Robot state publisher
    params = {
        'use_sim_time': use_sim_time, 
        'robot_description': robot_desc, 
        'publish_frequency': 30.0, 
        'ignore_timestamp': True,
        }
    start_robot_state_publisher_cmd = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params],
            remappings=[('/cmd_vel_nav', '/cmd_vel')],
            arguments=[])
 
    # slam_toolbox = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'
    #         ])

    #     ]),
    #     launch_arguments={
    #         'slam_params_file': PathJoinSubstitution(
    #             [
    #                 FindPackageShare('lynx_m20'), 'config', 'mapper_params_online_async.yaml',
    #             ]
    #         ),
    #         'use_sim_time': 'true',

    #     }.items()
    # )
    # nav2 = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([
    #             FindPackageShare('nav2_bringup'),
    #             'launch',
    #             'navigation_launch.py'
    #         ])
    #     ),
    #     launch_arguments={
    #         'use_sim_time': 'true',
    #         # 'slam': 'False',
    #         # 'map_subscribe_transient_local': 'true',
    #         'params_file': PathJoinSubstitution([
    #                 FindPackageShare('lynx_m20'),
    #                 'config',
    #                 'nav2_params.yaml'
    #     ]),
    #     }.items()
    # )


    # rviz_node = Node(
    #         package='rviz2',
    #         executable='rviz2',
    #         name='rviz2',
    #         output='screen',
    #         parameters=[{'use_sim_time': use_sim_time}],
    #         arguments=['-d', os.path.join(
    #                     get_package_share_directory('lynx_m20'), 'config', 'lynx_m20.rviz')]
    #     )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    leg_pose_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["leg_pose_controller"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    wheel_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheel_velocity_controller"],
        output="screen",
    )
    wheel_controller_node = Node(
        package="lynx_quanta",
        executable="wheel_controller",
        name="wheel_controller",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    # Create the launch description and populate
    ld = LaunchDescription()

    # ld.add_action(set_gazebo_model_path)
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    # Launch Gazebo
    ld.add_action(gz_sim)
    ld.add_action(gz_spawn_entity)
    ld.add_action(gz_ros2_bridge)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(leg_pose_controller_spawner)
    ld.add_action(wheel_velocity_controller_spawner)
    ld.add_action(wheel_controller_node)
    # ld.add_action(joint_state_publisher_node)
    # Launch Robot State Publisher
    ld.add_action(start_robot_state_publisher_cmd)
    # ld.add_action(slam_toolbox)
    # ld.add_action(rviz_node)
    # ld.add_action(nav2)
    # ld.add_action(static_tf_bridge)
    return ld