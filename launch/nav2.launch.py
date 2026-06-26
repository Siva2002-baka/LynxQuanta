import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use Gazebo simulation clock",
    )

    pkg_lynx = get_package_share_directory("lynx_quanta")

    urdf = os.path.join(
        pkg_lynx,
        "urdf",
        "m20_with_arm",
        "m20_with_piper_v3.urdf",
    )

    ctrl_yaml = os.path.join(
        pkg_lynx,
        "config",
        "m20_with_piper_controller.yaml",
    )

    world_file = os.path.join(
        pkg_lynx,
        "worlds",
        "small_house.world",
    )

    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")

    declare_map = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(pkg_lynx, "maps", "map.yaml"),
    )

    declare_params = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg_lynx, "config", "nav2_mppi_params.yaml"),
    )

    # -------------------------------------------------------------------------
    # Gazebo model/resource paths
    # Required for model://aws_robomaker_residential_* inside small_house.world
    # -------------------------------------------------------------------------
    src_models = "/home/sutd/lynx_ws/src/lynx_quanta/models"
    install_models = os.path.join(pkg_lynx, "models")

    existing_gz = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    existing_ign = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")

    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=f"{src_models}:{install_models}:{existing_gz}",
    )

    set_ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=f"{src_models}:{install_models}:{existing_ign}",
    )

    robot_desc = ParameterValue(
        Command(["xacro ", urdf, " ", "ros2_control_yaml:=", ctrl_yaml]),
        value_type=str,
    )

    # -------------------------------------------------------------------------
    # Gazebo Harmonic
    # -------------------------------------------------------------------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": f"-r {world_file}",
        }.items(),
    )

    gz_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "m20_with_arm",
            "-allow_renaming",
            "true",
            "-x",
            "1.0",
            "-y",
            "1.0",
            "-z",
            "0.6",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # -------------------------------------------------------------------------
    # Bridge
    # -------------------------------------------------------------------------
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_ros2_bridge",
        output="screen",
        arguments=[
            # ROS -> Gazebo
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",

            # Gazebo -> ROS
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",

            "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",

            "/camera_front/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera_front/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/camera_front/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera_front/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",

            "/camera_rear/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera_rear/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/camera_rear/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera_rear/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",

            "/lidar_front/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/lidar_rear/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/lidar_front@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/lidar_rear@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # -------------------------------------------------------------------------
    # Robot State Publisher
    # -------------------------------------------------------------------------
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_desc,
                "publish_frequency": 30.0,
                "ignore_timestamp": True,
            }
        ],
    )

    # -------------------------------------------------------------------------
    # ros2_control spawners
    # Delay them slightly so Gazebo + robot spawn has time to create controller_manager.
    # -------------------------------------------------------------------------
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    leg_pose_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["leg_pose_controller"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    wheel_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheel_velocity_controller"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    delayed_controllers = TimerAction(
        period=6.0,
        actions=[
            joint_state_broadcaster,
            leg_pose_controller,
            wheel_velocity_controller,
            arm_controller_spawner,
            gripper_controller_spawner,
        ],
    )

    # -------------------------------------------------------------------------
    # SLAM Toolbox
    # Use only if you are mapping. For localization on an existing map,
    # you should usually disable SLAM and use Nav2 AMCL only.
    # -------------------------------------------------------------------------
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("slam_toolbox"),
                        "launch",
                        "online_async_launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "slam_params_file": PathJoinSubstitution(
                [
                    FindPackageShare("lynx_quanta"),
                    "config",
                    "mapper_params_online_async.yaml",
                ]
            ),
            "use_sim_time": "true",
        }.items(),
    )

    # -------------------------------------------------------------------------
    # Nav2
    # -------------------------------------------------------------------------
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "bringup_launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "map": map_yaml,
            "params_file": params_file,
            "autostart": "true",
        }.items(),
    )

    # -------------------------------------------------------------------------
    # Lynx helper nodes
    # -------------------------------------------------------------------------
    depth_visualizer_node = Node(
        package="lynx_quanta",
        executable="depth_visualizer",
        name="depth_visualizer",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    dual_lidar_merger_node = Node(
        package="lynx_quanta",
        executable="lidar_merger",
        name="lidar_merger",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "target_frame": "base_link",
                "front_topic": "/lidar_front/points",
                "rear_topic": "/lidar_rear/points",
                "merged_cloud_topic": "/lidar_merged_points",
                "merged_scan_topic": "/scan",
                "min_height": -0.02,
                "max_height": 0.35,
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.00349,
                "range_min": 0.15,
                "range_max": 50.0,
                "publish_rate": 10.0,
            }
        ],
    )

    brain = Node(
        package="lynx_quanta",
        executable="nav2_brain",
        name="navigation_brain",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-d",
            os.path.join(pkg_lynx, "config", "lynx_quanta.rviz"),
        ],
    )

    # -------------------------------------------------------------------------
    # Assemble
    # -------------------------------------------------------------------------
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map)
    ld.add_action(declare_params)

    # These MUST come before gz_sim.
    ld.add_action(set_gz_resource_path)
    ld.add_action(set_ign_resource_path)

    ld.add_action(gz_sim)
    ld.add_action(rsp)
    ld.add_action(gz_spawn)
    ld.add_action(gz_bridge)

    ld.add_action(delayed_controllers)

    ld.add_action(depth_visualizer_node)
    ld.add_action(dual_lidar_merger_node)
    ld.add_action(brain)
    # ld.add_action(slam_toolbox)

    ld.add_action(nav2)
    ld.add_action(rviz_node)

    return ld