"""
Optimized GLIM mapping launch for Lynx M20 + Piper in Gazebo.

Active mapping path:
  /lidar_front/points + /imu/data -> GLIM
  GLIM publishes map/odom/base TF and mapping clouds.

Important:
  Do not run EKF in this launch at the same time as GLIM. EKF is useful for
  wheel-odom navigation, but GLIM already estimates lidar-inertial odometry.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use Gazebo simulation clock",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    pkg_lynx = get_package_share_directory("lynx_quanta")
    urdf = os.path.join(pkg_lynx, "urdf", "m20_with_arm", "m20_with_piper_v3.urdf")
    ctrl_yaml = os.path.join(pkg_lynx, "config", "m20_with_piper_controller.yaml")
    glim_cfg_path = os.path.join(pkg_lynx, "config", "glim")
    world_file = os.path.join(pkg_lynx,"worlds",
        "small_house.world"   # change this to your actual world filename
    )
    gz_resource_path = os.pathsep.join([
        os.path.join(pkg_lynx, "models"),
        os.path.join(pkg_lynx, "worlds"),
        pkg_lynx,
        os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    ])
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=gz_resource_path,
    )

    robot_desc = ParameterValue(
        Command(["xacro ", urdf, " ", "ros2_control_yaml:=", ctrl_yaml]),
        value_type=str,
    )

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
        arguments=[
            "-topic", "/robot_description",
            "-name", "m20_with_arm",
            "-allow_renaming", "true",
            "-x", "0.0", "-y", "0.0", "-z", "0.6",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": robot_desc,
            "publish_frequency": 50.0,
            "ignore_timestamp": True,
        }],
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_ros2_bridge",
        output="screen",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",

            # GLIM mapping inputs.
            "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/lidar_front/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/lidar_front@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/lidar_rear/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/lidar_rear@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",  

            # Camera kept only to avoid empty image topic crashes in some GLIM builds.
            "/camera_front/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera_front/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        )

    glim_node = Node(
        package="glim_ros",
        executable="glim_rosnode",
        name="glim_rosnode",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"config_path": glim_cfg_path},
        ],
        sigterm_timeout="60.0",
        sigkill_timeout="10.0",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    brain = Node(
        package="lynx_quanta",
        executable="lynx_brain",
        name="lynx_brain",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    
    ld.add_action(set_gz_resource_path)
    ld.add_action(gz_sim)
    ld.add_action(gz_spawn)
    ld.add_action(gz_bridge)
    ld.add_action(rsp)

    ld.add_action(spawner("joint_state_broadcaster"))
    ld.add_action(spawner("leg_pose_controller"))
    ld.add_action(spawner("wheel_velocity_controller"))
    ld.add_action(spawner("arm_controller"))
    ld.add_action(spawner("gripper_controller"))

    ld.add_action(
        TimerAction(
            period=12.0,
            actions=[
                glim_node,
                # rviz_node,
            ],
        )
    )
    ld.add_action(brain)

    return ld
