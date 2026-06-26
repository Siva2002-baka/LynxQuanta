M20 GLIM mapping-only fixes

1. URDF LiDAR joints corrected to official M20 manual coordinates:
   front LiDAR: xyz=0.32028 0.0 -0.013
   rear LiDAR:  xyz=-0.32028 0.0 -0.013

2. Rear LiDAR resolution reduced to 360x16 at 5 Hz, same as front LiDAR.

3. config_sensors.json T_lidar_imu corrected for front LiDAR input:
   lidar_front_link -> imu_link = [-0.25708, -0.0268, -0.0305, 0, 0, 0, 1]

4. Mapping launch is kept mapping-only:
   Uses cmd_vel, clock, imu, front/rear LiDAR, front camera image/camera_info only.
   EKF and lidar_merger remain disabled.

Install:
   cp m20_with_piper.urdf ~/lynx_ws/src/lynx_quanta/urdf/m20_with_arm/m20_with_piper.urdf
   cp glim_mapping.launch.py ~/lynx_ws/src/lynx_quanta/launch/glim_mapping.launch.py
   cp config_*.json ~/lynx_ws/src/lynx_quanta/config/glim/
   cp config.json ~/lynx_ws/src/lynx_quanta/config/glim/
   cd ~/lynx_ws && colcon build --symlink-install --packages-select lynx_quanta
   source install/setup.bash
