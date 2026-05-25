from setuptools import find_packages, setup #type: ignore
import os

package_name = 'lynx_quanta'

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            # Construct the full local path
            local_path = os.path.join(path, filename)
            # Construct the destination path (share/package_name/path)
            install_path = os.path.join('share', package_name, path)
            paths.append((install_path, [local_path]))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *package_files('meshes'),
        *package_files('urdf'),
        *package_files('worlds'),
        *package_files('config'),
        *package_files('launch'),
        *package_files('maps'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pavan',
    maintainer_email='18pavansai@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wheel_controller = lynx_quanta.wheel_controller:main',
            'leg_controller = lynx_quanta.leg_pose_controller:main',
            'teleop_node = lynx_quanta.teleop_node:main',
            'lynx_brain = lynx_quanta.lynx_brain:main',
            'depth_visualizer = lynx_quanta.depth_visualizer:main',
            'lidar_merger = lynx_quanta.lidar_merger:main',
            'arm_controller = lynx_quanta.piper_ik_arm_controller:main',
        ],
    },
)
