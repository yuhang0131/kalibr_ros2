from setuptools import setup
from glob import glob
import os

package_name = 'kalibr_imu_camera'

setup(
    name=package_name,
    version='1.0.0',
    packages=[
        'kalibr_imu_camera',
        'kalibr_common',
        'kalibr_camera_calibration',
        'kalibr_imu_camera_calibration',
        'kalibr_errorterms'
    ],
    package_dir={'': 'python'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Kalibr ROS2 - Visual-Inertial Calibration Package',
    license='BSD',
    entry_points={
        'console_scripts': [
            'kalibr_calibrate_imu_camera = kalibr_imu_camera.calibrate_imu_camera:main',
        ],
    },
)
