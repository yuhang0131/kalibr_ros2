from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Declare arguments
    bagfile_arg = DeclareLaunchArgument(
        'bagfile',
        description='Path to ROS2 bag file containing camera and IMU data'
    )
    
    target_yaml_arg = DeclareLaunchArgument(
        'target_yaml',
        default_value=PathJoinSubstitution([
            FindPackageShare('kalibr_imu_camera'),
            'config',
            'target_april_example.yaml'
        ]),
        description='Path to calibration target YAML file'
    )
    
    cam_yaml_arg = DeclareLaunchArgument(
        'cam_yaml',
        default_value=PathJoinSubstitution([
            FindPackageShare('kalibr_imu_camera'),
            'config',
            'camchain_example.yaml'
        ]),
        description='Path to camera chain YAML file'
    )
    
    imu_yaml_arg = DeclareLaunchArgument(
        'imu_yaml',
        default_value=PathJoinSubstitution([
            FindPackageShare('kalibr_imu_camera'),
            'config',
            'imu_example.yaml'
        ]),
        description='Path to IMU YAML file'
    )
    
    imu_model_arg = DeclareLaunchArgument(
        'imu_model',
        default_value='calibrated',
        description='IMU model type: calibrated, scale-misalignment, scale-misalignment-size-effect'
    )
    
    dont_show_report_arg = DeclareLaunchArgument(
        'dont_show_report',
        default_value='False',
        description='Do not show calibration report plots'
    )
    
    # Get the package share directory
    pkg_share = FindPackageShare('kalibr_imu_camera').find('kalibr_imu_camera')
    executable_path = os.path.join(os.path.dirname(os.path.dirname(pkg_share)), 
                                    'lib', 'kalibr_imu_camera', 'kalibr_calibrate_imu_camera')
    
    # Calibration process (using ExecuteProcess instead of Node to avoid ROS2 arguments)
    # Note: --show-extraction disables multithreading to avoid copy.copy() issues with C++ detector objects
    calibration_process = ExecuteProcess(
        cmd=[
            executable_path,
            '--bag', LaunchConfiguration('bagfile'),
            '--target', LaunchConfiguration('target_yaml'),
            '--cams', LaunchConfiguration('cam_yaml'),
            '--imu', LaunchConfiguration('imu_yaml'),
            '--imu-models', LaunchConfiguration('imu_model'),
            '--dont-show-report',
            '--show-extraction',
        ],
        output='screen'
    )

    return LaunchDescription([
        bagfile_arg,
        target_yaml_arg,
        cam_yaml_arg,
        imu_yaml_arg,
        imu_model_arg,
        dont_show_report_arg,
        calibration_process
    ])
