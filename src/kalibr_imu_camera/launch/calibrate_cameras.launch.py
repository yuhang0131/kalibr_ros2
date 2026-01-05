import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Declare launch arguments
    bagfile_arg = DeclareLaunchArgument(
        'bagfile',
        description='Path to ROS2 bag file containing camera images'
    )
    
    target_yaml_arg = DeclareLaunchArgument(
        'target_yaml',
        default_value=[
            FindPackageShare('kalibr_imu_camera'), 
            '/share/kalibr_imu_camera/config/',
            'target_april_example.yaml'
        ],
        description='Path to calibration target YAML file'
    )
    
    models_arg = DeclareLaunchArgument(
        'models',
        default_value='pinhole-radtan pinhole-radtan',
        description='Camera models for each camera (space separated)'
    )
    
    topics_arg = DeclareLaunchArgument(
        'topics',
        default_value='/cam0/image_raw /cam1/image_raw',
        description='Image topics for each camera (space separated)'
    )
    
    approx_sync_arg = DeclareLaunchArgument(
        'approx_sync',
        default_value='0.02',
        description='Time tolerance for approximate image synchronization [s]'
    )
    
    dont_show_report_arg = DeclareLaunchArgument(
        'dont_show_report',
        default_value='False',
        description='Do not show calibration report plots'
    )
    
    # Get the package share directory
    pkg_share = FindPackageShare('kalibr_imu_camera').find('kalibr_imu_camera')
    executable_path = os.path.join(os.path.dirname(os.path.dirname(pkg_share)), 
                                    'lib', 'kalibr_imu_camera', 'kalibr_calibrate_cameras')
    
    # Calibration process (using ExecuteProcess to avoid ROS2 arguments)
    # Note: --show-extraction disables multithreading to avoid copy.copy() issues with C++ detector objects
    calibration_process = ExecuteProcess(
        cmd=[
            executable_path,
            '--bag', LaunchConfiguration('bagfile'),
            '--target', LaunchConfiguration('target_yaml'),
            '--models', LaunchConfiguration('models'),
            '--topics', LaunchConfiguration('topics'),
            '--approx-sync', LaunchConfiguration('approx_sync'),
            '--dont-show-report',
            '--show-extraction',
        ],
        output='screen',
        shell=True
    )

    return LaunchDescription([
        bagfile_arg,
        target_yaml_arg,
        models_arg,
        topics_arg,
        approx_sync_arg,
        dont_show_report_arg,
        calibration_process
    ])
