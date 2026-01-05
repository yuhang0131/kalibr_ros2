import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Declare launch arguments
    bagfile_arg = DeclareLaunchArgument(
        'bagfile',
        description='Path to ROS2 bag file containing camera and IMU data'
    )
    
    camchain_arg = DeclareLaunchArgument(
        'camchain',
        description='Path to camera chain YAML file (output from kalibr_calibrate_cameras)'
    )
    
    imu0_arg = DeclareLaunchArgument(
        'imu0',
        default_value=[
            FindPackageShare('kalibr_imu_camera'), 
            '/share/kalibr_imu_camera/config/',
            'imu0_example.yaml'
        ],
        description='Path to IMU0 configuration YAML (reference IMU)'
    )
    
    imu1_arg = DeclareLaunchArgument(
        'imu1',
        default_value=[
            FindPackageShare('kalibr_imu_camera'), 
            '/share/kalibr_imu_camera/config/',
            'imu1_example.yaml'
        ],
        description='Path to IMU1 configuration YAML (second IMU)'
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
    
    imu_models_arg = DeclareLaunchArgument(
        'imu_models',
        default_value='calibrated calibrated',
        description='IMU models for each IMU (space separated): calibrated, scale-misalignment, or scale-misalignment-size-effect'
    )
    
    estimate_imu_delay_arg = DeclareLaunchArgument(
        'estimate_imu_delay',
        default_value='True',
        description='Estimate delay between IMUs by correlation'
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
    
    # Calibration process (using ExecuteProcess to avoid ROS2 arguments)
    calibration_process = ExecuteProcess(
        cmd=[
            executable_path,
            '--bag', LaunchConfiguration('bagfile'),
            '--cams', LaunchConfiguration('camchain'),
            '--imu', LaunchConfiguration('imu0'), LaunchConfiguration('imu1'),
            '--imu-models', LaunchConfiguration('imu_models'),
            '--imu-delay-by-correlation',
            '--target', LaunchConfiguration('target_yaml'),
            '--dont-show-report',
            '--show-extraction',
        ],
        output='screen',
        shell=True
    )

    return LaunchDescription([
        bagfile_arg,
        camchain_arg,
        imu0_arg,
        imu1_arg,
        target_yaml_arg,
        imu_models_arg,
        estimate_imu_delay_arg,
        dont_show_report_arg,
        calibration_process
    ])
