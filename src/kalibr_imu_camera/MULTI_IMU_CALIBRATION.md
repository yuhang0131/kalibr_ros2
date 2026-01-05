# Kalibr ROS2 多IMU标定使用说明

## 概述

kalibr_ros2已完全支持多IMU标定功能，可以同时标定多个IMU与相机系统之间的空间和时间关系。多IMU标定对于高精度导航系统、多传感器融合等应用非常有用。

## 功能特性

- ✅ 支持多个IMU同时标定
- ✅ 第一个IMU作为参考IMU（参考坐标系）
- ✅ 估计多个IMU之间的时间延迟（通过相关性）
- ✅ 支持三种IMU模型：
  - `calibrated`: 已标定的IMU（使用厂商提供的参数）
  - `scale-misalignment`: 尺度-失准模型（标定尺度因子和轴失准）
  - `scale-misalignment-size-effect`: 尺度-失准-尺寸效应模型（额外标定尺寸效应）
- ✅ 自动查找多个IMU之间的方向先验
- ✅ 输出包含所有IMU的标定结果

## 多IMU标定流程

### 1. 准备工作

#### 1.1 准备相机标定结果
首先完成相机系统标定，获得camchain.yaml文件：
```bash
ros2 run kalibr_imu_camera kalibr_calibrate_cameras \
  --bag camera_bag \
  --topics /cam0/image_raw /cam1/image_raw \
  --models pinhole-radtan pinhole-radtan \
  --target aprilgrid.yaml \
  --show-extraction
```

#### 1.2 准备每个IMU的配置文件
为每个IMU创建独立的YAML配置文件。

**imu0.yaml** (第一个IMU，作为参考IMU):
```yaml
#Accelerometer
accelerometer_noise_density: 0.006   # [m/s^2/sqrt(Hz)]
accelerometer_random_walk: 0.0002    # [m/s^3/sqrt(Hz)]

#Gyroscope  
gyroscope_noise_density: 0.0004      # [rad/s/sqrt(Hz)]
gyroscope_random_walk: 4.0e-06       # [rad/s^2/sqrt(Hz)]

#IMU update rate
update_rate: 200.0                   # [Hz]

#IMU ROS topic
rostopic: /imu0/data                 # ROS话题名称

#IMU time offset (initial guess, will be optimized)
time_offset: 0.0                     # [s]
```

**imu1.yaml** (第二个IMU):
```yaml
#Accelerometer
accelerometer_noise_density: 0.008   # [m/s^2/sqrt(Hz)]
accelerometer_random_walk: 0.0003    # [m/s^3/sqrt(Hz)]

#Gyroscope
gyroscope_noise_density: 0.0005      # [rad/s/sqrt(Hz)]
gyroscope_random_walk: 5.0e-06       # [rad/s^2/sqrt(Hz)]

#IMU update rate
update_rate: 200.0                   # [Hz]

#IMU ROS topic
rostopic: /imu1/data                 # ROS话题名称

#IMU time offset (initial guess, will be optimized)
time_offset: 0.0                     # [s]
```

#### 1.3 录制数据
录制包含所有相机和IMU数据的ROS2 bag文件：
```bash
ros2 bag record \
  /cam0/image_raw /cam1/image_raw \
  /imu0/data /imu1/data \
  -o multi_imu_camera_bag
```

**数据采集建议**:
- 进行充分激励的运动（6自由度运动）
- 包含旋转、平移、加速和减速
- 避免过快的运动导致图像模糊
- 录制时长至少60秒
- 保持标定板在视野中
- 覆盖不同的姿态和位置

### 2. 多IMU标定命令

#### 2.1 基本用法（双IMU，已标定模型）
```bash
ros2 run kalibr_imu_camera kalibr_calibrate_imu_camera \
  --bag multi_imu_camera_bag \
  --cams camchain.yaml \
  --imu imu0.yaml imu1.yaml \
  --imu-models calibrated calibrated \
  --target aprilgrid.yaml \
  --show-extraction
```

#### 2.2 启用IMU间时间延迟估计
```bash
ros2 run kalibr_imu_camera kalibr_calibrate_imu_camera \
  --bag multi_imu_camera_bag \
  --cams camchain.yaml \
  --imu imu0.yaml imu1.yaml \
  --imu-models calibrated calibrated \
  --imu-delay-by-correlation \
  --target aprilgrid.yaml \
  --show-extraction
```

#### 2.3 三IMU系统标定
```bash
ros2 run kalibr_imu_camera kalibr_calibrate_imu_camera \
  --bag multi_imu_camera_bag \
  --cams camchain.yaml \
  --imu imu0.yaml imu1.yaml imu2.yaml \
  --imu-models calibrated calibrated calibrated \
  --imu-delay-by-correlation \
  --target aprilgrid.yaml \
  --show-extraction
```

#### 2.4 使用不同IMU模型
```bash
# IMU0: 已标定模型（使用厂商参数）
# IMU1: 尺度-失准模型（在线标定尺度和失准）
ros2 run kalibr_imu_camera kalibr_calibrate_imu_camera \
  --bag multi_imu_camera_bag \
  --cams camchain.yaml \
  --imu imu0.yaml imu1.yaml \
  --imu-models calibrated scale-misalignment \
  --imu-delay-by-correlation \
  --target aprilgrid.yaml \
  --show-extraction
```

### 3. 重要参数说明

#### 3.1 必需参数
- `--bag`: ROS2 bag文件路径
- `--cams`: 相机链标定结果（camchain.yaml）
- `--imu`: IMU配置文件列表（空格分隔，**第一个IMU为参考IMU**）
- `--imu-models`: 每个IMU的模型类型（与--imu对应）
- `--target`: 标定板配置文件

#### 3.2 IMU模型类型
- `calibrated`: 
  - 使用厂商提供的标定参数
  - 只估计IMU与相机的外参（位置和姿态）
  - 适用于高质量、已标定的IMU
  
- `scale-misalignment`:
  - 标定尺度因子和轴失准
  - 适用于需要在线标定的中低端IMU
  - 增加12个自由度（加速度计和陀螺仪各6个）
  
- `scale-misalignment-size-effect`:
  - 除尺度和失准外，还标定尺寸效应
  - 适用于高精度要求的应用
  - 自由度最多，需要更多激励运动

#### 3.3 多IMU特殊参数
- `--imu-delay-by-correlation`:
  - 通过相关性估计多个IMU之间的时间延迟
  - **强烈推荐对非硬件同步的多IMU系统使用**
  - 对硬件同步的系统可以不使用

#### 3.4 其他重要参数
- `--no-time-calibration`: 禁用相机-IMU时间标定
- `--max-iter`: 最大迭代次数（默认：30）
- `--recover-covariance`: 恢复协方差矩阵
- `--timeoffset-padding`: 时间偏移允许的最大变化范围 [s]（默认：0.03）

### 4. 输出文件

多IMU标定完成后会生成：

#### 4.1 camchain-imucam.yaml
包含相机链和所有IMU的标定结果：
```yaml
cam0:
  T_cam_imu:  # cam0到imu0的变换
  - [...
  - [...
  - [...
  - [0.0, 0.0, 0.0, 1.0]
  timeshift_cam_imu: -0.001234  # 相机到IMU的时间偏移
  ...

imu0:  # 参考IMU
  T_i_b:  # IMU0到body frame的变换（通常为单位矩阵）
  - [1.0, 0.0, 0.0, 0.0]
  - [0.0, 1.0, 0.0, 0.0]
  - [0.0, 0.0, 1.0, 0.0]
  - [0.0, 0.0, 0.0, 1.0]
  accelerometer_noise_density: 0.006
  accelerometer_random_walk: 0.0002
  gyroscope_noise_density: 0.0004
  gyroscope_random_walk: 4.0e-06
  rostopic: /imu0/data
  update_rate: 200.0
  
imu1:  # 第二个IMU
  T_i_b:  # IMU1到body frame的变换
  - [0.9998, -0.0123, 0.0045, 0.05]
  - [0.0124, 0.9999, -0.0023, 0.02]
  - [-0.0044, 0.0024, 0.9999, -0.01]
  - [0.0, 0.0, 0.0, 1.0]
  time_offset: 0.002345  # 相对于IMU0的时间偏移
  accelerometer_noise_density: 0.008
  accelerometer_random_walk: 0.0003
  gyroscope_noise_density: 0.0005
  gyroscope_random_walk: 5.0e-06
  rostopic: /imu1/data
  update_rate: 200.0
```

#### 4.2 imu.yaml
包含所有IMU的独立配置（用于单独使用）

#### 4.3 results-imucam.txt
详细的标定结果文本文件，包含：
- 优化前后的误差统计
- 所有参数的估计值和标准差
- 重投影误差、IMU误差等

#### 4.4 report-imucam.pdf
可视化报告，包含：
- 误差分布图
- 轨迹可视化
- IMU测量残差
- 相机重投影误差

### 5. 结果评估

#### 5.1 检查重投影误差
```
Camera-IMU found 1234 foverlaps 
Reprojection error:
  cam0: mean = 0.245 px, std = 0.123 px
  cam1: mean = 0.267 px, std = 0.145 px
```
- 平均重投影误差应 < 1.0 像素
- 标准差应 < 0.5 像素

#### 5.2 检查IMU误差
```
Accelerometer error:
  imu0: mean = 0.012 m/s^2, std = 0.008 m/s^2
  imu1: mean = 0.015 m/s^2, std = 0.010 m/s^2

Gyroscope error:
  imu0: mean = 0.0012 rad/s, std = 0.0008 rad/s
  imu1: mean = 0.0015 rad/s, std = 0.0010 rad/s
```
- 误差应与noise density参数在同一数量级

#### 5.3 检查多IMU相对姿态
查看camchain-imucam.yaml中imu1的T_i_b矩阵：
- 旋转部分应合理（通常小角度）
- 平移部分应与物理安装位置一致

#### 5.4 检查时间偏移
```
Time offset (imu0 to cam0): -0.001234 s
Time offset (imu1 to imu0):  0.002345 s
```
- 绝对值通常 < 0.05 秒
- 如果过大，检查数据采集同步

### 6. 使用Launch文件

创建自定义launch文件简化多IMU标定：

**calibrate_multi_imu.launch.py**:
```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    bagfile_arg = DeclareLaunchArgument(
        'bagfile',
        description='Path to ROS2 bag file'
    )
    
    camchain_arg = DeclareLaunchArgument(
        'camchain',
        description='Path to camera chain YAML file'
    )
    
    imu0_arg = DeclareLaunchArgument(
        'imu0',
        description='Path to IMU0 configuration YAML'
    )
    
    imu1_arg = DeclareLaunchArgument(
        'imu1',
        description='Path to IMU1 configuration YAML'
    )
    
    target_yaml_arg = DeclareLaunchArgument(
        'target_yaml',
        description='Path to calibration target YAML file'
    )
    
    # Get executable path
    pkg_share = FindPackageShare('kalibr_imu_camera').find('kalibr_imu_camera')
    executable_path = os.path.join(os.path.dirname(os.path.dirname(pkg_share)), 
                                    'lib', 'kalibr_imu_camera', 'kalibr_calibrate_imu_camera')
    
    # Calibration process
    calibration_process = ExecuteProcess(
        cmd=[
            executable_path,
            '--bag', LaunchConfiguration('bagfile'),
            '--cams', LaunchConfiguration('camchain'),
            '--imu', LaunchConfiguration('imu0'), LaunchConfiguration('imu1'),
            '--imu-models', 'calibrated', 'calibrated',
            '--imu-delay-by-correlation',
            '--target', LaunchConfiguration('target_yaml'),
            '--show-extraction',
            '--dont-show-report',
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
        calibration_process
    ])
```

使用launch文件：
```bash
ros2 launch kalibr_imu_camera calibrate_multi_imu.launch.py \
  bagfile:=multi_imu_bag \
  camchain:=camchain.yaml \
  imu0:=imu0.yaml \
  imu1:=imu1.yaml \
  target_yaml:=aprilgrid.yaml
```

### 7. 常见问题

#### Q1: 多个IMU的顺序重要吗？
**A**: 是的，**第一个IMU（imu0）会被作为参考IMU**。所有其他IMU的位姿都是相对于这个参考IMU的。建议选择：
- 最接近相机的IMU
- 精度最高的IMU
- 更新频率最高的IMU

#### Q2: 必须启用--imu-delay-by-correlation吗？
**A**: 取决于您的硬件配置：
- **硬件同步**的多IMU系统：不需要
- **非硬件同步**的系统：强烈建议启用
- 如果不确定，建议启用，算法会自动判断

#### Q3: 可以标定多少个IMU？
**A**: 理论上没有限制，但：
- 每增加一个IMU，优化问题变得更复杂
- 建议最多3-4个IMU
- 更多IMU需要更长的数据和更充分的激励

#### Q4: 标定失败或结果不好？
**A**: 检查以下几点：
1. 数据采集是否充分激励（6自由度运动）
2. IMU的noise参数是否合理
3. 相机标定结果是否准确
4. 标定板检测是否稳定
5. 尝试增加`--max-iter`参数
6. 尝试使用`--verbose`查看详细输出

#### Q5: 如何获取IMU noise参数？
**A**: 三种方法：
1. 查看IMU数据手册（datasheet）
2. 使用Allan方差分析工具
3. 使用保守估计值（通常偏大）

#### Q6: scale-misalignment模型何时使用？
**A**: 
- 低成本MEMS IMU
- 没有出厂标定的IMU
- 需要提高精度的应用
- 注意：需要更充分的激励运动

### 8. 典型工作流程总结

```bash
# 步骤1: 标定相机系统
ros2 run kalibr_imu_camera kalibr_calibrate_cameras \
  --bag camera_bag \
  --topics /cam0/image_raw /cam1/image_raw \
  --models pinhole-radtan pinhole-radtan \
  --target aprilgrid.yaml \
  --show-extraction

# 步骤2: 准备IMU配置文件
# 创建 imu0.yaml, imu1.yaml

# 步骤3: 录制多IMU-相机数据
ros2 bag record /cam0/image_raw /cam1/image_raw /imu0/data /imu1/data -o multi_imu_bag

# 步骤4: 多IMU标定
ros2 run kalibr_imu_camera kalibr_calibrate_imu_camera \
  --bag multi_imu_bag \
  --cams camchain-TIMESTAMP.yaml \
  --imu imu0.yaml imu1.yaml \
  --imu-models calibrated calibrated \
  --imu-delay-by-correlation \
  --target aprilgrid.yaml \
  --show-extraction

# 步骤5: 检查结果
# 查看 report-imucam.pdf 和 results-imucam.txt
# 确认重投影误差和IMU误差合理

# 步骤6: 使用标定结果
# 将 camchain-imucam.yaml 和 imu.yaml 用于SLAM/VIO等应用
```

### 9. 参考资料

- [Kalibr Wiki - IMU-Camera Calibration](https://github.com/ethz-asl/kalibr/wiki/IMU-camera-calibration)
- [Kalibr Wiki - YAML Formats](https://github.com/ethz-asl/kalibr/wiki/yaml-formats)
- [论文: A Robust and Modular Multi-Sensor Fusion Approach](https://www.research-collection.ethz.ch/handle/20.500.11850/155340)

## 10. 示例配置文件模板

参考 `/config` 目录下的示例文件：
- `imu0_example.yaml`: 参考IMU配置
- `imu1_example.yaml`: 第二个IMU配置
- `target_april_example.yaml`: Aprilgrid标定板配置
