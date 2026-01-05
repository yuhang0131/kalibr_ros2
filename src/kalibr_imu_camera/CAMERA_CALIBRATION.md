# Kalibr ROS2 多相机标定使用说明

## 概述

本文档说明如何使用kalibr_ros2进行多相机系统的标定。

## 依赖项

确保已安装以下Python库：
```bash
pip install python-igraph
```

## 相机模型

支持以下相机模型：
- `pinhole-radtan`: 针孔相机 + 径向-切向畸变 (最常用)
- `pinhole-equi`: 针孔相机 + 等距畸变 (鱼眼相机)
- `pinhole-fov`: 针孔相机 + FOV畸变模型
- `omni-none`: 全向相机 (无畸变)
- `omni-radtan`: 全向相机 + 径向-切向畸变
- `eucm-none`: 扩展统一相机模型 (无畸变)
- `ds-none`: 双球面相机模型 (无畸变)

## 准备标定板配置文件

创建一个YAML文件描述您的标定板，例如 `aprilgrid.yaml`：

### Aprilgrid 标定板
```yaml
target_type: 'aprilgrid'
tagCols: 6      # 列数
tagRows: 6      # 行数
tagSize: 0.088  # 标签大小 [米]
tagSpacing: 0.3 # 标签间距（占tagSize的百分比）
```

### 棋盘格标定板
```yaml
target_type: 'checkerboard'
targetCols: 8   # 内部角点列数
targetRows: 6   # 内部角点行数
rowSpacingMeters: 0.025  # 行间距 [米]
colSpacingMeters: 0.025  # 列间距 [米]
```

## 录制标定数据

使用ROS2录制包含相机图像的bag文件：

```bash
ros2 bag record /cam0/image_raw /cam1/image_raw -o camera_calibration_bag
```

**录制建议**:
- 覆盖整个图像区域
- 从不同角度和距离拍摄标定板
- 至少50-100帧有效图像
- 保持运动平缓，避免模糊

## 单相机标定

标定单个相机的内参：

```bash
# 使用命令行
ros2 run kalibr_imu_camera kalibr_calibrate_cameras \
  --bag camera_calibration_bag \
  --topics /cam0/image_raw \
  --models pinhole-radtan \
  --target aprilgrid.yaml \
  --show-extraction

# 或使用launch文件
ros2 launch kalibr_imu_camera calibrate_cameras.launch.py \
  bagfile:=camera_calibration_bag \
  topics:='/cam0/image_raw' \
  models:='pinhole-radtan' \
  target_yaml:=aprilgrid.yaml
```

## 多相机标定

标定多相机系统的内参和外参（相机之间的相对位置）：

```bash
# 双相机系统
ros2 run kalibr_imu_camera kalibr_calibrate_cameras \
  --bag camera_calibration_bag \
  --topics /cam0/image_raw /cam1/image_raw \
  --models pinhole-radtan pinhole-radtan \
  --target aprilgrid.yaml \
  --approx-sync 0.02 \
  --show-extraction

# 三相机系统
ros2 run kalibr_imu_camera kalibr_calibrate_cameras \
  --bag camera_calibration_bag \
  --topics /cam0/image_raw /cam1/image_raw /cam2/image_raw \
  --models pinhole-radtan pinhole-radtan pinhole-fov \
  --target aprilgrid.yaml \
  --approx-sync 0.02 \
  --show-extraction
```

## 重要参数说明

### 必需参数
- `--bag`: ROS2 bag文件路径
- `--topics`: 图像话题列表（空格分隔）
- `--models`: 每个相机的模型（空格分隔，顺序与topics对应）
- `--target`: 标定板配置YAML文件

### 图像同步参数
- `--approx-sync`: 近似时间同步容差 [秒]（默认：0.02）
  - 多相机系统中用于匹配时间戳接近的图像
  - 如果相机硬件同步，可以设置更小的值（如0.005）
  - 如果相机不同步，可能需要增大（如0.05）

### 标定器设置
- `--qr-tol`: QR分解的容差（默认：0.02）
  - 控制几何信息的质量要求
  - 较小值：选择几何更好的图像
- `--mi-tol`: 互信息容差（默认：0.2）
  - 控制添加新图像的阈值
  - 较大值：使用更少但质量更好的图像
  - -1：强制使用所有图像

### 异常值过滤选项
- `--no-outliers-removal`: 禁用角点异常值过滤
- `--no-final-filtering`: 禁用所有视图处理后的过滤
- `--min-views-outlier`: 初始化统计的原始视图数量（默认：20）
- `--use-blakezisserman`: 启用Blake-Zisserman M-估计器（更鲁棒）
- `--plot-outliers`: 在提取期间绘制检测到的异常值

### 输出选项
- `--show-extraction`: 显示标定板提取过程（禁用多线程，推荐使用）
- `--verbose`: 启用详细输出
- `--plot`: 标定期间显示图表
- `--dont-show-report`: 标定后不显示报告
- `--export-poses`: 导出优化后的位姿到CSV文件

## 输出文件

标定完成后会生成以下文件：

1. **camchain-TIMESTAMP.yaml**: 相机链标定结果
   - 包含每个相机的内参、畸变参数
   - 包含相机之间的外参（T_cn_cnm1变换矩阵）
   - 可直接用于IMU-相机标定

2. **report-cam-TIMESTAMP.pdf**: 标定报告
   - 重投影误差统计
   - 标定板检测可视化
   - 相机参数估计的不确定性

3. **results-cam-TIMESTAMP.txt**: 详细结果文本文件

## 典型工作流程

### 1. 准备阶段
```bash
# 创建标定板配置文件
vim aprilgrid.yaml
```

### 2. 数据采集
```bash
# 录制bag
ros2 bag record /cam0/image_raw /cam1/image_raw -o my_camera_bag
```

### 3. 标定
```bash
# 运行标定
ros2 run kalibr_imu_camera kalibr_calibrate_cameras \
  --bag my_camera_bag \
  --topics /cam0/image_raw /cam1/image_raw \
  --models pinhole-radtan pinhole-radtan \
  --target aprilgrid.yaml \
  --show-extraction
```

### 4. 检查结果
- 查看PDF报告，确认重投影误差合理（通常<1像素）
- 检查camchain.yaml中的参数是否合理
- 如果结果不满意，调整参数重新标定或重新采集数据

### 5. 用于IMU-相机标定
```bash
# 使用生成的camchain.yaml进行IMU-相机标定
ros2 run kalibr_imu_camera kalibr_calibrate_imu_camera \
  --bag imu_camera_bag \
  --cam camchain-TIMESTAMP.yaml \
  --imu imu.yaml \
  --target aprilgrid.yaml
```

## 常见问题

### Q: 标定失败，找不到标定板
**A**: 
- 确保光照条件良好
- 检查标定板配置文件是否正确
- 尝试使用`--show-extraction`查看检测过程
- 减慢录制速度，避免运动模糊

### Q: 重投影误差太大
**A**:
- 重新采集数据，确保覆盖整个图像区域
- 增加`--mi-tol`值，只使用质量最好的图像
- 检查标定板的物理尺寸是否准确
- 启用`--use-blakezisserman`使用更鲁棒的估计器

### Q: 多相机同步问题
**A**:
- 增大`--approx-sync`值（如0.05）
- 确保相机的时间戳来自同一时钟源
- 考虑使用硬件同步

### Q: 程序运行很慢
**A**:
- 使用`--show-extraction`禁用多线程（numpy_eigen在多线程下有问题）
- 减少图像频率：`--bag-freq 4`
- 增大`--mi-tol`减少使用的图像数量

## 参考链接

- [Kalibr Wiki](https://github.com/ethz-asl/kalibr/wiki)
- [相机模型详解](https://github.com/ethz-asl/kalibr/wiki/supported-models)
- [标定板制作](https://github.com/ethz-asl/kalibr/wiki/calibration-targets)
