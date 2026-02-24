# 车道线检测系统

MATLAB 实现的车道线检测程序，支持图像和视频处理。

## 功能

- **图像检测**：单张图片车道线识别
- **视频检测**：实时/离线视频车道线跟踪
- **卡尔曼滤波**：可选滤波优化检测结果
- **GUI 界面**：图形化操作界面

## 文件说明

| 文件 | 功能 |
|------|------|
| `img_lane_detect.m` | 图像车道线检测 |
| `video_lane_detect.m` | 视频车道线检测 |
| `lane_detection_gui.m` | 图形界面程序 |

## 使用方法

### 图像检测
```matlab
img_lane_detect('image.jpg')
```

### 视频检测
```matlab
video_lane_detect('video.mp4')
```

### GUI 界面
```matlab
lane_detection_gui()
```

## 算法流程

1. ROI 提取（底部 40% 区域）
2. 灰度化 + 二值化
3. 边缘检测（Sobel/Canny/Otsu）
4. 霍夫变换检测直线
5. 几何约束筛选车道线
6. 卡尔曼滤波（可选）

## 参数配置

在 GUI 界面中可调节：
- 过程噪声 Q
- 测量噪声 R
- 边缘检测方法
- 霍夫变换参数

## 依赖

- MATLAB R2018b 或更高版本
- Computer Vision Toolbox
- Image Processing Toolbox
