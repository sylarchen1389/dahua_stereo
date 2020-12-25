# 大华 双目 驱动节点


## 简介
---
使用大华摄像头A5501M/CG20 开发库编写的ros node，实现硬触发同时调用两个摄像头，同步取帧

参数可以在launch文件中设置

include包含所有头文件

硬触发配置
- LINE1 信号源接入
- 需要信号地接入
- 使用上升沿触发
- 低电平为 1.5~0 高电平为 2.2 ~ 24 

pub一共需要20~35ms，sub需要6ms


## 依赖
---
- ROS Melodic
- 大华摄像机SDK
- opencv >= 3.2.0


## 配置及使用
---
### 运行节点

1. 使用catkin_make 编译
```
$catkin_make --pkg dahua_cam 
```

2. 运行节点
```
$roslaunch dahua_stereo dahua.launch
```

4. 检测节点消息
```
$rostopic list 
/camera/leftRaw
....
/camera/rightRaw
....
/rosout
/rosout_agg
```

5. 订阅节点
订阅 "/camera/leftRaw"或"/camera/rightRaw"节点

6. 关闭节点
Ctrl+C

### 修改预处理

可以直接修改launch 文件中的参数,

可修改的有 roi, 白平衡, 曝光,伽马

```
 <rosparam>
        roiWidth: 2592
        roiHeight: 1038
        roiX: 0
        roiY: 600
        nWidth: 2592
        nHeight: 2048
        exposureTime: 10000.0
        balanceRatoioRed: 1.5
        balanceRatoioGreen: 1.0
        balanceRatoioBlue: 1.5
        dGamma: 0.6 
    </rosparam>
```


## 开发日志
---
[2020.9.11]
- 编写完毕并上传