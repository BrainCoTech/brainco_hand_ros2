<div align="right">

[简体中文](README_CN.md)|[English](README.md)

</div>

<div align="center">

# BrainCo Revo2 灵巧手 ROS2 描述包使用说明书

</div>

## 目录
* 1.[revo2_description功能包说明](#revo2_description功能包说明)
* 2.[revo2_description功能包使用](#revo2_description功能包使用)
* 3.[revo2_description功能包架构说明](#revo2_description功能包架构说明)
* 3.1[功能包文件总览](#功能包文件总览)
* 4.[revo2_description话题说明](#revo2_description话题说明)

## revo2_description功能包说明
revo2_description功能包为显示BrainCo Revo2灵巧手机器人模型和TF变换的功能包，通过该功能包，我们可以实现电脑中的虚拟灵巧手与现实中的实际灵巧手的联动的效果，在之后的MoveIt2控制中我们也需要该功能包的支持。

* 1.功能包使用。
* 2.功能包架构说明。
* 3.功能包话题说明。

通过这三部分内容的介绍可以帮助大家：
* 1.了解该功能包的使用。
* 2.熟悉功能包中的文件构成及作用。
* 3.熟悉功能包相关的话题，方便开发和使用

## revo2_description功能包使用
首先配置好环境完成连接后我们可以通过以下命令直接启动节点，运行revo2_description功能包。

### 可视化启动
```bash
# 可视化左手（默认）
ros2 launch revo2_description view_revo2_left_hand.launch.py

# 可视化右手
ros2 launch revo2_description view_revo2_right_hand.launch.py
```

### Docker方式启动（推荐）
```bash
# 可视化左手（默认）
./scripts/visualize_revo2.sh left

# 可视化右手
./scripts/visualize_revo2.sh right
```

节点启动成功后，将显示RViz2可视化界面，可以看到BrainCo Revo2灵巧手的3D模型。

## revo2_description功能包架构说明

## 功能包文件总览
当前revo2_description功能包的文件构成如下。

```
├── CMakeLists.txt                #编译规则文件
├── launch/
│   ├── view_revo2_left_hand.launch.py     #左手可视化启动文件
│   └── view_revo2_right_hand.launch.py    #右手可视化启动文件
├── meshes/                       #模型文件存放文件夹
│   ├── revo2_left_hand/          #左手模型文件存放文件夹
│   │   ├── left_base_link.STL
│   │   ├── left_index_distal_link.STL
│   │   ├── left_index_proximal_link.STL
│   │   ├── left_index_tip_link.STL
│   │   ├── left_index_touch_link.STL
│   │   ├── left_middle_distal_link.STL
│   │   ├── left_middle_proximal_link.STL
│   │   ├── left_middle_tip_link.STL
│   │   ├── left_middle_touch_link.STL
│   │   ├── left_pinky_distal_link.STL
│   │   ├── left_pinky_proximal_link.STL
│   │   ├── left_pinky_tip_link.STL
│   │   ├── left_pinky_touch_link.STL
│   │   ├── left_ring_distal_link.STL
│   │   ├── left_ring_proximal_link.STL
│   │   ├── left_ring_tip_link.STL
│   │   ├── left_ring_touch_link.STL
│   │   ├── left_thumb_distal_link.STL
│   │   ├── left_thumb_metacarpal_link.STL
│   │   ├── left_thumb_proximal_link.STL
│   │   ├── left_thumb_proximal_visual_link.STL
│   │   ├── left_thumb_tip_link.STL
│   │   └── left_thumb_touch_link.STL
│   └── revo2_right_hand/         #右手模型文件存放文件夹
│       ├── right_base_link.STL
│       ├── right_index_distal_link.STL
│       ├── right_index_proximal_link.STL
│       ├── right_index_tip.STL
│       ├── right_index_touch_link.STL
│       ├── right_middle_distal_link.STL
│       ├── right_middle_proximal_link.STL
│       ├── right_middle_tip.STL
│       ├── right_middle_touch_link.STL
│       ├── right_pinky_distal_link.STL
│       ├── right_pinky_proximal_link.STL
│       ├── right_pinky_tip.STL
│       ├── right_pinky_touch_link.STL
│       ├── right_ring_distal_link.STL
│       ├── right_ring_proximal_link.STL
│       ├── right_ring_tip.STL
│       ├── right_ring_touch_link.STL
│       ├── right_thumb_distal_link.STL
│       ├── right_thumb_metacarpal_link.STL
│       ├── right_thumb_proximal_link.STL
│       ├── right_thumb_proximal_visual_link.STL
│       ├── right_thumb_tip.STL
│       └── right_thumb_touch_link.STL
├── scripts/                      #Docker工具脚本
│   └── visualize_revo2.sh       #可视化启动脚本
├── urdf/                         #URDF描述文件
│   ├── revo2_left_hand.urdf      #左手URDF描述文件
│   └── revo2_right_hand.urdf     #右手URDF描述文件
├── rviz/                         #RViz2配置文件
│   ├── revo2_left_hand.rviz      #左手RViz配置文件
│   └── revo2_right_hand.rviz     #右手RViz配置文件
├── .docker/                      #Docker支持文件
│   ├── Dockerfile                #Docker镜像构建文件
│   └── visualize_revo2.entrypoint.sh  #Docker启动脚本
├── package.xml                   #包配置文件
├── CHANGELOG.rst                 #版本变更日志
├── README.md                     #英文说明文档
└── README_CN.md                  #中文说明文档
```

## revo2_description话题说明
如下为该功能包的话题说明。

```
  Subscribers:
    /joint_states: sensor_msgs/msg/JointState
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /robot_description: std_msgs/msg/String
    /rosout: rcl_interfaces/msg/Log
    /tf: tf2_msgs/msg/TFMessage
    /tf_static: tf2_msgs/msg/TFMessage
  Service Servers:
    /robot_state_publisher/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /robot_state_publisher/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /robot_state_publisher/get_parameters: rcl_interfaces/srv/GetParameters
    /robot_state_publisher/list_parameters: rcl_interfaces/srv/ListParameters
    /robot_state_publisher/set_parameters: rcl_interfaces/srv/SetParameters
    /robot_state_publisher/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:
```

我们主要关注以下几个话题。
**Subscribers**:代表其订阅的话题，其中的`/joint_states`代表灵巧手当前的状态，joint_state_publisher_gui会发布该话题，这样RViz中的模型就会根据关节状态进行显示。

**Publishers**:代表其当前发布的话题，其最主要发布的话题为`/tf`和`/tf_static`，这两个话题描述了灵巧手关节与关节之间的坐标变换关系，也就是TF变换。

剩余话题和服务使用场景较少，大家可自行了解。

## 关节信息

### 左手关节

| 关节名称 | 描述 | 角度范围（度） | 角度范围（弧度） |
|------------|-------------|-----------------|-----------------|
| left_thumb_flex_joint | 拇指屈伸 | 0 ~ 59 | 0 ~ 1.03 |
| left_thumb_abduct_joint | 拇指外展 | 0 ~ 90 | 0 ~ 1.57 |
| left_index_joint | 食指 | 0 ~ 81 | 0 ~ 1.41 |
| left_middle_joint | 中指 | 0 ~ 81 | 0 ~ 1.41 |
| left_ring_joint | 无名指 | 0 ~ 81 | 0 ~ 1.41 |
| left_pinky_joint | 小指 | 0 ~ 81 | 0 ~ 1.41 |

### 右手关节

| 关节名称 | 描述 | 角度范围（度） | 角度范围（弧度） |
|------------|-------------|-----------------|-----------------|
| right_thumb_flex_joint | 拇指屈伸 | 0 ~ 59 | 0 ~ 1.03 |
| right_thumb_abduct_joint | 拇指外展 | 0 ~ 90 | 0 ~ 1.57 |
| right_index_joint | 食指 | 0 ~ 81 | 0 ~ 1.41 |
| right_middle_joint | 中指 | 0 ~ 81 | 0 ~ 1.41 |
| right_ring_joint | 无名指 | 0 ~ 81 | 0 ~ 1.41 |
| right_pinky_joint | 小指 | 0 ~ 81 | 0 ~ 1.41 |

## 许可证

本项目采用Apache License 2.0许可证