[English](./README.md) | 简体中文

# 功能介绍

感知融合节点订阅多个[hobot_msgs/ai_msgs/msg/PerceptionTargets](https://github.com/D-Robotics/hobot_msgs/blob/develop/ai_msgs/msg/PerceptionTargets.msg)类型的topic，经过时间对齐后，将所有消息中的数据合并、去重后，再使用一个[hobot_msgs/ai_msgs/msg/PerceptionTargets](https://github.com/D-Robotics/hobot_msgs/blob/develop/ai_msgs/msg/PerceptionTargets.msg)类型的topic发布。

支持使用service接口动态查询和修改用于融合的topic。

# 安装方法

```bash
sudo apt update
sudo apt install -y tros-tros-perception-fusion
```

# 使用方法

## 使用launch启动节点

```bash
ros2 launch tros_perception_fusion perc_fusion.launch.py topic_name_base:=tros_perc topic_names_fusion:='[tros_perc_1, tros_perc_2]' pub_fusion_topic_name:=tros_perc_fusion
```

启动后，节点订阅3个[hobot_msgs/ai_msgs/msg/PerceptionTargets](https://github.com/D-Robotics/hobot_msgs/blob/develop/ai_msgs/msg/PerceptionTargets.msg)类型的话题消息，分别是`tros_perc`, `tros_perc_1`, `tros_perc_2`，融合成功后发布`tros_perc_fusion`话题消息。


## 使用ros2 run启动节点

```bash
ros2 run tros_perception_fusion tros_perception_fusion --ros-args -p topic_name_base:=tros_perc -p "topic_names_fusion:=['tros_perc_1', 'tros_perc_2']" -p pub_fusion_topic_name:=tros_perc_fusion
```

## 使用composition方式启动

以launch启动方式举例，将节点添加到composition中：

```python
ComposableNode(
    package='tros_perception_fusion',
    plugin='tros::TrosAiMsgFusionNode',
    name='tros_perc_fusion_node',
    parameters=[
        {'topic_name_base': 'tros_perc'},
        {'topic_names_fusion': ['tros_perc_1', 'tros_perc_2']},
        {'pub_fusion_topic_name': 'tros_perc_fusion'},
    ],
    extra_arguments=[{'use_intra_process_comms': True}],
)
```

# 接口说明

## 话题

| 名称                 | 消息类型        | 说明|
| ---------------------- | ----------- |---------------------------- |
| /tros_perc_fusion     | [hobot_msgs/ai_msgs/msg/PerceptionTargets](https://github.com/D-Robotics/hobot_msgs/blob/develop/ai_msgs/msg/PerceptionTargets.msg)     | 发布的融合后的信息 |
| /tros_perc          | [hobot_msgs/ai_msgs/msg/PerceptionTargets](https://github.com/D-Robotics/hobot_msgs/blob/develop/ai_msgs/msg/PerceptionTargets.msg)   | 订阅的融合前的信息 |

## 参数

| 参数名                 | 类型        | 解释                                        | 是否必须 | 支持的配置           | 默认值                        |
| ---------------------- | ----------- | ------------------------------------------- | -------- | -------------------- | ----------------------------- |
| topic_name_base | std::string | 订阅的用于融合的基础信息topic名 | 否       | 根据实际部署环境配置 | /tros_perc |
| topic_names_fusion | std::string数组 | 订阅的被融合的信息topic名 | 否       | 根据实际部署环境配置 | 空 |
| pub_fusion_topic_name | std::string | 发布的的融合后的信息topic名   | 否       | 根据实际部署环境配置 | /tros_perc_fusion     |
| enable_filter | bool | 是否对消息进行去重过滤   | 否       | True/False | True     |

> [!NOTE]
> 如果`topic_names_fusion`配置为空，发布的消息内容和`topic_name_base`配置中的话题内容相同。
> 如果`topic_name_base`或`topic_names_fusion`中配置的任何一个话题不存在，将会导致消息融合失败，无消息发布。

# 动态修改订阅topic

通过service请求动态修改用于融合的topic，service使用的消息类型为[TopicManage.srv](./tros_perception_fusion_msgs/srv/TopicManage.srv)。

1. 查询用于融合的topic：

```bash
ros2 service call /tros_topic_manage tros_perception_fusion_msgs/srv/TopicManage "{action: 'get'}"
```

2. 增加用于融合的topic：

```bash
ros2 service call /tros_topic_manage tros_perception_fusion_msgs/srv/TopicManage "{action: 'add', topics: {'/tros_det', '/tros_seg'}}"
```

3. 删除用于融合的topic：

```bash
ros2 service call /tros_topic_manage tros_perception_fusion_msgs/srv/TopicManage "{action: 'delete', topics: {'/tros_det'}}"
```
