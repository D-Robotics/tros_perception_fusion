English| [简体中文](./README_cn.md)

# Feature Introduction

The perception fusion node subscribes to multiple [hobot_msgs/ai_msgs/msg/PerceptionTargets](https://github.com/D-Robotics/hobot_msgs/blob/develop/ai_msgs/msg/PerceptionTargets.msg) type topics, aligns them in time, merges and deduplicates all the data in the messages, and then publishes using a [hobot_msgs/ai_msgs/msg/PerceptionTargets](https://github.com/D-Robotics/hobot_msgs/blob/develop/ai_msgs/msg/PerceptionTargets.msg) type topic.

Supports dynamic querying and modifying of topics used for fusion via a service interface.

# Installation Method

```bash
sudo apt update
sudo apt install -y tros-tros-perception-fusion
```

# Usage

## Launching the Node with Launch

```bash
ros2 launch tros_perception_fusion perc_fusion.launch.py topic_name_base:=tros_perc topic_names_fusion:='[tros_perc_1, tros_perc_2]' pub_fusion_topic_name:=tros_perc_fusion
```

After startup, the node subscribes to 3 [hobot_msgs/ai_msgs/msg/PerceptionTargets](https://github.com/D-Robotics/hobot_msgs/blob/develop/ai_msgs/msg/PerceptionTargets.msg) type topic messages, namely `tros_perc`, `tros_perc_1`, `tros_perc_2`, and publishes `tros_perc_fusion` topic messages after successful fusion.

## Launching the Node with ros2 run

```bash
ros2 run tros_perception_fusion tros_perception_fusion --ros-args -p topic_name_base:=tros_perc -p "topic_names_fusion:=['tros_perc_1', 'tros_perc_2']" -p pub_fusion_topic_name:=tros_perc_fusion
```

## Launching with Composition

Taking the launch method as an example, add the node to the composition:

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

# Interface Description

## Topics

| Name                  | Message Type       | Description                              |
| ---------------------- | ----------------- | ---------------------------------------- |
| /tros_perc_fusion     | [hobot_msgs/ai_msgs/msg/PerceptionTargets](https://github.com/D-Robotics/hobot_msgs/blob/develop/ai_msgs/msg/PerceptionTargets.msg)     | Published fused information |
| /tros_perc            | [hobot_msgs/ai_msgs/msg/PerceptionTargets](https://github.com/D-Robotics/hobot_msgs/blob/develop/ai_msgs/msg/PerceptionTargets.msg)   | Subscribed pre-fusion information |

## Parameters

| Parameter Name        | Type           | Explanation                                  | Required | Supported Configurations | Default Value                              |
| ---------------------- | -------------- | -------------------------------------------- | -------- | ----------------------- | ---------------------------------------- |
| topic_name_base       | std::string    | Base topic name for subscription used in fusion | No       | Configure according to actual deployment environment | /tros_perc |
| topic_names_fusion    | std::string array | Topic names for subscription to be fused | No       | Configure according to actual deployment environment | Empty |
| pub_fusion_topic_name | std::string    | Topic name for publishing fused information   | No       | Configure according to actual deployment environment | /tros_perc_fusion     |
| enable_filter         | bool           | Whether to perform deduplication filtering on messages | No       | True/False | True     |

> [!NOTE]
> If `topic_names_fusion` is configured as empty, the published message content is the same as the topic configured in `topic_name_base`.
> If any of the topics configured in `topic_name_base` or `topic_names_fusion` does not exist, it will cause the message fusion to fail, and no messages will be published.

# Dynamically Modifying Subscription Topics

Dynamically modify the topics used for fusion via service requests. The service uses the message type [TopicManage.srv](./tros_perception_fusion_msgs/srv/TopicManage.srv).

1. Query topics used for fusion:

```bash
ros2 service call /tros_topic_manage tros_perception_fusion_msgs/srv/TopicManage "{action: 'get'}"
```

2. Add topics for fusion:

```bash
ros2 service call /tros_topic_manage tros_perception_fusion_msgs/srv/TopicManage "{action: 'add', topics: {'/tros_det', '/tros_seg'}}"
```

3. Delete topics for fusion:

```bash
ros2 service call /tros_topic_manage tros_perception_fusion_msgs/srv/TopicManage "{action: 'delete', topics: {'/tros_det'}}"
```
