# Copyright (c) 2024，D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction

def generate_launch_description():
    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='warn')
    perc_sub_topic_arg = DeclareLaunchArgument(
        'topic_name_base',
        default_value='/tros_perc',
        description='subscribed base msg topic')
    topic_names_fusion_arg = DeclareLaunchArgument(
        'topic_names_fusion',
        default_value='',
        # default_value='[/tros_perc_fusion_1, /tros_perc_fusion_2]',
        description='subscribed base msg topic')
    perc_pub_topic_arg = DeclareLaunchArgument(
        'pub_fusion_topic_name',
        default_value='/tros_perc_fusion',
        description='published msg topic')
    enable_filter_arg = DeclareLaunchArgument(
        'enable_filter',
        default_value='True',
        description='enable filter')

    perc_fusion_node = Node(
        package='tros_perception_fusion',
        executable='tros_perception_fusion',
        name='tros_perc_fusion_node',
        output='screen',
        parameters=[
                    {'topic_name_base': LaunchConfiguration("topic_name_base")},
                    {'topic_names_fusion': LaunchConfiguration("topic_names_fusion")},
                    {'pub_fusion_topic_name': LaunchConfiguration("pub_fusion_topic_name")},
                    {'enable_filter': LaunchConfiguration("enable_filter")}
        ],
       arguments=['--ros-args', '--log-level', LaunchConfiguration("log_level")]
    )

    ld = LaunchDescription()
    ld.add_action(log_level_arg)
    ld.add_action(perc_sub_topic_arg)
    ld.add_action(topic_names_fusion_arg)
    ld.add_action(perc_pub_topic_arg)
    ld.add_action(enable_filter_arg)
    ld.add_action(perc_fusion_node)
    return ld
