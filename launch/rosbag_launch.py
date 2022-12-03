#  Copyright 2022 Adarsh M.

#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at

#      http://www.apache.org/licenses/LICENSE-2.0

#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    
    queue_size = LaunchConfiguration('queue_size')
    pub_freq = LaunchConfiguration('pub_freq')
    record_all_topics = LaunchConfiguration('record_all_topics')

    return LaunchDescription([

        DeclareLaunchArgument(
            'queue_size',
            default_value='10.0'
        ),
        
        DeclareLaunchArgument(
            'pub_freq',
            default_value='2.0'
        ),

        DeclareLaunchArgument(
            'record_all_topics',
            default_value='True'
        ),

        Node(
            package='beginner_tutorials',
            executable='talker',
            name='talker',
            parameters=[{
                "queue_size": LaunchConfiguration('queue_size'),
                "pub_freq": LaunchConfiguration('pub_freq'),
            }]
        ),

        ExecuteProcess(
        condition=IfCondition(record_all_topics),
        cmd=[
            'ros2', 'bag', 'record', '-o all_topics_bag', '-a'
        ],
        shell=True
        )

    ])