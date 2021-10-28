# -----------------------------------------------------------------------------
# Copyright 2021 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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
#
#

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    """Create composable node."""
    bag_prefix = LaunchConfig('bag_prefix')
    container = ComposableNodeContainer(
            name='composable_recorder_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                #
                # add further ComposableNode elements here, e.g. the driver.
                # make sure to set use_intra_process_comms to True
                #
                ComposableNode(
                    package='rosbag2_composable_recorder',
                    plugin='rosbag2_composable_recorder::ComposableRecorder',
                    name="recorder",
                    # set topics etc here
                    parameters=[{'topics': ['/your_topic_here'],
                                 'storage_id': 'sqlite3',
                                 'record_all': False,
                                 'disable_discovery': False,
                                 'serialization_format': 'cdr',
                                 'start_recording_immediately': False,
                                 'bag_prefix': bag_prefix}],
                    remappings=[],
                    extra_arguments=[{'use_intra_process_comms': True}],
                )
            ],
            output='screen',
    )
    return [container]


def generate_launch_description():
    """Create composable node by calling opaque function.
    This creates a context such that string arguments can be extracted
    """
    return launch.LaunchDescription([
        LaunchArg('bag_prefix', default_value=['rosbag2_'],
                  description='prefix of rosbag'),
        OpaqueFunction(function=launch_setup)
        ])
