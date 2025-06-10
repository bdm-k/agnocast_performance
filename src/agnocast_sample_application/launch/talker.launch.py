import os
from launch import LaunchDescription
from launch_ros.actions import Node


##############
# parameters #
##############

# 0: All publisher nodes are placed in one process, and all subscriber nodes are
#    placed in the other process.
# 1: Publisher nodes for the same topic are placed in the same process. The same
#    applies to subscriber nodes.
# 2: Each process only has one node.
composition_pattern = 0

# The number of publishers per topic is fixed at MAX_PUBLISHER_NUM (=4)
# The number of subscribers per topic is fixed at MAX_SUBSCRIBER_NUM (=16)
num_topics = 1

use_multithreaded_executor = True

timer_interval_ms = 100
##############


def prepend_libagnocast_heaphook(s):
    return "/opt/ros/humble/lib/libagnocast_heaphook.so:" + s

def generate_launch_description():
    if composition_pattern == 0:
        return LaunchDescription([
            Node(
                package="agnocast_sample_application",
                executable="talker",
                output="screen",
                parameters=[{
                    'starting_topic_id': 0,
                    'num_topics': num_topics,
                    'use_multithreaded_executor': use_multithreaded_executor,
                    'timer_interval_ms': timer_interval_ms,
                }],
                additional_env={
                    'LD_PRELOAD': prepend_libagnocast_heaphook(os.environ.get('LD_PRELOAD', '')),
                    'AGNOCAST_MEMPOOL_SIZE': '67108864',  # 64 MB
                },
            ),
        ])
    elif composition_pattern == 1:
        return LaunchDescription([
            Node(
                package="agnocast_sample_application",
                executable="talker",
                output="screen",
                parameters=[{
                    'starting_topic_id': topic_id,
                    'num_topics': 1,
                    'use_multithreaded_executor': use_multithreaded_executor,
                    'timer_interval_ms': timer_interval_ms,
                }],
                additional_env={
                    'LD_PRELOAD': prepend_libagnocast_heaphook(os.environ.get('LD_PRELOAD', '')),
                    'AGNOCAST_MEMPOOL_SIZE': '67108864',  # 64 MB
                },
            )
            for topic_id in range(0, num_topics)
        ])
    elif composition_pattern == 2:
        raise Exception('Unsupported composition pattern')
    else:
        raise Exception('Invalid composition pattern')
