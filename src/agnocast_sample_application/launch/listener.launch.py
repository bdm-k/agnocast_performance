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
##############


def prepend_libagnocast_heaphook(s):
    return "/opt/ros/humble/lib/libagnocast_heaphook.so:" + s

def generate_launch_description():
    if composition_pattern == 0:
        return LaunchDescription([
            Node(
                package="agnocast_sample_application",
                executable="listener",
                output="screen",
                parameters=[{
                    'starting_topic_id': 0,
                    'num_topics': num_topics,
                    'use_multithreaded_executor': use_multithreaded_executor,
                }],
                additional_env={
                    'LD_PRELOAD': prepend_libagnocast_heaphook(os.environ.get('LD_PRELOAD', '')),
                    'AGNOCAST_MEMPOOL_SIZE': '16777216',  # 16 MB
                },
            ),
        ])
    elif composition_pattern == 1:
        raise Exception('Unsupported composition pattern')
    elif composition_pattern == 2:
        raise Exception('Unsupported composition pattern')
    else:
        raise Exception('Invalid composition pattern')
