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

# The number of nodes per topic is evenly distributed. For example, if `num_topics` is 3 and
# `num_nodes` is 8, the first and second topics will have 3 nodes each, and the third topic will
# have 2.
num_topics = 1
num_nodes = 4 * num_topics

use_multithreaded_executor = True

# Thread count parameters take effect only when `use_multithreaded_executor` is
# set to True. A value of 0 indicates the default, which is half the number of hardware
# threads (i.e., std::thread::hardware_concurrency() / 2).
ros2_thread_count = 0
agnocast_thread_count = 0

timer_interval_ms = 10

# Prevent the process form printing the ROS 2 parameters it received. This is useful when launching
# many processes.
quiet = False
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
                    'num_nodes': num_nodes,
                    'use_multithreaded_executor': use_multithreaded_executor,
                    'ros2_thread_count': ros2_thread_count,
                    'agnocast_thread_count': agnocast_thread_count,
                    'timer_interval_ms': timer_interval_ms,
                    'quiet': quiet,
                }],
                additional_env={
                    'LD_PRELOAD': prepend_libagnocast_heaphook(os.environ.get('LD_PRELOAD', '')),
                    'AGNOCAST_MEMPOOL_SIZE': '67108864',  # 64 MB
                },
            ),
        ])
    elif composition_pattern == 1:
        first_half_num_nodes = num_nodes // num_topics + 1
        second_half_num_nodes = num_nodes // num_topics
        first_half_count = num_nodes % num_topics
        return LaunchDescription([
            Node(
                package="agnocast_sample_application",
                executable="talker",
                output="screen",
                parameters=[{
                    'starting_topic_id': topic_id,
                    'num_topics': 1,
                    'num_nodes': (
                        first_half_num_nodes
                        if topic_id < first_half_count
                        else second_half_num_nodes
                    ),
                    'use_multithreaded_executor': use_multithreaded_executor,
                    'ros2_thread_count': ros2_thread_count,
                    'agnocast_thread_count': agnocast_thread_count,
                    'timer_interval_ms': timer_interval_ms,
                    'quiet': quiet,
                }],
                additional_env={
                    'LD_PRELOAD': prepend_libagnocast_heaphook(os.environ.get('LD_PRELOAD', '')),
                    'AGNOCAST_MEMPOOL_SIZE': '67108864',  # 64 MB
                },
            )
            for topic_id in range(0, num_topics)
        ])
    elif composition_pattern == 2:
        return LaunchDescription([
            Node(
                package="agnocast_sample_application",
                executable="talker",
                output="screen",
                parameters=[{
                    'starting_topic_id': i % num_topics,
                    'num_topics': 1,
                    'num_nodes': 1,
                    'use_multithreaded_executor': use_multithreaded_executor,
                    'ros2_thread_count': ros2_thread_count,
                    'agnocast_thread_count': agnocast_thread_count,
                    'timer_interval_ms': timer_interval_ms,
                    'quiet': quiet,
                }],
                additional_env={
                    'LD_PRELOAD': prepend_libagnocast_heaphook(os.environ.get('LD_PRELOAD', '')),
                    'AGNOCAST_MEMPOOL_SIZE': '67108864',  # 64 MB
                },
            )
            for i in range(0, num_nodes)
        ])
    else:
        raise Exception('Invalid composition pattern')
