import os
from launch import LaunchDescription
from launch_ros.actions import Node


def prepend_libagnocast_heaphook(s):
  return "/opt/ros/humble/lib/libagnocast_heaphook.so:" + s

def generate_launch_description():
  return LaunchDescription([
    Node(
      package="agnocast_sample_application",
      executable="talker",
      output="screen",
      parameters=[{
        'starting_topic_id': 0,
        'num_topics': 1,
        'use_multithreaded_executor': True,
        'timer_interval_ms': 100,
      }],
      additional_env={
        'LD_PRELOAD': prepend_libagnocast_heaphook(os.environ.get('LD_PRELOAD', '')),
        'AGNOCAST_MEMPOOL_SIZE': '67108864',  # 64 MB
      },
    ),
  ])
