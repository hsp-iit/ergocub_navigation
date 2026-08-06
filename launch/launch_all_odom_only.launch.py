"""
Robot, odometry only: blank map plus an identity map->odom transform.

Thin wrapper around bringup.launch.py; see that file for the full argument list.
"""

from ergocub_navigation.launch_utils import include
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        include('bringup.launch.py', {
            'world': 'robot',
            'localization': 'odom_only',
            'use_planner_trigger': 'false',
        }),
    ])
