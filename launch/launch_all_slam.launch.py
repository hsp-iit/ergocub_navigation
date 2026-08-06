"""
Robot, slam_toolbox mapping.

The pre-refactor file did not start planner_trigger_server, plane_detector or the
pointcloud filter.

Thin wrapper around bringup.launch.py; see that file for the full argument list.
"""

from ergocub_navigation.launch_utils import include
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        include('bringup.launch.py', {
            'world': 'robot',
            'localization': 'slam',
            'use_planner_trigger': 'false',
        }),
    ])
