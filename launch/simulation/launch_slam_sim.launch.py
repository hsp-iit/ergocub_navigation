"""
Simulation, slam_toolbox mapping.

Thin wrapper around bringup.launch.py; see that file for the full argument list.
"""

from ergocub_navigation.launch_utils import include, sim_pointcloud_filter
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        include('bringup.launch.py', {
            'world': 'sim',
            'localization': 'slam',
            'use_planner_trigger': 'true',
        }),
        *sim_pointcloud_filter(),
    ])
