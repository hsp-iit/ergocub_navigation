"""
Simulation, odometry only.

    The pre-refactor file had its localization include commented out, so no map
    server and no map->odom transform: that is localization:=none here. RViz was
    only switched off as a side effect of that same commented-out include, so it
    is kept on.

Thin wrapper around bringup.launch.py; see that file for the full argument list.
"""

from ergocub_navigation.launch_utils import include
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        include('bringup.launch.py', {
            'world': 'sim',
            'localization': 'none',
            'use_planner_trigger': 'false',
        }),
    ])
