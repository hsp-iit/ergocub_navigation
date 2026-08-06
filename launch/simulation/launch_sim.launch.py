"""
Simulation, AMCL localization.

Thin wrapper around bringup.launch.py; see that file for the full argument list.
"""

from ergocub_navigation.launch_utils import include
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        include('bringup.launch.py', {
            'world': 'sim',
            'localization': 'amcl',
            'use_planner_trigger': 'true',
        }),
    ])
