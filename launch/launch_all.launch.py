"""
Robot, AMCL localization.

setup_robot + setup_localization (map + amcl) + nav2 with the keepout filters +
plane_detector + pointcloud_filter + planner_trigger_server + RViz, matching the
pre-refactor launch_all.launch.py.

This is bringup.launch.py's default profile, so the wrapper passes nothing; see
that file for the full argument list.
"""

from ergocub_navigation.launch_utils import include
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        include('bringup.launch.py', {
            'world': 'robot',
            'localization': 'amcl',
        }),
    ])
