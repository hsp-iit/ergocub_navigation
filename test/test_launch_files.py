"""
Structural checks over every launch file in the package.

These catch the failure mode this package kept hitting: a launch file that
references another launch file, a params YAML or a map that is not there. Nothing
here starts a node, so it is safe to run in CI without a robot or a simulator.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import (
    get_launch_description_from_python_launch_file,
)
from launch.utilities import (
    normalize_to_list_of_substitutions,
    perform_substitutions,
)
import pytest

SHARE = get_package_share_directory('ergocub_navigation')
LAUNCH_DIR = os.path.join(SHARE, 'launch')

# Top-level entry points. These are additionally exercised through their
# OpaqueFunctions, which is where profile selection happens.
ENTRY_POINTS = [
    ('bringup.launch.py', {}),
    ('bringup.launch.py', {'world': 'sim', 'localization': 'slam'}),
    ('bringup.launch.py', {'world': 'sim', 'localization': 'amcl'}),
    ('bringup.launch.py', {'world': 'robot', 'localization': 'odom_only'}),
    ('bringup.launch.py', {'world': 'sim', 'localization': 'odom_only'}),
    ('bringup.launch.py', {'world': 'robot', 'localization': 'none'}),
    ('launch_all.launch.py', {}),
    ('launch_all_slam.launch.py', {}),
    ('launch_all_odom_only.launch.py', {}),
    ('launch_sim.launch.py', {}),
    ('launch_slam_sim.launch.py', {}),
    ('launch_all_odom_only_sim.launch.py', {}),
]


def all_launch_files():
    for root, _, files in os.walk(LAUNCH_DIR):
        for name in sorted(files):
            if name.endswith('.launch.py'):
                yield os.path.join(root, name)


def find_launch_file(basename):
    for path in all_launch_files():
        if os.path.basename(path) == basename:
            return path
    raise FileNotFoundError(basename)


def _context(argv=None):
    ctx = LaunchContext(argv=argv or [])
    return ctx


def _walk(entity, context, seen_includes):
    """Recurse through an entity, resolving includes and running OpaqueFunctions."""
    if isinstance(entity, LaunchDescription):
        for sub in entity.entities:
            _walk(sub, context, seen_includes)
        return

    if isinstance(entity, DeclareLaunchArgument):
        entity.visit(context)
        return

    if isinstance(entity, IncludeLaunchDescription):
        source = entity.launch_description_source
        # Recurse in a child context so the parent's arguments do not leak.
        child = _context()
        # get_launch_description() both resolves the path and expands
        # source.location from its substitutions; it raises if the file is absent.
        child_ld = source.get_launch_description(child)
        location = source.location
        seen_includes.append(location)
        assert os.path.isfile(location), f'included launch file missing: {location}'

        # Pass the caller's arguments down, and check any that name a file.
        for name_subs, value_subs in entity.launch_arguments:
            name = perform_substitutions(
                context, normalize_to_list_of_substitutions(name_subs))
            value = perform_substitutions(
                context, normalize_to_list_of_substitutions(value_subs))
            child.launch_configurations[name] = value
            if os.path.isabs(value) and os.path.splitext(value)[1]:
                assert os.path.exists(value), (
                    f'{os.path.basename(location)} was passed {name}:={value}, '
                    'which does not exist')

        _walk(child_ld, child, seen_includes)
        return

    if isinstance(entity, OpaqueFunction):
        for sub in entity.execute(context) or []:
            _walk(sub, context, seen_includes)
        return

    if isinstance(entity, GroupAction):
        # Recurse into both branches regardless of condition, to cover them all.
        for sub in entity.get_sub_entities():
            _walk(sub, context, seen_includes)
        return

    # Nodes, ExecuteProcess, event handlers: constructing them was the check.


@pytest.mark.parametrize('path', list(all_launch_files()),
                         ids=lambda p: os.path.relpath(p, LAUNCH_DIR))
def test_launch_file_constructs(path):
    """Every launch file imports and builds a LaunchDescription."""
    assert isinstance(get_launch_description_from_python_launch_file(path),
                      LaunchDescription)


@pytest.mark.parametrize('basename,args', ENTRY_POINTS,
                         ids=lambda v: str(v) if not isinstance(v, str) else v)
def test_entry_point_graph_resolves(basename, args):
    """Entry points resolve their whole include graph, with no missing targets."""
    context = _context([f'{k}:={v}' for k, v in args.items()])
    ld = get_launch_description_from_python_launch_file(find_launch_file(basename))

    # Apply the caller-supplied arguments the way `ros2 launch` would.
    for key, value in args.items():
        context.launch_configurations[key] = value

    includes = []
    _walk(ld, context, includes)
    assert includes, f'{basename} resolved no includes'


def test_referenced_config_files_exist():
    """Every param/map/rviz/behaviour-tree path baked into a launch file exists."""
    import re

    missing = []
    pattern = re.compile(r"pkg_share\(\s*((?:'[^']*'\s*,\s*)*'[^']*')\s*\)")
    for path in all_launch_files():
        with open(path) as fh:
            source = fh.read()
        for match in pattern.finditer(source):
            parts = [p.strip().strip("'") for p in match.group(1).split(',')]
            if parts[0] == 'launch':  # include() targets, checked above
                continue
            target = os.path.join(SHARE, *parts)
            if not os.path.exists(target):
                missing.append(f'{os.path.relpath(path, LAUNCH_DIR)} -> {target}')
    assert not missing, 'launch files reference missing files:\n' + '\n'.join(missing)


def test_launch_basenames_are_unique():
    """`ros2 launch <pkg> <file>` matches on basename only, so they must not clash."""
    seen = {}
    for path in all_launch_files():
        name = os.path.basename(path)
        seen.setdefault(name, []).append(path)
    clashes = {k: v for k, v in seen.items() if len(v) > 1}
    assert not clashes, f'duplicate launch file basenames: {clashes}'
