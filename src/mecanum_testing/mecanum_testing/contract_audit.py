#!/usr/bin/env python3
"""Fast, deterministic checks for repository-level architecture contracts."""

import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
from typing import Any

import yaml


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _record(checks: list[dict[str, Any]], name: str, passed: bool, detail: str) -> None:
    checks.append({'name': name, 'passed': passed, 'detail': detail})


def audit(workspace: Path) -> dict[str, Any]:
    checks: list[dict[str, Any]] = []
    src = workspace / 'src'
    nav = src / 'mecanum_navigation'
    sim = src / 'mecanum_simulation'

    profiles_path = nav / 'profiles/environments.yaml'
    profiles = yaml.safe_load(profiles_path.read_text())['environments']
    for name, profile in profiles.items():
        world = sim / 'worlds' / profile['world_file']
        map_yaml = nav / 'maps' / profile['map_file']
        _record(checks, f'environment.{name}.world_exists', world.is_file(), str(world))
        if world.is_file():
            _record(
                checks,
                f'environment.{name}.imu_system_present',
                'gazebo::systems::Imu' in world.read_text(),
                str(world),
            )
        _record(checks, f'environment.{name}.map_exists', map_yaml.is_file(), str(map_yaml))
        if map_yaml.is_file():
            image_name = yaml.safe_load(map_yaml.read_text())['image']
            image = map_yaml.parent / image_name
            _record(checks, f'environment.{name}.map_image_exists', image.is_file(), str(image))

    robot_xacro = src / 'mecanum_robot_description/urdf/mecanum_robot.xacro'
    xacro_text = robot_xacro.read_text()
    properties = {
        match.group(1): float(match.group(2))
        for match in re.finditer(
            r'<xacro:property name="(body_length|body_width|wheel_width)"\s+value="([0-9.]+)"',
            xacro_text,
        )
    }
    expected_lx = properties['body_length'] / 2.0 - 0.10
    expected_ly = properties['body_width'] / 2.0 + properties['wheel_width'] + 0.1
    controller_path = src / 'mecanum_control/config/mecanum_controllers.yaml'
    controller = yaml.safe_load(controller_path.read_text())
    configured_sum = controller['mecanum_drive_controller']['ros__parameters']['kinematics'][
        'sum_of_robot_center_projection_on_X_Y_axis'
    ]
    expected_sum = expected_lx + expected_ly
    _record(
        checks,
        'controller.geometry_matches_urdf',
        abs(configured_sum - expected_sum) < 1e-9,
        f'configured={configured_sum}, urdf_lx_plus_ly={expected_sum}',
    )

    launch_path = nav / 'launch/navigation.launch.py'
    launch_text = launch_path.read_text()
    behavior_block = launch_text.split('behavior_server = Node(', 1)[1].split(
        'bt_navigator = Node(', 1
    )[0]
    _record(
        checks,
        'command_chain.behaviors_enter_nav_mux',
        "remappings=[('cmd_vel', 'cmd_vel_nav')]" in behavior_block,
        'behavior_server cmd_vel must not publish directly to final cmd_vel',
    )

    bridge_path = sim / 'config/gz_bridge.yaml'
    bridge = yaml.safe_load(bridge_path.read_text())
    bridged_ros_topics = {entry['ros_topic_name'] for entry in bridge}
    _record(
        checks,
        'ground_truth.bridge_is_explicit',
        '/ground_truth/odom' in bridged_ros_topics,
        str(bridge_path),
    )
    forbidden_consumers = []
    for path in src.rglob('*'):
        if not path.is_file() or path.suffix not in {'.yaml', '.py'} or path == bridge_path:
            continue
        if 'mecanum_testing' in path.parts:
            continue
        if 'ground_truth/odom' in path.read_text(errors='ignore'):
            forbidden_consumers.append(str(path.relative_to(workspace)))
    _record(
        checks,
        'ground_truth.not_in_production_config',
        not forbidden_consumers,
        f'forbidden_references={forbidden_consumers}',
    )

    forbidden_plugins = []
    for path in (src / 'mecanum_robot_description').rglob('*.xacro'):
        text = path.read_text()
        if re.search(r'<plugin\b[^>]*(?:MecanumDrive|mecanum-drive)', text, re.IGNORECASE):
            forbidden_plugins.append(str(path.relative_to(workspace)))
    _record(
        checks,
        'gazebo.no_legacy_mecanum_drive_plugin',
        not forbidden_plugins,
        f'matches={forbidden_plugins}',
    )

    hashed_inputs = [
        profiles_path,
        controller_path,
        launch_path,
        robot_xacro,
        bridge_path,
        nav / 'params/nav2_params.yaml',
    ]
    hashed_inputs.extend(sim / 'worlds' / profile['world_file'] for profile in profiles.values())
    failed = [check['name'] for check in checks if not check['passed']]
    return {
        'schema_version': 1,
        'result_state': 'PASS_STATIC' if not failed else 'FAIL',
        'failed_checks': failed,
        'checks': checks,
        'input_sha256': {str(path.relative_to(workspace)): _sha256(path) for path in hashed_inputs},
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--workspace', type=Path, default=Path.cwd())
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = audit(args.workspace.resolve())
    rendered = json.dumps(result, indent=2, sort_keys=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + '\n')
    print(rendered)
    sys.exit(0 if result['result_state'] == 'PASS_STATIC' else 1)


if __name__ == '__main__':
    main()
