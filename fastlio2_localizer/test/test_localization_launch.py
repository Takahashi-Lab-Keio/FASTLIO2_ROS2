import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _load_launch(relative_path, module_name):
    path = PACKAGE_ROOT / "launch" / relative_path
    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _declarations(description):
    return {
        action.name: action
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }


def _default(action):
    return perform_substitutions(LaunchContext(), action.default_value)


def test_generic_launch_preserves_defaults_and_forwards_both_frames():
    module = _load_launch(
        "localizer_launch.py", "fastlio2_generic_localization_launch"
    )
    declarations = _declarations(module.generate_launch_description())

    for name, default in {
        "world_frame_override": "",
        "local_frame_override": "",
        "allow_missing_point_time": "false",
    }.items():
        assert _default(declarations[name]) == default

    lio_parameters = module._lio_node_parameters(
        "lio.yaml", "false", "true", "odom", True
    )
    assert lio_parameters["world_frame_override"] == "odom"
    assert isinstance(
        lio_parameters["allow_missing_point_time"], ParameterValue
    )
    assert lio_parameters["allow_missing_point_time"].value_type is bool

    localizer_parameters = module._localizer_node_parameters(
        "localizer.yaml", "map.pcd", False, "false", "odom", {}
    )
    assert localizer_parameters["local_frame_override"] == "odom"


def test_jt128_wrapper_declares_and_forwards_runtime_overrides():
    module = _load_launch(
        "jt128_localization.launch.py", "fastlio2_jt128_localization_launch"
    )
    description = module.generate_launch_description()
    declarations = _declarations(description)
    include = next(
        action
        for action in description.entities
        if isinstance(action, IncludeLaunchDescription)
    )
    forwarded = dict(include.launch_arguments)

    for name, default in {
        "world_frame_override": "",
        "local_frame_override": "",
        "allow_missing_point_time": "false",
    }.items():
        assert _default(declarations[name]) == default
        assert name in forwarded
