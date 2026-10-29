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


def test_generic_launch_preserves_override_defaults_and_forwards_parameters():
    module = _load_launch("lio_launch.py", "fastlio2_lio_launch")
    declarations = _declarations(module.generate_launch_description())

    assert _default(declarations["world_frame_override"]) == ""
    assert _default(declarations["allow_missing_point_time"]) == "false"

    parameters = module._lio_node_parameters(
        "config.yaml", "false", "true", "odom", True
    )
    assert parameters["world_frame_override"] == "odom"
    assert isinstance(parameters["allow_missing_point_time"], ParameterValue)
    assert parameters["allow_missing_point_time"].value_type is bool


def test_jt128_wrapper_declares_and_forwards_runtime_overrides():
    module = _load_launch("jt128_lio.launch.py", "fastlio2_jt128_lio_launch")
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
        "allow_missing_point_time": "false",
    }.items():
        assert _default(declarations[name]) == default
        assert name in forwarded
