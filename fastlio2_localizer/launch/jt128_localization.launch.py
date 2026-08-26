from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    argument_defaults = {
        "lio_config": PathJoinSubstitution(
            [FindPackageShare("fastlio2"), "config", "jt128.yaml"]
        ),
        "localizer_config": PathJoinSubstitution(
            [FindPackageShare("fastlio2_localizer"), "config", "localizer.yaml"]
        ),
        "map_path": "",
        "initialize_from_parameters": "false",
        "enable_rviz": "true",
        "rviz_config": PathJoinSubstitution(
            [FindPackageShare("fastlio2_localizer"), "rviz", "localizer.rviz"]
        ),
        "use_sim_time": "false",
        "publish_tf": "true",
        "initial_x": "0.0",
        "initial_y": "0.0",
        "initial_z": "0.0",
        "initial_roll": "0.0",
        "initial_pitch": "0.0",
        "initial_yaw": "0.0",
    }
    descriptions = {
        "lio_config": "Hesai JT-128 sensor and estimator YAML",
        "localizer_config": "ICP localizer YAML",
        "map_path": "Prior map PCD to preload",
        "initialize_from_parameters": "Queue the initial_* pose after loading the map",
        "enable_rviz": "Start RViz2",
        "rviz_config": "RViz2 configuration file",
        "use_sim_time": "Use /clock instead of wall time",
        "publish_tf": "Publish fastlio_odom -> base_footprint TF",
        "initial_x": "Initial map-frame x in metres",
        "initial_y": "Initial map-frame y in metres",
        "initial_z": "Initial map-frame z in metres",
        "initial_roll": "Initial roll in radians",
        "initial_pitch": "Initial pitch in radians",
        "initial_yaw": "Initial yaw in radians",
    }

    return LaunchDescription(
        [
            *[
                DeclareLaunchArgument(
                    name,
                    default_value=default,
                    description=descriptions[name],
                )
                for name, default in argument_defaults.items()
            ],
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("fastlio2_localizer"),
                            "launch",
                            "localizer_launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    name: LaunchConfiguration(name)
                    for name in argument_defaults
                }.items(),
            )
        ]
    )
