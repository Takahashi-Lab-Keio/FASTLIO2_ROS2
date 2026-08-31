from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _lio_node_parameters(
    config_path,
    use_sim_time,
    publish_tf,
    world_frame_override,
    allow_missing_point_time,
):
    return {
        "config_path": config_path,
        "use_sim_time": use_sim_time,
        "publish_tf": publish_tf,
        "world_frame_override": world_frame_override,
        "allow_missing_point_time": ParameterValue(
            allow_missing_point_time, value_type=bool
        ),
    }


def _localizer_node_parameters(
    config_path,
    map_path,
    initialize_from_parameters,
    use_sim_time,
    local_frame_override,
    initial_values,
):
    return {
        "config_path": config_path,
        "map_path": map_path,
        "initialize_from_parameters": ParameterValue(
            initialize_from_parameters, value_type=bool
        ),
        "use_sim_time": use_sim_time,
        "local_frame_override": local_frame_override,
        **initial_values,
    }


def generate_launch_description():
    default_lio_config = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "lio.yaml"]
    )
    default_localizer_config = PathJoinSubstitution(
        [FindPackageShare("fastlio2_localizer"), "config", "localizer.yaml"]
    )
    default_rviz = PathJoinSubstitution(
        [FindPackageShare("fastlio2_localizer"), "rviz", "localizer.rviz"]
    )

    lio_config = LaunchConfiguration("lio_config")
    localizer_config = LaunchConfiguration("localizer_config")
    map_path = LaunchConfiguration("map_path")
    initialize_from_parameters = LaunchConfiguration(
        "initialize_from_parameters"
    )
    enable_rviz = LaunchConfiguration("enable_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_tf = LaunchConfiguration("publish_tf")
    world_frame_override = LaunchConfiguration("world_frame_override")
    local_frame_override = LaunchConfiguration("local_frame_override")
    allow_missing_point_time = LaunchConfiguration("allow_missing_point_time")

    initial_names = [
        "initial_x",
        "initial_y",
        "initial_z",
        "initial_roll",
        "initial_pitch",
        "initial_yaw",
    ]
    initial_values = {
        name: ParameterValue(LaunchConfiguration(name), value_type=float)
        for name in initial_names
    }

    declarations = [
        DeclareLaunchArgument(
            "lio_config",
            default_value=default_lio_config,
            description="FAST-LIO2 sensor and estimator YAML",
        ),
        DeclareLaunchArgument(
            "localizer_config",
            default_value=default_localizer_config,
            description="ICP localizer YAML",
        ),
        DeclareLaunchArgument(
            "map_path",
            default_value="",
            description="Prior map PCD to preload",
        ),
        DeclareLaunchArgument(
            "initialize_from_parameters",
            default_value="false",
            description="Queue the initial_* pose immediately after loading the map",
        ),
        DeclareLaunchArgument(
            "enable_rviz", default_value="true", description="Start RViz2"
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz,
            description="RViz2 configuration file",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock instead of wall time",
        ),
        DeclareLaunchArgument(
            "publish_tf",
            default_value="true",
            description="Publish fastlio_odom -> base_footprint TF",
        ),
        DeclareLaunchArgument(
            "world_frame_override",
            default_value="",
            description="Override the LIO YAML world_frame when non-empty",
        ),
        DeclareLaunchArgument(
            "local_frame_override",
            default_value="",
            description="Override the localizer YAML local_frame when non-empty",
        ),
        DeclareLaunchArgument(
            "allow_missing_point_time",
            default_value="false",
            description=(
                "Allow simulation PointCloud2 without a per-point time field"
            ),
        ),
    ]
    declarations.extend(
        DeclareLaunchArgument(name, default_value="0.0")
        for name in initial_names
    )

    return LaunchDescription(
        declarations
        + [
            Node(
                package="fastlio2",
                namespace="fastlio2",
                executable="lio_node",
                name="lio_node",
                output="screen",
                parameters=[
                    _lio_node_parameters(
                        lio_config,
                        use_sim_time,
                        publish_tf,
                        world_frame_override,
                        allow_missing_point_time,
                    )
                ],
            ),
            Node(
                package="fastlio2_localizer",
                namespace="fastlio2/localizer",
                executable="localizer_node",
                name="localizer_node",
                output="screen",
                parameters=[
                    _localizer_node_parameters(
                        localizer_config,
                        map_path,
                        initialize_from_parameters,
                        use_sim_time,
                        local_frame_override,
                        initial_values,
                    )
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="fastlio2_localization_rviz",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(enable_rviz),
            ),
        ]
    )
