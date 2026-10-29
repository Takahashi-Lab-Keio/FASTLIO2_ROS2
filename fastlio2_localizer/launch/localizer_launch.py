from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _nav_override(nav2_mode, enabled, disabled):
    if isinstance(nav2_mode, str):
        return enabled if nav2_mode.lower() == "true" else disabled
    return PythonExpression([
        "'", str(enabled), "' if '", nav2_mode,
        "'.lower() == 'true' else '", disabled, "'",
    ])


def _bool_parameter(value):
    if isinstance(value, str):
        if value.lower() not in ("true", "false"):
            raise ValueError("Expected true or false")
        value = value.lower() == "true"
    return ParameterValue(value, value_type=bool)


def _lio_node_parameters(
    config_path,
    use_sim_time,
    publish_tf,
    world_frame_override,
    allow_missing_point_time,
    nav2_mode="false",
):
    return {
        "config_path": config_path,
        "use_sim_time": use_sim_time,
        "publish_tf": _bool_parameter(_nav_override(nav2_mode, "false", publish_tf)),
        "enable_imu_prediction": _bool_parameter(nav2_mode),
        "world_frame_override": _nav_override(nav2_mode, "fastlio_odom", world_frame_override),
        "body_frame_override": _nav_override(nav2_mode, "fastlio_body", ""),
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
    nav2_mode="false",
):
    return {
        "config_path": config_path,
        "map_path": map_path,
        "initialize_from_parameters": ParameterValue(
            initialize_from_parameters, value_type=bool
        ),
        "use_sim_time": use_sim_time,
        "local_frame_override": _nav_override(nav2_mode, "fastlio_odom", local_frame_override),
        "body_frame_override": _nav_override(nav2_mode, "fastlio_body", ""),
        "publish_tf": _bool_parameter(_nav_override(nav2_mode, "false", "true")),
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

    nav2_mode = LaunchConfiguration("nav2_mode")
    nav2_planar = LaunchConfiguration("nav2_planar")

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
        DeclareLaunchArgument("nav2_mode", default_value="false",
                              description="Use IMU prediction and the independent Nav2 TF bridge"),
        DeclareLaunchArgument("nav2_planar", default_value="true",
                              description="Project Nav2 TF to x/y/yaw with z=0; preserve full 3D outputs"),
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
                        nav2_mode,
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
                        nav2_mode,
                    )
                ],
            ),
            Node(
                package="fastlio2",
                namespace="fastlio2",
                executable="nav_tf_node",
                name="nav_tf_node",
                output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "planar": ParameterValue(nav2_planar, value_type=bool),
                }],
                condition=IfCondition(nav2_mode),
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
