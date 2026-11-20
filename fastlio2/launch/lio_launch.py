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


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "lio.yaml"]
    )
    default_rviz = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "rviz", "fastlio2.rviz"]
    )

    config_path = LaunchConfiguration("config_path")
    enable_rviz = LaunchConfiguration("enable_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_tf = LaunchConfiguration("publish_tf")
    world_frame_override = LaunchConfiguration("world_frame_override")
    allow_missing_point_time = LaunchConfiguration("allow_missing_point_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_path",
                default_value=default_config,
                description="FAST-LIO2 sensor and estimator YAML",
            ),
            DeclareLaunchArgument(
                "enable_rviz",
                default_value="true",
                description="Start RViz2",
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
                description="Publish world_frame -> body_frame TF",
            ),
            DeclareLaunchArgument(
                "world_frame_override",
                default_value="",
                description="Override the YAML world_frame when non-empty",
            ),
            DeclareLaunchArgument(
                "allow_missing_point_time",
                default_value="false",
                description=(
                    "Allow simulation PointCloud2 without a per-point time field"
                ),
            ),
            Node(
                package="fastlio2",
                namespace="fastlio2",
                executable="lio_node",
                name="lio_node",
                output="screen",
                parameters=[
                    _lio_node_parameters(
                        config_path,
                        use_sim_time,
                        publish_tf,
                        world_frame_override,
                        allow_missing_point_time,
                    )
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="fastlio2_rviz",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(enable_rviz),
            ),
        ]
    )
