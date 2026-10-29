from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("fastlio2")
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
                default_value=PathJoinSubstitution(
                    [package_share, "config", "jt128.yaml"]
                ),
                description="Hesai JT-128 sensor and estimator YAML",
            ),
            DeclareLaunchArgument(
                "enable_rviz", default_value="true", description="Start RViz2"
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [package_share, "rviz", "fastlio2.rviz"]
                ),
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
                description="Override the YAML world_frame when non-empty",
            ),
            DeclareLaunchArgument(
                "allow_missing_point_time",
                default_value="false",
                description=(
                    "Allow simulation PointCloud2 without a per-point time field"
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([package_share, "launch", "lio_launch.py"])
                ),
                launch_arguments={
                    "config_path": config_path,
                    "enable_rviz": enable_rviz,
                    "rviz_config": rviz_config,
                    "use_sim_time": use_sim_time,
                    "publish_tf": publish_tf,
                    "world_frame_override": world_frame_override,
                    "allow_missing_point_time": allow_missing_point_time,
                }.items(),
            )
        ]
    )
