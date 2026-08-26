from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lio_config = LaunchConfiguration("lio_config")
    pgo_config = LaunchConfiguration("pgo_config")
    enable_rviz = LaunchConfiguration("enable_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_tf = LaunchConfiguration("publish_tf")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lio_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("fastlio2"), "config", "jt128.yaml"]
                ),
                description="Hesai JT-128 sensor and estimator YAML",
            ),
            DeclareLaunchArgument(
                "pgo_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("fastlio2_pgo"), "config", "pgo.yaml"]
                ),
                description="Pose graph and map-saving YAML",
            ),
            DeclareLaunchArgument(
                "enable_rviz", default_value="true", description="Start RViz2"
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("fastlio2_pgo"), "rviz", "pgo.rviz"]
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("fastlio2_pgo"),
                            "launch",
                            "pgo_launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "lio_config": lio_config,
                    "pgo_config": pgo_config,
                    "enable_rviz": enable_rviz,
                    "rviz_config": rviz_config,
                    "use_sim_time": use_sim_time,
                    "publish_tf": publish_tf,
                }.items(),
            )
        ]
    )
