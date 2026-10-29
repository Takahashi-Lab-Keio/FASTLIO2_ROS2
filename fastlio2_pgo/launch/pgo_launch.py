from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_lio_config = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "lio.yaml"]
    )
    default_pgo_config = PathJoinSubstitution(
        [FindPackageShare("fastlio2_pgo"), "config", "pgo.yaml"]
    )
    default_rviz = PathJoinSubstitution(
        [FindPackageShare("fastlio2_pgo"), "rviz", "pgo.rviz"]
    )

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
                default_value=default_lio_config,
                description="FAST-LIO2 sensor and estimator YAML",
            ),
            DeclareLaunchArgument(
                "pgo_config",
                default_value=default_pgo_config,
                description="Pose graph and map-saving YAML",
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
            Node(
                package="fastlio2",
                namespace="fastlio2",
                executable="lio_node",
                name="lio_node",
                output="screen",
                parameters=[
                    {
                        "config_path": lio_config,
                        "use_sim_time": use_sim_time,
                        "publish_tf": publish_tf,
                    }
                ],
            ),
            Node(
                package="fastlio2_pgo",
                namespace="fastlio2/pgo",
                executable="pgo_node",
                name="pgo_node",
                output="screen",
                parameters=[
                    {"config_path": pgo_config, "use_sim_time": use_sim_time}
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="fastlio2_mapping_rviz",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(enable_rviz),
            ),
        ]
    )
