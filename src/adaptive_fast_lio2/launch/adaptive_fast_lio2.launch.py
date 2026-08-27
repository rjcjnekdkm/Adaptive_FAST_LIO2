import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory("adaptive_fast_lio2")
    default_config_path = os.path.join(package_path, "config")
    default_rviz_config_path = os.path.join(
        package_path, "rviz", "fastlio.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    config_path = LaunchConfiguration("config_path")
    config_file = LaunchConfiguration("config_file")
    rviz_use = LaunchConfiguration("rviz")
    rviz_cfg = LaunchConfiguration("rviz_cfg")
    backend_use = LaunchConfiguration("backend")
    loop_current_cooldown_keyframes = LaunchConfiguration("loop_current_cooldown_keyframes")
    loop_candidate_cooldown_keyframes = LaunchConfiguration("loop_candidate_cooldown_keyframes")
    loop_pair_cooldown_keyframes = LaunchConfiguration("loop_pair_cooldown_keyframes")
    sync_queue_size = LaunchConfiguration("sync_queue_size")
    backend_tum_path = LaunchConfiguration("backend_tum_path")
    backend_loop_csv_path = LaunchConfiguration("backend_loop_csv_path")
    backend_log_enable = LaunchConfiguration("backend_log_enable")
    publish_optimized_map = LaunchConfiguration("publish_optimized_map")
    optimized_map_publish_interval = LaunchConfiguration("optimized_map_publish_interval")
    optimized_map_leaf_size = LaunchConfiguration("optimized_map_leaf_size")
    frontend_runtime_csv_path = LaunchConfiguration("frontend_runtime_csv_path")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true"
    )

    declare_config_path_cmd = DeclareLaunchArgument(
        "config_path",
        default_value=default_config_path,
        description="Yaml config file path"
    )

    declare_config_file_cmd = DeclareLaunchArgument(
        "config_file",
        default_value="adaptive_fast_lio2.yaml",
        description="Config file"
    )

    declare_rviz_cmd = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Use RViz to monitor results"
    )

    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        "rviz_cfg",
        default_value=default_rviz_config_path,
        description="RViz config file path"
    )

    declare_backend_cmd = DeclareLaunchArgument(
        "backend",
        default_value="false",
        description="Run degenerate-aware loop backend if true"
    )

    declare_loop_current_cooldown_cmd = DeclareLaunchArgument(
        "loop_current_cooldown_keyframes",
        default_value="10",
        description="Cooldown distance for nearby current keyframes"
    )

    declare_loop_candidate_cooldown_cmd = DeclareLaunchArgument(
        "loop_candidate_cooldown_keyframes",
        default_value="20",
        description="Cooldown distance for nearby candidate keyframes"
    )

    declare_loop_pair_cooldown_cmd = DeclareLaunchArgument(
        "loop_pair_cooldown_keyframes",
        default_value="30",
        description="Cooldown distance for a repeated loop pair event"
    )

    declare_sync_queue_cmd = DeclareLaunchArgument(
        "sync_queue_size",
        default_value="200",
        description="Number of cloud/degeneracy messages kept for timestamp matching"
    )

    declare_backend_tum_path_cmd = DeclareLaunchArgument(
        "backend_tum_path",
        default_value=(
            "/home/romi/Adaptive_FAST_LIO2/experiments/"
            "subt_mrs_hawkins_long_corridor/results/"
            "adaptive_backend_optimized.tum"
        ),
        description="TUM output path for optimized backend trajectory"
    )

    declare_backend_loop_csv_path_cmd = DeclareLaunchArgument(
        "backend_loop_csv_path",
        default_value=(
            "/home/romi/Adaptive_FAST_LIO2/experiments/"
            "subt_mrs_hawkins_long_corridor/results/"
            "adaptive_backend_loops.csv"
        ),
        description="CSV output path for backend loop diagnostics"
    )

    declare_backend_log_enable_cmd = DeclareLaunchArgument(
        "backend_log_enable",
        default_value="true",
        description="Enable backend trajectory and loop CSV logging"
    )

    declare_publish_optimized_map_cmd = DeclareLaunchArgument(
        "publish_optimized_map",
        default_value="false",
        description="Publish the backend optimized map for RViz"
    )

    declare_optimized_map_publish_interval_cmd = DeclareLaunchArgument(
        "optimized_map_publish_interval",
        default_value="10",
        description="Publish optimized map every N backend keyframes"
    )

    declare_optimized_map_leaf_size_cmd = DeclareLaunchArgument(
        "optimized_map_leaf_size",
        default_value="0.5",
        description="Voxel leaf size for the published optimized map"
    )

    declare_frontend_runtime_csv_path_cmd = DeclareLaunchArgument(
        "frontend_runtime_csv_path",
        default_value=(
            "/home/romi/Adaptive_FAST_LIO2/experiments/"
            "geode_tunneling_tunnel_gamma/results/adaptive_runtime.csv"
        ),
        description="CSV output path for frontend runtime/degradation statistics"
    )

    adaptive_lio_node = Node(
        package="adaptive_fast_lio2",
        executable="adaptive_fastlio_mapping",
        name="adaptive_fastlio_mapping",
        parameters=[
            PathJoinSubstitution([config_path, config_file]),
            {"use_sim_time": use_sim_time},
            {"runtime_log.csv_path": frontend_runtime_csv_path}
        ],
        output="screen"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    adaptive_backend_node = Node(
        package="adaptive_fast_lio2",
        executable="adaptive_degenerate_backend",
        name="adaptive_degenerate_backend",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"loop_current_cooldown_keyframes": loop_current_cooldown_keyframes},
            {"loop_candidate_cooldown_keyframes": loop_candidate_cooldown_keyframes},
            {"loop_pair_cooldown_keyframes": loop_pair_cooldown_keyframes},
            {"sync_queue_size": sync_queue_size},
            {"backend_tum_path": backend_tum_path},
            {"backend_loop_csv_path": backend_loop_csv_path},
            {"backend_log_enable": backend_log_enable},
            {"publish_optimized_map": publish_optimized_map},
            {"optimized_map_publish_interval": optimized_map_publish_interval},
            {"optimized_map_leaf_size": optimized_map_leaf_size}
        ],
        output="screen",
        condition=IfCondition(backend_use)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_backend_cmd)
    ld.add_action(declare_loop_current_cooldown_cmd)
    ld.add_action(declare_loop_candidate_cooldown_cmd)
    ld.add_action(declare_loop_pair_cooldown_cmd)
    ld.add_action(declare_sync_queue_cmd)
    ld.add_action(declare_backend_tum_path_cmd)
    ld.add_action(declare_backend_loop_csv_path_cmd)
    ld.add_action(declare_backend_log_enable_cmd)
    ld.add_action(declare_publish_optimized_map_cmd)
    ld.add_action(declare_optimized_map_publish_interval_cmd)
    ld.add_action(declare_optimized_map_leaf_size_cmd)
    ld.add_action(declare_frontend_runtime_csv_path_cmd)

    ld.add_action(adaptive_lio_node)
    ld.add_action(adaptive_backend_node)
    ld.add_action(rviz_node)

    return ld
