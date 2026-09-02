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
    keyframe_distance = LaunchConfiguration("keyframe_distance")
    keyframe_yaw = LaunchConfiguration("keyframe_yaw")
    recent_exclusion_num = LaunchConfiguration("recent_exclusion_num")
    loop_min_path_separation = LaunchConfiguration("loop_min_path_separation")
    loop_candidate_top_k = LaunchConfiguration("loop_candidate_top_k")
    icp_fitness_threshold_normal = LaunchConfiguration("icp_fitness_threshold_normal")
    loop_bidirectional_icp_enable = LaunchConfiguration("loop_bidirectional_icp_enable")
    loop_observability_enable = LaunchConfiguration("loop_observability_enable")
    loop_observability_reject_enable = LaunchConfiguration("loop_observability_reject_enable")
    loop_current_cooldown_keyframes = LaunchConfiguration("loop_current_cooldown_keyframes")
    loop_candidate_cooldown_keyframes = LaunchConfiguration("loop_candidate_cooldown_keyframes")
    loop_pair_cooldown_keyframes = LaunchConfiguration("loop_pair_cooldown_keyframes")
    sync_queue_size = LaunchConfiguration("sync_queue_size")
    backend_tum_path = LaunchConfiguration("backend_tum_path")
    backend_full_tum_path = LaunchConfiguration("backend_full_tum_path")
    backend_loop_csv_path = LaunchConfiguration("backend_loop_csv_path")
    backend_log_enable = LaunchConfiguration("backend_log_enable")
    publish_optimized_map = LaunchConfiguration("publish_optimized_map")
    optimized_map_publish_interval = LaunchConfiguration("optimized_map_publish_interval")
    optimized_map_leaf_size = LaunchConfiguration("optimized_map_leaf_size")
    frontend_runtime_csv_path = LaunchConfiguration("frontend_runtime_csv_path")
    adaptive_map_enable = LaunchConfiguration("adaptive_map_enable")
    adaptive_window_enable = LaunchConfiguration("adaptive_window_enable")

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

    declare_keyframe_distance_cmd = DeclareLaunchArgument(
        "keyframe_distance",
        default_value="0.5",
        description="Backend keyframe translation threshold in meters"
    )

    declare_keyframe_yaw_cmd = DeclareLaunchArgument(
        "keyframe_yaw",
        default_value="0.10",
        description="Backend keyframe yaw threshold in radians"
    )

    declare_recent_exclusion_num_cmd = DeclareLaunchArgument(
        "recent_exclusion_num",
        default_value="30",
        description=(
            "Exclude this many most recent keyframes from ScanContext loop "
            "retrieval to avoid short-term repeated-structure matches"
        )
    )

    declare_loop_min_path_separation_cmd = DeclareLaunchArgument(
        "loop_min_path_separation",
        default_value="0.0",
        description=(
            "Minimum frontend path distance in meters between loop endpoints; "
            "0 disables this additional temporal/spatial exclusion"
        )
    )

    declare_icp_fitness_threshold_normal_cmd = DeclareLaunchArgument(
        "icp_fitness_threshold_normal",
        default_value="0.30",
        description=(
            "Maximum ICP fitness for a normal-state loop candidate; lower "
            "values apply stricter geometric verification"
        )
    )

    declare_loop_candidate_top_k_cmd = DeclareLaunchArgument(
        "loop_candidate_top_k",
        default_value="1",
        description="Maximum number of time-separated ScanContext candidates verified per keyframe"
    )

    declare_loop_bidirectional_icp_enable_cmd = DeclareLaunchArgument(
        "loop_bidirectional_icp_enable",
        default_value="false",
        description="Require reverse ICP and forward/reverse transform consistency for loop acceptance"
    )

    declare_loop_observability_enable_cmd = DeclareLaunchArgument(
        "loop_observability_enable",
        default_value="false",
        description="Require a well-conditioned ICP Hessian-SVD loop constraint"
    )

    declare_loop_observability_reject_enable_cmd = DeclareLaunchArgument(
        "loop_observability_reject_enable",
        default_value="false",
        description="Reject loop factors failing the recorded ICP Hessian-SVD observability criteria"
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

    declare_backend_full_tum_path_cmd = DeclareLaunchArgument(
        "backend_full_tum_path",
        default_value=(
            "/home/romi/Adaptive_FAST_LIO2/experiments/"
            "subt_mrs_hawkins_long_corridor/results/"
            "adaptive_backend_optimized_full.tum"
        ),
        description=(
            "Full-rate TUM trajectory after propagating optimized keyframe "
            "corrections to all frontend odometry samples"
        )
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

    declare_adaptive_map_enable_cmd = DeclareLaunchArgument(
        "adaptive_map_enable",
        default_value="true",
        description="Enable adaptive point-quality filtering and map insertion control"
    )

    declare_adaptive_window_enable_cmd = DeclareLaunchArgument(
        "adaptive_window_enable",
        default_value="true",
        description="Enable sliding-window persistent-degeneracy state machine"
    )

    adaptive_lio_node = Node(
        package="adaptive_fast_lio2",
        executable="adaptive_fastlio_mapping",
        name="adaptive_fastlio_mapping",
        parameters=[
            PathJoinSubstitution([config_path, config_file]),
            {"use_sim_time": use_sim_time},
            {"runtime_log.csv_path": frontend_runtime_csv_path},
            {"adaptive_map.enable": adaptive_map_enable},
            {"adaptive_window.enable": adaptive_window_enable}
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
            {"keyframe_distance": keyframe_distance},
            {"keyframe_yaw": keyframe_yaw},
            {"recent_exclusion_num": recent_exclusion_num},
            {"loop_min_path_separation": loop_min_path_separation},
            {"loop_candidate_top_k": loop_candidate_top_k},
            {"icp_fitness_threshold_normal": icp_fitness_threshold_normal},
            {"loop_bidirectional_icp_enable": loop_bidirectional_icp_enable},
            {"loop_observability_enable": loop_observability_enable},
            {"loop_observability_reject_enable": loop_observability_reject_enable},
            {"loop_current_cooldown_keyframes": loop_current_cooldown_keyframes},
            {"loop_candidate_cooldown_keyframes": loop_candidate_cooldown_keyframes},
            {"loop_pair_cooldown_keyframes": loop_pair_cooldown_keyframes},
            {"sync_queue_size": sync_queue_size},
            {"backend_tum_path": backend_tum_path},
            {"backend_full_tum_path": backend_full_tum_path},
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
    ld.add_action(declare_keyframe_distance_cmd)
    ld.add_action(declare_keyframe_yaw_cmd)
    ld.add_action(declare_recent_exclusion_num_cmd)
    ld.add_action(declare_loop_min_path_separation_cmd)
    ld.add_action(declare_loop_candidate_top_k_cmd)
    ld.add_action(declare_icp_fitness_threshold_normal_cmd)
    ld.add_action(declare_loop_bidirectional_icp_enable_cmd)
    ld.add_action(declare_loop_observability_enable_cmd)
    ld.add_action(declare_loop_observability_reject_enable_cmd)
    ld.add_action(declare_loop_current_cooldown_cmd)
    ld.add_action(declare_loop_candidate_cooldown_cmd)
    ld.add_action(declare_loop_pair_cooldown_cmd)
    ld.add_action(declare_sync_queue_cmd)
    ld.add_action(declare_backend_tum_path_cmd)
    ld.add_action(declare_backend_full_tum_path_cmd)
    ld.add_action(declare_backend_loop_csv_path_cmd)
    ld.add_action(declare_backend_log_enable_cmd)
    ld.add_action(declare_publish_optimized_map_cmd)
    ld.add_action(declare_optimized_map_publish_interval_cmd)
    ld.add_action(declare_optimized_map_leaf_size_cmd)
    ld.add_action(declare_frontend_runtime_csv_path_cmd)
    ld.add_action(declare_adaptive_map_enable_cmd)
    ld.add_action(declare_adaptive_window_enable_cmd)

    ld.add_action(adaptive_lio_node)
    ld.add_action(adaptive_backend_node)
    ld.add_action(rviz_node)

    return ld
