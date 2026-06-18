import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():

    share_dir = get_package_share_directory('lio_sam')
    parameter_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    runtime_recorder = LaunchConfiguration('runtime_recorder')
    runtime_csv_path = LaunchConfiguration('runtime_csv_path')
    runtime_csv_append = LaunchConfiguration('runtime_csv_append')
    xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'params.yaml'),
        description='FPath to the ROS2 parameters file to use.')

    use_sim_time_declare = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true.')

    runtime_recorder_declare = DeclareLaunchArgument(
        'runtime_recorder',
        default_value='true',
        description='Start the external runtime recorder.')

    runtime_csv_path_declare = DeclareLaunchArgument(
        'runtime_csv_path',
        default_value=(
            '/home/romi/Adaptive_FAST_LIO2/experiments/'
            'subt_mrs_hawkins_long_corridor/results/'
            'liosam_runtime_external.csv'
        ),
        description='LIO-SAM runtime CSV output path.')

    runtime_csv_append_declare = DeclareLaunchArgument(
        'runtime_csv_append',
        default_value='false',
        description='Append to the runtime CSV instead of truncating it.')

    print("urdf_file_name : {}".format(xacro_path))

    return LaunchDescription([
        params_declare,
        use_sim_time_declare,
        runtime_recorder_declare,
        runtime_csv_path_declare,
        runtime_csv_append_declare,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
            parameters=[parameter_file, {'use_sim_time': use_sim_time}],
            output='screen'
            ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro', ' ', xacro_path]),
                'use_sim_time': use_sim_time
            }]
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_imuPreintegration',
            name='lio_sam_imuPreintegration',
            parameters=[parameter_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_imageProjection',
            name='lio_sam_imageProjection',
            parameters=[parameter_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_featureExtraction',
            name='lio_sam_featureExtraction',
            parameters=[parameter_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_mapOptimization',
            name='lio_sam_mapOptimization',
            parameters=[parameter_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='liosam_runtime_recorder',
            name='liosam_runtime_recorder',
            parameters=[{
                'use_sim_time': use_sim_time,
                'csv_path': runtime_csv_path,
                'csv_append': runtime_csv_append
            }],
            condition=IfCondition(runtime_recorder),
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])
