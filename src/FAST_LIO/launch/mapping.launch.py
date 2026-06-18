import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    runtime_recorder = LaunchConfiguration('runtime_recorder')
    runtime_csv_path = LaunchConfiguration('runtime_csv_path')
    runtime_csv_append = LaunchConfiguration('runtime_csv_append')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    decalre_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    declare_runtime_recorder_cmd = DeclareLaunchArgument(
        'runtime_recorder', default_value='true',
        description='Start the external runtime recorder'
    )
    declare_runtime_csv_path_cmd = DeclareLaunchArgument(
        'runtime_csv_path',
        default_value=(
            '/home/romi/Adaptive_FAST_LIO2/experiments/'
            'subt_mrs_hawkins_long_corridor/results/'
            'fastlio_runtime_external.csv'
        ),
        description='FAST-LIO runtime CSV output path'
    )
    declare_runtime_csv_append_cmd = DeclareLaunchArgument(
        'runtime_csv_append', default_value='false',
        description='Append to the runtime CSV instead of truncating it'
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )
    runtime_recorder_node = Node(
        package='fast_lio',
        executable='fastlio_runtime_recorder',
        name='fastlio_runtime_recorder',
        parameters=[{
            'use_sim_time': use_sim_time,
            'csv_path': runtime_csv_path,
            'csv_append': runtime_csv_append,
        }],
        condition=IfCondition(runtime_recorder),
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(decalre_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_runtime_recorder_cmd)
    ld.add_action(declare_runtime_csv_path_cmd)
    ld.add_action(declare_runtime_csv_append_cmd)

    ld.add_action(fast_lio_node)
    ld.add_action(runtime_recorder_node)
    ld.add_action(rviz_node)

    return ld
