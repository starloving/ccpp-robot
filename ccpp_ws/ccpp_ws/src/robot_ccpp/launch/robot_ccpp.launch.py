import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('robot_ccpp'),
            'params',
            'robot_ccpp.yaml'))  
    
    params_file = LaunchConfiguration('params_file')
    
    param_substitutions = {
        'use_sim_time': use_sim_time}
    
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True)
    
    # 全覆盖路径规划算法
    path_planning_node = Node(package='robot_ccpp',
                            executable='path_planning_node',
                            parameters=[{'use_sim_time': use_sim_time},configured_params],
                            output = 'screen'
                            )
    # rviz2 显示
    rviz2_node = Node(
                        package='rviz2',
                        executable='rviz2',
                        name='rviz2',
                        arguments=['-d', os.path.join(
                                            get_package_share_directory('robot_ccpp'),
                                            'rviz2',
                                            'rviz2_ccpp.rviz')],
                        parameters=[{'use_sim_time': use_sim_time}],
                        output='screen')
    # next_goal node 发布
    next_goal_node = Node(package='robot_ccpp',
                            executable='next_goal_node',
                            parameters=[{'use_sim_time': use_sim_time},configured_params],
                            output = 'screen'
                            )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),
        
        # voxel_visualizer_node,
        path_planning_node,
        # rviz2 node
        rviz2_node,
        next_goal_node
    ])
