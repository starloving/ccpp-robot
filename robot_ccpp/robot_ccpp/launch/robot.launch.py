import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    # Get the launch directory
    robot_ccpp_dir = get_package_share_directory('robot_ccpp')

    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('robot_ccpp'),
            'maps',
            'map.yaml'))
    
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('robot_ccpp'),
            'params',
            'robot_ccpp.yaml'))

    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = ['map_server']
    
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                    ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=map_dir,
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=param_dir,
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composed bringup if True')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    
    # 全覆盖路径规划算法node
    path_planning_node = Node(package='robot_ccpp',
                            executable='path_planning_node',
                            parameters=[{'use_sim_time': use_sim_time},configured_params],
                            output = 'screen'
                            )
    # 地图加载launch
    map_server_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('robot_ccpp'),
                                                    'launch',
                                                    'robot_map_server.launch.py')),
            # condition=IfCondition(PythonExpression(['not'])),
            launch_arguments={'namespace': namespace,
                            'map': map_yaml_file,
                            'use_sim_time': use_sim_time,
                            'autostart': autostart,
                            'params_file': params_file,
                            'use_composition': use_composition,
                            'use_respawn': use_respawn,
                            'container_name': 'nav2_container'}.items())
    
    # 模拟 tf2 发布node
    simulate_tf2_node = Node(package='robot_ccpp',
                        executable='simulate_tf2_pub.py',
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
    
    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    
    # launch 
    ld.add_action(map_server_launch)

    # Add the actions to launch all of the localiztion nodes
    ld.add_action(path_planning_node)
    ld.add_action(simulate_tf2_node)
    ld.add_action(rviz2_node)

    return ld
