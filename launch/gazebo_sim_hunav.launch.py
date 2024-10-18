from os import path, environ
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable,
                            RegisterEventHandler, TimerAction, LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    
    # Determine workspace root and paths
    current_dir = os.path.abspath(__file__)
    workspace_root = current_dir
    while not workspace_root.endswith('arena4_ws'):
        workspace_root = os.path.dirname(workspace_root)
    if not workspace_root.endswith('arena4_ws'):
        raise ValueError("Could not find the 'arena4_ws' directory in the current path.")

    # Define paths for Gazebo Sim and resources

    GZ_CONFIG_PATH = os.path.join(workspace_root, 'install', 'gz-sim7', 'share', 'gz')
    GZ_SIM_PHYSICS_ENGINE_PATH = os.path.join(workspace_root, 'build', 'gz-physics6')
    GZ_SIM_RESOURCE_PATHS = [
        os.path.join(workspace_root, 'src', 'gazebo', 'gz-sim', 'test', 'worlds', 'models'),
        os.path.join(workspace_root, 'src', 'gazebo', 'gz-sim', 'examples', 'worlds'),
        os.path.join(workspace_root, 'src', 'arena', 'simulation-setup', 'entities'),
        os.path.join(workspace_root, 'src', 'arena', 'simulation-setup', 'worlds'),
        os.path.join(workspace_root, 'src', 'arena', 'simulation-setup', 'gazebo_models'),
        os.path.join(workspace_root, 'src', 'deps'),
        os.path.join(workspace_root, 'src', 'deps', 'hunav_gazebo_wrapper'),
        os.path.join(workspace_root, 'src', 'deps', 'hunav_gazebo_wrapper', 'media', 'models'),
        os.path.join(workspace_root, 'src', 'deps', 'hunav_gazebo_wrapper', 'worlds'),
        os.path.join(workspace_root, 'src', 'deps', 'hunav_sim'),
        # Add the install path for hunav_gazebo_wrapper models
        os.path.join(workspace_root, 'install', 'hunav_gazebo_wrapper', 'share', 'hunav_gazebo_wrapper', 'models')
    ]

    GZ_SIM_RESOURCE_PATHS_COMBINED = ':'.join(GZ_SIM_RESOURCE_PATHS)

    # Set environment variables for Gazebo Sim paths
    set_gz_config_path = SetEnvironmentVariable('GZ_CONFIG_PATH', GZ_CONFIG_PATH)
    set_gz_sim_physics_engine_path = SetEnvironmentVariable('GZ_SIM_PHYSICS_ENGINE_PATH', GZ_SIM_PHYSICS_ENGINE_PATH)
    set_gz_sim_resource_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', GZ_SIM_RESOURCE_PATHS_COMBINED)

    # World generation parameters
    world_file = LaunchConfiguration('base_world')
    gz_obs = LaunchConfiguration('use_gazebo_obs')
    rate = LaunchConfiguration('update_rate')
    robot_name = LaunchConfiguration('robot_name')
    global_frame = LaunchConfiguration('global_frame_to_publish')
    use_navgoal = LaunchConfiguration('use_navgoal_to_start')
    ignore_models = LaunchConfiguration('ignore_models')

    # Agent configuration file
    agent_conf_file = PathJoinSubstitution([
        FindPackageShare('hunav_agent_manager'),
        'config',
        LaunchConfiguration('configuration_file')
    ])

    # Hunav Loader Node
    hunav_loader_node = Node(
        package='hunav_agent_manager',
        executable='hunav_loader',
        output='screen',
        parameters=[agent_conf_file]
    )

    # # World file for Ignition Gazebo (.world format)
    # world_file = PathJoinSubstitution([
    #     FindPackageShare('hunav_gazebo_wrapper'),
    #     'worlds',
    #     world_file_name  # This refers to the .world file
    # ])

    # Hunav Gazebo world generator
    hunav_gazebo_worldgen_node = Node(
        package='hunav_gazebo_wrapper',
        executable='hunav_gazebo_world_generator',
        output='screen',
        parameters=[{'base_world': world_file},
                    {'use_gazebo_obs': gz_obs},
                    {'update_rate': rate},
                    {'robot_name': robot_name},
                    {'global_frame_to_publish': global_frame},
                    {'use_navgoal_to_start': use_navgoal},
                    {'ignore_models': ignore_models}]
    )

    ordered_launch_event = RegisterEventHandler(
        OnProcessStart(
            target_action=hunav_loader_node,
            on_start=[
                LogInfo(msg='HunNavLoader started, launching HuNav_Gazebo_world_generator after 2 seconds...'),
                TimerAction(
                    period=2.0,
                    actions=[hunav_gazebo_worldgen_node],
                )
            ]
        )
    )

    # Gazebo Sim launch file from ros_gz_sim
    gz_sim_launch_file = path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_file),
        launch_arguments={
            'gz_args': [world_file, ' -v 6', ' -r', ' --render-engine ogre'],
            'physics-engine': 'gz-physics-dartsim'
        }.items()
    )

    # Hunav manager node
    hunav_manager_node = Node(
        package='hunav_agent_manager',
        executable='hunav_agent_manager',
        name='hunav_agent_manager',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    metrics_file = PathJoinSubstitution([
        FindPackageShare('hunav_evaluator'),
        'config',
        LaunchConfiguration('metrics_file')
    ])

    hunav_evaluator_node = Node(
        package='hunav_evaluator',
        executable='hunav_evaluator_node',
        output='screen',
        parameters=[metrics_file]
    )

    # Static TF node (if no robot localization is launched)
    static_tf_node = Node(
        package="tf2_ros", 
        executable="static_transform_publisher",
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # Declare launch arguments
    declare_agents_conf_file = DeclareLaunchArgument(
        'configuration_file', default_value='agents.yaml',
        description='Specify configuration file name in the config directory'
    )
    declare_metrics_conf_file = DeclareLaunchArgument(
        'metrics_file', default_value='metrics.yaml',
        description='Specify the name of the metrics configuration file in the config directory'
    )
    declare_arg_world = DeclareLaunchArgument(
        'base_world', default_value='empty_world.world',  # Specify .world file explicitly
        description='Specify the world file name'
    )
    declare_gz_obs = DeclareLaunchArgument(
        'use_gazebo_obs', default_value='true',
        description='Whether to fill the agents obstacles with closest Gazebo obstacle or not'
    )
    declare_update_rate = DeclareLaunchArgument(
        'update_rate', default_value='100.0',
        description='Update rate of the plugin'
    )
    declare_robot_name = DeclareLaunchArgument(
        'robot_name', default_value='actor3',
        description='Specify the name of the robot Gazebo model'
    )
    declare_frame_to_publish = DeclareLaunchArgument(
        'global_frame_to_publish', default_value='map',
        description='Name of the global frame in which the position of the agents are provided'
    )
    declare_use_navgoal = DeclareLaunchArgument(
        'use_navgoal_to_start', default_value='false',
        description='Whether to start the agents movements when a navigation goal is received or not'
    )
    declare_ignore_models = DeclareLaunchArgument(
        'ignore_models', default_value='ground_plane cafe',
        description='List of Gazebo models that the agents should ignore as obstacles. Indicate the models with a blank space between them'
    )
    declare_arg_verbose = DeclareLaunchArgument(
        'verbose', default_value='false',
        description='Set "true" to increase messages written to terminal.'
    )

    # LaunchDescription
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(set_gz_config_path)
    ld.add_action(set_gz_sim_physics_engine_path)
    ld.add_action(set_gz_sim_resource_path)

    # Add actions to the LaunchDescription
    ld.add_action(declare_agents_conf_file)
    ld.add_action(declare_metrics_conf_file)
    ld.add_action(declare_arg_world)
    ld.add_action(declare_gz_obs)
    ld.add_action(declare_update_rate)
    ld.add_action(declare_robot_name)
    ld.add_action(declare_frame_to_publish)
    ld.add_action(declare_use_navgoal)
    ld.add_action(declare_ignore_models)
    ld.add_action(declare_arg_verbose)

    # Generate the world with the agents
    ld.add_action(hunav_loader_node)
    ld.add_action(ordered_launch_event)

    # Add Gazebo Sim (Ignition) launch
    ld.add_action(gazebo)

    # Hunav behavior manager node and evaluator
    ld.add_action(hunav_manager_node)
    ld.add_action(hunav_evaluator_node)

    # Optional static TF publisher
    ld.add_action(static_tf_node)

    return ld
