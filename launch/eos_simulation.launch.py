#!/usr/bin/env python3
"""
Eos Robotics OS Simulation Launch File

Launches Gazebo simulation with TurtleBot3 and the Eos ROS node for
testing adaptive navigation with spiking neural networks.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Generate launch description for Eos simulation.
    
    Returns:
        LaunchDescription: Complete launch configuration
    """
    
    # =========================================================================
    # Configuration Parameters
    # =========================================================================
    
    # Package paths
    eos_pkg_share = FindPackageShare('eos_robotics').find('eos_robotics')
    turtlebot3_pkg_share = FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo')
    nav2_pkg_share = FindPackageShare('nav2_bringup').find('nav2_bringup')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='turtlebot3_world')
    robot_model = LaunchConfiguration('robot_model', default='waffle_pi')
    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    enable_neural = LaunchConfiguration('enable_neural', default='true')
    
    # =========================================================================
    # Launch Arguments
    # =========================================================================
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='turtlebot3_world',
        description='Gazebo world file name without extension'
    )
    
    declare_robot_model = DeclareLaunchArgument(
        'robot_model',
        default_value='waffle_pi',
        description='TurtleBot3 model type: burger, waffle, or waffle_pi'
    )
    
    declare_enable_rviz = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Enable RViz2 visualization'
    )
    
    declare_enable_neural = DeclareLaunchArgument(
        'enable_neural',
        default_value='true',
        description='Enable Eos neural network processing'
    )
    
    # =========================================================================
    # Gazebo Simulation
    # =========================================================================
    
    # Gazebo world file path
    world_path = PathJoinSubstitution([
        turtlebot3_pkg_share,
        'worlds',
        [world_name, '.world']
    ])
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                turtlebot3_pkg_share,
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world_path,
            'robot_model': robot_model
        }.items()
    )
    
    # =========================================================================
    # Eos Robotics OS Nodes
    # =========================================================================
    
    # Main Eos ROS node
    eos_ros_node = Node(
        package='eos_robotics',
        executable='eos_ros_node',
        name='eos_ros_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'neural_update_rate': 10.0,
            'navigation_update_rate': 15.0,
            'safety_distance': 0.5,
            'max_velocity': 0.5,
            'neural_model_path': PathJoinSubstitution([
                eos_pkg_share, 'models', 'default_snn.json'
            ])
        }],
        condition=IfCondition(enable_neural),
        remappings=[
            ('/scan', '/scan'),
            ('/imu', '/imu'),
            ('/odom', '/odom'),
            ('/cmd_vel', '/cmd_vel')
        ]
    )
    
    # Python neural bridge (if using Python for neural network interface)
    neural_bridge_node = Node(
        package='eos_robotics',
        executable='neural_bridge.py',
        name='neural_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'model_path': PathJoinSubstitution([
                eos_pkg_share, 'models', 'snn_model.h5'
            ]),
            'update_rate': 10.0
        }],
        condition=IfCondition(enable_neural)
    )
    
    # Sensor data processor
    sensor_processor_node = Node(
        package='eos_robotics',
        executable='sensor_processor.py',
        name='sensor_processor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'laser_topic': '/scan',
            'imu_topic': '/imu',
            'odom_topic': '/odom'
        }]
    )
    
    # =========================================================================
    # Navigation Stack (Optional - for comparison with Eos navigation)
    # =========================================================================
    
    # SLAM Toolbox for mapping (optional)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'params_file': PathJoinSubstitution([
                eos_pkg_share, 'config', 'slam_toolbox.yaml'
            ])
        }]
    )
    
    # =========================================================================
    # Visualization
    # =========================================================================
    
    # RViz2 configuration file path
    rviz_config = PathJoinSubstitution([
        eos_pkg_share, 'config', 'eos_navigation.rviz'
    ])
    
    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(enable_rviz),
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # =========================================================================
    # Utility Nodes
    # =========================================================================
    
    # Static transform publisher for robot base
    static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command([
                'xacro ', PathJoinSubstitution([
                    turtlebot3_pkg_share,
                    'urdf',
                    [robot_model, '.urdf.xacro']
                ])
            ])
        }]
    )
    
    # =========================================================================
    # Event Handlers
    # =========================================================================
    
    # Delay RViz start after Gazebo is ready
    delay_rviz_after_gazebo = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_launch,
            on_exit=[rviz_node]
        )
    )
    
    # =========================================================================
    # Launch Description
    # =========================================================================
    
    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time,
        declare_world_name,
        declare_robot_model,
        declare_enable_rviz,
        declare_enable_neural,
        
        # Simulation
        gazebo_launch,
        
        # Eos nodes
        eos_ros_node,
        neural_bridge_node,
        sensor_processor_node,
        
        # Navigation (optional)
        slam_toolbox_node,
        
        # Visualization and utilities
        static_transform,
        robot_state_publisher,
        delay_rviz_after_gazebo,
    ])

# Additional launch configurations for different scenarios

def generate_navigation_test_launch_description():
    """
    Generate launch description for navigation testing without neural networks.
    """
    base_launch = generate_launch_description()
    
    # Override neural network enablement
    for action in base_launch.actions:
        if hasattr(action, 'condition') and 'enable_neural' in str(action.condition):
            action.condition = IfCondition('false')
    
    return base_launch

def generate_real_robot_launch_description():
    """
    Generate launch description for real robot deployment.
    """
    real_robot_launch = LaunchDescription()
    
    # Similar structure but with real sensor drivers instead of Gazebo
    # This would include:
    # - Real LiDAR driver
    # - Real IMU driver  
    # - Real motor controllers
    # - No Gazebo simulation
    
    return real_robot_launch