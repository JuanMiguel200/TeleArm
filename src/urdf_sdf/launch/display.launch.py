import os
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'urdf_sdf'  # Cambia esto por el nombre de tu paquete
    urdf_file = 'test.urdf'  # Nombre de tu archivo URDF

    urdf_path = os.path.join(get_package_share_directory(pkg_name), 'urdf', urdf_file)

    # Argumento para RViz con configuración por defecto
    rviz_config = os.path.join(get_package_share_directory(pkg_name), 'rviz', 'conf.rviz')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Usar tiempo de simulación si es True'
        ),
        
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', urdf_path])}]
        ),

        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])
