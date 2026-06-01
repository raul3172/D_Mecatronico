import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('arm_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'arm_description.xacro')
    controller_params = os.path.join(pkg_share, 'config', 'arm_controllers.yaml')

    # Compilamos el URDF activando el hardware invisible (Mock Hardware)
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' use_mock_hardware:=true']), 
        value_type=str
    )

    # 1. Publicador del estado del robot (Necesario para la matemática)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # 2. El cerebro central (Reemplaza a Gazebo)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controller_params],
        output='screen'
    )

    # 3. Activadores de tus controladores
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
    )

    # 4. Tu script puente hacia el ESP32 (Se lanza automáticamente)
    serial_bridge = Node(
        package='arm_description',
        executable='serial_bridge.py',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        serial_bridge, # <-- Lanzamos la comunicación al ESP32 directo
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner],
            )
        ),
        joint_state_broadcaster_spawner
    ])