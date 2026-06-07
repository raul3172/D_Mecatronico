#!/usr/bin/env python3
"""
real_robot.launch.py
====================
Lanza el brazo en modo real (Raspberry Pi + ESP32, sin Gazebo).

Secuencia de arranque garantizada:
  1. robot_state_publisher    ← publica TF y /robot_description
  2. ros2_control_node        ← controller_manager con mock_components
  3. joint_state_broadcaster  ← publica /joint_states (mock actualiza estos)
  4. arm_controller           ← JointTrajectoryController (4 joints del brazo)
  5. gripper_controller       ← JointTrajectoryController (1 joint gripper)
  6. serial_bridge            ← lee /joint_states y envía tramas al ESP32

Cada paso espera a que el anterior termine (OnProcessExit).
El serial_bridge solo arranca cuando TODOS los controladores están activos,
evitando que envíe tramas de ceros mientras el sistema inicializa.
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    RegisterEventHandler,
    LogInfo,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE = 'arm_description'


def generate_launch_description():

    pkg_share   = FindPackageShare(PACKAGE)
    urdf_file   = PathJoinSubstitution([pkg_share, 'urdf',   'Diseño_Final_Brazo_carro_5.urdf'])
    ctrl_yaml   = PathJoinSubstitution([pkg_share, 'config', 'arm_controllers.yaml'])
    bridge_exec = PathJoinSubstitution([pkg_share, 'scripts', 'serial_bridge.py'])

    # ── robot_description: URDF plano (sin xacro, ya está procesado) ──
    robot_desc = ParameterValue(
        Command(['cat ', urdf_file]),
        value_type=str
    )

    # ─────────────────────────────────────────────────────────────
    # NODO 1 — robot_state_publisher
    #   Publica TF y /robot_description para RViz y demás nodos.
    # ─────────────────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc,
                     'publish_frequency': 50.0}],
    )

    # ─────────────────────────────────────────────────────────────
    # NODO 2 — controller_manager (ros2_control_node)
    #   Hardware plugin: mock_components/GenericSystem
    #   → simula los joints en RAM; el serial_bridge hace el puente real
    # ─────────────────────────────────────────────────────────────
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'robot_description': robot_desc},
            ctrl_yaml,
        ],
    )

    # ─────────────────────────────────────────────────────────────
    # SPAWNER 1 — joint_state_broadcaster
    #   Publica /joint_states a partir del estado interno del mock.
    #   Arranca en cuanto el controller_manager está vivo.
    # ─────────────────────────────────────────────────────────────
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    # ─────────────────────────────────────────────────────────────
    # SPAWNER 2 — arm_controller (4 joints del brazo + riel)
    #   Arranca cuando joint_state_broadcaster_spawner termina (exit 0).
    # ─────────────────────────────────────────────────────────────
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    # ─────────────────────────────────────────────────────────────
    # SPAWNER 3 — gripper_controller (1 joint)
    #   Arranca cuando arm_controller_spawner termina.
    # ─────────────────────────────────────────────────────────────
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    # ─────────────────────────────────────────────────────────────
    # NODO 3 — serial_bridge
    #   Lee /joint_states y envía tramas al ESP32 vía puerto serial.
    #   Arranca cuando gripper_controller_spawner termina → TODOS los
    #   controladores están activos y /joint_states fluye correctamente.
    # ─────────────────────────────────────────────────────────────
    serial_bridge = Node(
        package='arm_description',
        executable='serial_bridge.py',
        output='screen',
        emulate_tty=True,       # para ver los emojis del logger correctamente
    )

    # ─────────────────────────────────────────────────────────────
    # CADENA DE EVENTOS (orden garantizado)
    #
    #  controller_manager arranca
    #         │
    #         ▼
    #  joint_state_broadcaster_spawner (inicia al arrancar el ctrl_manager)
    #         │ OnProcessExit (exit 0 = spawned ok)
    #         ▼
    #  arm_controller_spawner
    #         │ OnProcessExit
    #         ▼
    #  gripper_controller_spawner
    #         │ OnProcessExit
    #         ▼
    #  serial_bridge  ← arranca aquí, cuando todo está listo
    # ─────────────────────────────────────────────────────────────
    on_jsb_ready = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                LogInfo(msg='✅ joint_state_broadcaster activo → iniciando arm_controller'),
                arm_controller_spawner,
            ],
        )
    )

    on_arm_ready = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[
                LogInfo(msg='✅ arm_controller activo → iniciando gripper_controller'),
                gripper_controller_spawner,
            ],
        )
    )

    on_gripper_ready = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gripper_controller_spawner,
            on_exit=[
                LogInfo(msg='✅ gripper_controller activo → iniciando serial_bridge'),
                serial_bridge,
            ],
        )
    )

    return LaunchDescription([
        # Nodos que arrancan inmediatamente
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,

        # Cadena de eventos
        on_jsb_ready,
        on_arm_ready,
        on_gripper_ready,
    ])