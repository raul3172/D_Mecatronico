#!/usr/bin/env python3
"""
teach_pendant_gui.py  —  GUI para Diseño_Final_Brazo_carro_5 (Versión UI Híbrida)
=============================================================================
Sliders con resolución específica + Input por teclado (SpinBox)
"""

import sys
import threading
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QListWidget, QSpinBox,
    QMessageBox, QSlider, QGroupBox, QComboBox
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont

ARM_TOPIC = '/arm_controller/joint_trajectory'
GRIPPER_TOPIC = '/gripper_controller/joint_trajectory'

ARM_JOINTS = [
    'moviento_en_x',
    'disco_soportes_rotacion',
    'brazo_motores',
    'brazo_gripper_azul'
]
GRIPPER_JOINTS = ['gripper_mecanismo']
ALL_JOINTS = ARM_JOINTS + GRIPPER_JOINTS


class TeachAndRepeatNode(Node):
    def __init__(self):
        super().__init__('teach_pendant_node')
        self.sub = self.create_subscription(JointState, '/joint_states', self._state_cb, 10)
        self.pub_arm = self.create_publisher(JointTrajectory, ARM_TOPIC, 10)
        self.pub_grip = self.create_publisher(JointTrajectory, GRIPPER_TOPIC, 10)
        
        self._pos_map: dict[str, float] = {j: 0.0 for j in ALL_JOINTS}
        self.waypoints: list[list[float]] = []

    def _state_cb(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            if name in self._pos_map:
                self._pos_map[name] = pos

    @property
    def current_pos(self) -> list[float]:
        return [self._pos_map[j] for j in ALL_JOINTS]

    def send_single_point(self, positions: list[float], duration_sec: float = 1.0) -> None:
        if len(positions) < 5:
            return

        msg_arm = JointTrajectory()
        msg_arm.joint_names = ARM_JOINTS
        pt_arm = JointTrajectoryPoint()
        pt_arm.positions = positions[:4] 
        pt_arm.time_from_start.sec = int(duration_sec)
        pt_arm.time_from_start.nanosec = int((duration_sec % 1) * 1e9)
        msg_arm.points.append(pt_arm)
        self.pub_arm.publish(msg_arm)

        msg_grip = JointTrajectory()
        msg_grip.joint_names = GRIPPER_JOINTS
        pt_grip = JointTrajectoryPoint()
        pt_grip.positions = [positions[4]]
        pt_grip.time_from_start.sec = int(duration_sec)
        pt_grip.time_from_start.nanosec = int((duration_sec % 1) * 1e9)
        msg_grip.points.append(pt_grip)
        self.pub_grip.publish(msg_grip)

    def play_sequence(self, loop_count: int = 1) -> float:
        if not self.waypoints:
            return 0.0
            
        msg_arm = JointTrajectory(); msg_arm.joint_names = ARM_JOINTS
        msg_grip = JointTrajectory(); msg_grip.joint_names = GRIPPER_JOINTS
        
        t = 2.0
        for _ in range(loop_count):
            for wp in self.waypoints:
                pt_arm = JointTrajectoryPoint()
                pt_arm.positions = wp[:4]
                pt_arm.velocities = [0.0] * 4
                pt_arm.accelerations = [0.0] * 4
                pt_arm.time_from_start.sec = int(t)
                pt_arm.time_from_start.nanosec = int((t % 1) * 1e9)
                msg_arm.points.append(pt_arm)
                
                pt_grip = JointTrajectoryPoint()
                pt_grip.positions = [wp[4]]
                pt_grip.velocities = [0.0]
                pt_grip.accelerations = [0.0]
                pt_grip.time_from_start.sec = int(t)
                pt_grip.time_from_start.nanosec = int((t % 1) * 1e9)
                msg_grip.points.append(pt_grip)
                
                t += 2.0
                
        self.pub_arm.publish(msg_arm)
        self.pub_grip.publish(msg_grip)
        
        return (t - 2.0)


class TeachPendantGUI(QWidget):
    def __init__(self, ros_node: TeachAndRepeatNode):
        super().__init__()
        self.node = ros_node
        self.target_positions = [0.0] * 5
        
        # Config: (Nombre, Min, Max, Paso/Resolución, Sufijo)
        self.arm_configs = [
            ("Riel Base", 0, 460, 8, " mm"),
            ("Cintura", -150, 150, 10, "°"),
            ("Hombro", -150, 150, 10, "°"),
            ("Codo", -150, 150, 10, "°")
        ]
        
        # Tupla para guardar la referencia del (Slider, SpinBox)
        self.controls: list[tuple[QSlider, QSpinBox]] = []
        self.locked_joints = set()
        
        self.play_timer = QTimer()
        self.play_timer.setSingleShot(True)
        self.play_timer.timeout.connect(lambda: self._set_ui_state(True))
        
        self.arrival_timer = QTimer()
        self.arrival_timer.timeout.connect(self._check_arrival)
        
        self.riel_timer = QTimer()
        self.riel_timer.setSingleShot(True)
        self.riel_timer.timeout.connect(self._unlock_riel)
        
        self.telemetry_timer = QTimer()
        self.telemetry_timer.timeout.connect(self._update_telemetry)
        
        self._build_ui()
        self.telemetry_timer.start(100)

    def _build_ui(self):
        self.setWindowTitle('Teach Pendant - Operador Industrial')
        self.setMinimumSize(850, 500)
        
        main_layout = QVBoxLayout(self)

        self.lbl_tel = QLabel("Iniciando telemetría del sistema...")
        self.lbl_tel.setStyleSheet(
            "background-color: #1e272e; color: #0be881; padding: 8px;"
            "font-family: Consolas, monospace; font-size: 13px; border-radius: 4px;"
        )
        main_layout.addWidget(self.lbl_tel)

        body_layout = QHBoxLayout()
        
        self.left_panel = QGroupBox("Control Manual de Ejes (Resolución Fija)")
        left_layout = QVBoxLayout(self.left_panel)
        
        self.btn_home = QPushButton("IR A HOME (0, 0, 0, 0)")
        self.btn_home.setStyleSheet("background:#f39c12; color:white; height: 35px; font-weight:bold;")
        self.btn_home.clicked.connect(self._go_home)
        left_layout.addWidget(self.btn_home)
        left_layout.addSpacing(10)
        
        # Construcción de los 4 ejes (Sliders + Cajas de texto)
        for i, (label, lo, hi, step, suffix) in enumerate(self.arm_configs):
            row = QHBoxLayout()
            lbl_n = QLabel(label); lbl_n.setFixedWidth(80); lbl_n.setFont(QFont("Arial", 10, QFont.Bold))
            
            sl = QSlider(Qt.Horizontal)
            sl.setRange(lo, hi)
            sl.setSingleStep(step)
            sl.setTickInterval(step)
            sl.setTickPosition(QSlider.TicksBelow)
            sl.setValue(0)
            
            spin = QSpinBox()
            spin.setRange(lo, hi)
            spin.setSingleStep(step)
            spin.setSuffix(suffix)
            spin.setFixedWidth(80)
            spin.setFont(QFont("Consolas", 11, QFont.Bold))
            spin.setValue(0)
            
            row.addWidget(lbl_n); row.addWidget(sl); row.addWidget(spin)
            left_layout.addLayout(row)
            
            self.controls.append((sl, spin))
            
            # Sincronización bidireccional (Slider <-> SpinBox)
            sl.valueChanged.connect(lambda val, idx=i, s=spin: s.setValue(val))
            spin.valueChanged.connect(lambda val, idx=i, slider=sl: slider.setValue(val))
            
            # Mandar comando a ROS 2 SOLO cuando se suelta el mouse o se da "Enter" en la caja de texto
            sl.sliderReleased.connect(lambda idx=i: self._execute_movement(idx))
            spin.editingFinished.connect(lambda idx=i: self._execute_movement(idx))

        left_layout.addSpacing(10)
        
        gb_grip = QGroupBox("Efector Final (Gripper)")
        lay_grip = QHBoxLayout(gb_grip)
        
        btn_abierto = QPushButton("Abierto (80°)")
        btn_abierto.setStyleSheet("background:#e74c3c; color:white; height: 35px; font-weight:bold;")
        btn_abierto.clicked.connect(lambda: self._set_gripper_preset(80, "Abierto"))
        
        btn_pelota = QPushButton("Pelota (77°)")
        btn_pelota.setStyleSheet("background:#3498db; color:white; height: 35px; font-weight:bold;")
        btn_pelota.clicked.connect(lambda: self._set_gripper_preset(77, "Pelota"))
        
        btn_matraz = QPushButton("Matraz (31°)")
        btn_matraz.setStyleSheet("background:#9b59b6; color:white; height: 35px; font-weight:bold;")
        btn_matraz.clicked.connect(lambda: self._set_gripper_preset(31, "Matraz"))
        
        lay_grip.addWidget(btn_abierto)
        lay_grip.addWidget(btn_pelota)
        lay_grip.addWidget(btn_matraz)
        
        left_layout.addWidget(gb_grip)
        left_layout.addStretch()
        body_layout.addWidget(self.left_panel, stretch=6)

        right_panel = QGroupBox("Memoria de Trayectoria")
        right_layout = QVBoxLayout(right_panel)
        self.lista = QListWidget()
        self.lista.setStyleSheet("font-family: Consolas, monospace; font-size: 12px;")
        right_layout.addWidget(self.lista)
        body_layout.addWidget(right_panel, stretch=4) 

        main_layout.addLayout(body_layout)

        footer_layout = QHBoxLayout()
        footer_layout.setContentsMargins(0, 10, 0, 0)

        self.combo_acciones = QComboBox()
        self.combo_acciones.addItems(["Grabar Postura Actual", "Reproducir Secuencia", "Reproducir en Bucle", "Vaciar Memoria"])
        self.combo_acciones.setStyleSheet("height: 40px; font-size: 14px; padding-left: 10px;")
        footer_layout.addWidget(self.combo_acciones, stretch=3)

        self.spin = QSpinBox()
        self.spin.setPrefix("Ciclos: ")
        self.spin.setRange(1, 99); self.spin.setValue(5)
        self.spin.setStyleSheet("height: 40px; font-size: 14px;")
        footer_layout.addWidget(self.spin, stretch=1)

        self.btn_ejecutar = QPushButton("EJECUTAR")
        self.btn_ejecutar.setStyleSheet("background:#27ae60; color:white; height: 40px; font-weight:bold;")
        self.btn_ejecutar.clicked.connect(self._ejecutar_accion)
        footer_layout.addWidget(self.btn_ejecutar, stretch=2)

        footer_layout.addSpacing(40)

        btn_estop = QPushButton("PARO DE EMERGENCIA")
        btn_estop.setStyleSheet("background:#c0392b; color:white; height: 40px; font-weight:bold; border-radius: 4px;")
        btn_estop.clicked.connect(self._estop)
        footer_layout.addWidget(btn_estop, stretch=2)

        main_layout.addLayout(footer_layout)

    def _set_ui_state(self, enabled: bool):
        self.left_panel.setEnabled(enabled)
        self.combo_acciones.setEnabled(enabled)
        self.spin.setEnabled(enabled)
        self.btn_ejecutar.setEnabled(enabled)
        if enabled:
            self.lbl_tel.setStyleSheet("background-color: #1e272e; color: #0be881; padding: 8px; font-family: Consolas, monospace;")
        else:
            self.lbl_tel.setStyleSheet("background-color: #f1c40f; color: #2c3e50; padding: 8px; font-weight: bold; font-family: Consolas, monospace;")

    def _execute_movement(self, idx: int):
        """Función unificada: se llama al soltar el slider o terminar de escribir en el SpinBox."""
        # Se obtiene el valor del SpinBox ya que está sincronizado con el Slider
        raw_val = self.controls[idx][1].value()
        
        # Redondear al paso más cercano para forzar la resolución (por si escribieron un número intermedio)
        step = self.arm_configs[idx][3]
        raw_val = round(raw_val / step) * step
        self.controls[idx][1].setValue(raw_val) # Corrige visualmente
        
        if idx == 0:
            target_m = raw_val / 1000.0
            dist_m = abs(target_m - self.node.current_pos[0])
            self.target_positions[0] = target_m
            
            timeout_ms = max(500, int((dist_m / 0.02) * 1000))
            self.controls[0][0].setEnabled(False)
            self.controls[0][1].setEnabled(False)
            self.riel_timer.start(timeout_ms)
            
            self.node.send_single_point(self.target_positions, duration_sec=timeout_ms/1000.0)
            
        else:
            target_rad = math.radians(raw_val)
            self.target_positions[idx] = target_rad
            
            self.controls[idx][0].setEnabled(False)
            self.controls[idx][1].setEnabled(False)
            self.locked_joints.add(idx)
            
            if not self.arrival_timer.isActive():
                self.arrival_timer.start(100)
                
            self.node.send_single_point(self.target_positions, duration_sec=1.0)

    def _check_arrival(self):
        if not self.locked_joints:
            self.arrival_timer.stop()
            return
            
        current = self.node.current_pos
        arrived = []
        tolerance = math.radians(2.0)
        
        for idx in list(self.locked_joints):
            if abs(current[idx] - self.target_positions[idx]) <= tolerance:
                arrived.append(idx)
                
        for idx in arrived:
            self.locked_joints.remove(idx)
            self.controls[idx][0].setEnabled(True) 
            self.controls[idx][1].setEnabled(True)
            
        if not self.locked_joints:
            self.arrival_timer.stop()

    def _unlock_riel(self):
        self.controls[0][0].setEnabled(True)
        self.controls[0][1].setEnabled(True)

    def _set_gripper_preset(self, angle_deg: int, name: str):
        self.target_positions[4] = math.radians(angle_deg)
        self.node.send_single_point(self.target_positions, duration_sec=0.5)

    def _update_telemetry(self):
        cur = self.node.current_pos
        texto = "ESTADO DEL ROBOT:\n" if self.left_panel.isEnabled() else "PRECAUCIÓN: ROBOT EN MOVIMIENTO...\n"
        
        mov_riel = " ⏳" if not self.controls[0][0].isEnabled() else "  "
        texto += f"Riel Base: {cur[0]*1000:+.0f} mm{mov_riel} | "
        
        names = ['Cintura', 'Hombro', 'Codo']
        for i in range(1, 4):
            deg = math.degrees(cur[i])
            mov = " ⏳" if i in self.locked_joints else "  "
            texto += f"{names[i-1]:<8}: {deg:+.0f}°{mov} | "
            if i == 2: texto += "\n"
            
        texto += f"Gripper: {math.degrees(cur[4]):.0f}°"
        self.lbl_tel.setText(texto)

    def _ejecutar_accion(self):
        accion = self.combo_acciones.currentIndex()
        if accion == 0: self._grabar()
        elif accion == 1: self._play()
        elif accion == 2: self._bucle()
        elif accion == 3: self._borrar()

    def _go_home(self):
        self.target_positions = [0.0] * 5
        self.node.send_single_point(self.target_positions, duration_sec=2.0)
        
        for i, (sl, spin) in enumerate(self.controls):
            sl.blockSignals(True)
            spin.blockSignals(True)
            
            sl.setValue(0)
            spin.setValue(0)
            sl.setEnabled(True)
            spin.setEnabled(True)
            
            sl.blockSignals(False)
            spin.blockSignals(False)
            
        self.locked_joints.clear()
        self.riel_timer.stop()
        self.arrival_timer.stop()

    def _grabar(self):
        pos = self.target_positions.copy()
        self.node.waypoints.append(pos)
        short = " ".join(f"{v:+.2f}" for v in pos)
        self.lista.addItem(f"P{len(self.node.waypoints)}: [{short}]")

    def _play(self):
        duration = self.node.play_sequence(1)
        if duration > 0.0:
            self._set_ui_state(False)
            self.play_timer.start(int(duration * 1000))

    def _bucle(self):
        duration = self.node.play_sequence(self.spin.value())
        if duration > 0.0:
            self._set_ui_state(False)
            self.play_timer.start(int(duration * 1000))

    def _borrar(self):
        self.node.waypoints.clear()
        self.lista.clear()

    def _estop(self):
        self.play_timer.stop()
        self.arrival_timer.stop()
        self.riel_timer.stop()
        
        self.locked_joints.clear()
        self._set_ui_state(True)
        
        current = self.node.current_pos
        self.node.send_single_point(current, duration_sec=0.1) 
        self.target_positions = current.copy()
        
        for i, (sl, spin) in enumerate(self.controls):
            sl.blockSignals(True)
            spin.blockSignals(True)
            sl.setEnabled(True)
            spin.setEnabled(True)
            
            if i == 0:
                mm = int(current[0] * 1000)
                mm = round(mm / 8) * 8 # Ajustar a la grilla de 8mm
                sl.setValue(mm)
                spin.setValue(mm)
            else:
                deg = int(math.degrees(current[i]))
                deg = round(deg / 10) * 10 # Ajustar a la grilla de 10°
                sl.setValue(deg)
                spin.setValue(deg)
                
            sl.blockSignals(False)
            spin.blockSignals(False)
            
        QMessageBox.critical(self, "SISTEMA DETENIDO", "Paro de Emergencia accionado.\nActuadores mantienen postura actual.")

def main():
    rclpy.init(args=sys.argv)
    ros_node = TeachAndRepeatNode()
    threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True).start()
    app = QApplication(sys.argv)
    gui = TeachPendantGUI(ros_node)
    gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()