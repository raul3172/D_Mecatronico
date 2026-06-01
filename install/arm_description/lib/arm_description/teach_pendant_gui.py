#!/usr/bin/env python3
"""
teach_pendant_gui.py  —  GUI para Diseño_Final_Brazo_carro_5 (Versión Segura)
=============================================================================
Controladores separados:
  arm_controller (4 joints)
  gripper_controller (1 joint)
"""

import sys
import threading

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

# LÍMITES DE SOFTWARE (ANTI-COLISIONES)
JOINT_CONFIG = [
    ('Riel Base',         0,    30,  'm  ', 100),
    ('Artic. Hombro',  -157,   157,  'rad', 100),
    ('Artic. Codo',    -157,   157,  'rad', 100),
    ('Artic. Muñeca',  -157,   157,  'rad', 100),
]


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

    def send_single_point(self, positions: list[float]) -> None:
        if len(positions) < 5:
            return

        msg_arm = JointTrajectory()
        msg_arm.joint_names = ARM_JOINTS
        pt_arm = JointTrajectoryPoint()
        pt_arm.positions = positions[:4] 
        pt_arm.time_from_start.nanosec = 100_000_000
        msg_arm.points.append(pt_arm)
        self.pub_arm.publish(msg_arm)

        msg_grip = JointTrajectory()
        msg_grip.joint_names = GRIPPER_JOINTS
        pt_grip = JointTrajectoryPoint()
        pt_grip.positions = [positions[4]]
        pt_grip.time_from_start.nanosec = 100_000_000
        msg_grip.points.append(pt_grip)
        self.pub_grip.publish(msg_grip)

    def play_sequence(self, loop_count: int = 1) -> float:
        """Devuelve el tiempo total (en segundos) que tardará la trayectoria."""
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
        
        return (t - 2.0) # Retorna la duración exacta de la maniobra


class TeachPendantGUI(QWidget):
    def __init__(self, ros_node: TeachAndRepeatNode):
        super().__init__()
        self.node = ros_node
        self.gripper_val = 0.0
        self.sliders: list[tuple[QSlider, QLabel, int]] = []
        
        # Timer para rehabilitar la interfaz cuando termina la secuencia
        self.play_timer = QTimer()
        self.play_timer.setSingleShot(True)
        self.play_timer.timeout.connect(lambda: self._set_ui_state(True))
        
        self._build_ui()
        
        self.telemetry_timer = QTimer()
        self.telemetry_timer.timeout.connect(self._update_telemetry)
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
        
        # Contenedor Izquierdo (Convertido a atributo de clase para poder bloquearlo)
        self.left_panel = QGroupBox("Control Manual de Ejes")
        left_layout = QVBoxLayout(self.left_panel)
        
        # BOTÓN HOME
        self.btn_home = QPushButton("IR A HOME (0, 0, 0, 0, 0)")
        self.btn_home.setStyleSheet("background:#f39c12; color:white; height: 35px; font-weight:bold;")
        self.btn_home.clicked.connect(self._go_home)
        left_layout.addWidget(self.btn_home)
        left_layout.addSpacing(10)
        
        for label, lo, hi, unit, div in JOINT_CONFIG:
            row = QHBoxLayout()
            lbl_n = QLabel(label); lbl_n.setFixedWidth(100); lbl_n.setFont(QFont("Arial", 10, QFont.Bold))
            sl = QSlider(Qt.Horizontal)
            sl.setRange(lo, hi); sl.setValue(0)
            lbl_v = QLabel("0.000"); lbl_v.setFixedWidth(50)
            lbl_u = QLabel(unit);   lbl_u.setFixedWidth(30)
            row.addWidget(lbl_n); row.addWidget(sl)
            row.addWidget(lbl_v); row.addWidget(lbl_u)
            left_layout.addLayout(row)
            
            self.sliders.append((sl, lbl_v, div))
            sl.valueChanged.connect(self._slider_moved)

        lay_grip = QHBoxLayout()
        lbl_grip = QLabel("Efector Final"); lbl_grip.setFixedWidth(100); lbl_grip.setFont(QFont("Arial", 10, QFont.Bold))
        
        btn_abrir = QPushButton("ABRIR")
        btn_abrir.setStyleSheet("background:#95a5a6; color:white; height: 35px; font-weight:bold;")
        btn_abrir.clicked.connect(lambda: self._set_gripper(1.0))
        
        btn_cerrar = QPushButton("CERRAR")
        btn_cerrar.setStyleSheet("background:#2c3e50; color:white; height: 35px; font-weight:bold;")
        btn_cerrar.clicked.connect(lambda: self._set_gripper(0.0))
        
        lay_grip.addWidget(lbl_grip); lay_grip.addWidget(btn_abrir); lay_grip.addWidget(btn_cerrar)
        left_layout.addLayout(lay_grip)
        left_layout.addStretch()
        body_layout.addWidget(self.left_panel, stretch=6)

        # Contenedor Derecho
        right_panel = QGroupBox("Memoria de Trayectoria")
        right_layout = QVBoxLayout(right_panel)
        self.lista = QListWidget()
        self.lista.setStyleSheet("font-family: Consolas, monospace; font-size: 12px;")
        right_layout.addWidget(self.lista)
        body_layout.addWidget(right_panel, stretch=4) 

        main_layout.addLayout(body_layout)

        # Barra de Acciones Inferior
        footer_layout = QHBoxLayout()
        footer_layout.setContentsMargins(0, 10, 0, 0)

        self.combo_acciones = QComboBox()
        self.combo_acciones.addItems([
            "Grabar Postura Actual", 
            "Reproducir Secuencia", 
            "Reproducir en Bucle", 
            "Vaciar Memoria"
        ])
        self.combo_acciones.setStyleSheet("height: 40px; font-size: 14px; padding-left: 10px;")
        footer_layout.addWidget(self.combo_acciones, stretch=3)

        self.spin = QSpinBox()
        self.spin.setPrefix("Ciclos: ")
        self.spin.setRange(1, 99); self.spin.setValue(5)
        self.spin.setStyleSheet("height: 40px; font-size: 14px;")
        footer_layout.addWidget(self.spin, stretch=1)

        self.btn_ejecutar = QPushButton("EJECUTAR")
        self.btn_ejecutar.setStyleSheet("background:#27ae60; color:white; height: 40px; font-weight:bold; font-size: 14px;")
        self.btn_ejecutar.clicked.connect(self._ejecutar_accion)
        footer_layout.addWidget(self.btn_ejecutar, stretch=2)

        footer_layout.addSpacing(40)

        # Paro de Emergencia SIEMPRE activo
        btn_estop = QPushButton("PARO DE EMERGENCIA")
        btn_estop.setStyleSheet("background:#c0392b; color:white; height: 40px; font-weight:bold; font-size: 14px; border-radius: 4px;")
        btn_estop.clicked.connect(self._estop)
        footer_layout.addWidget(btn_estop, stretch=2)

        main_layout.addLayout(footer_layout)

    def _set_ui_state(self, enabled: bool):
        """Bloquea o desbloquea los controles manuales por seguridad."""
        self.left_panel.setEnabled(enabled)
        self.combo_acciones.setEnabled(enabled)
        self.spin.setEnabled(enabled)
        self.btn_ejecutar.setEnabled(enabled)
        if enabled:
            self.lbl_tel.setStyleSheet(
                "background-color: #1e272e; color: #0be881; padding: 8px;"
                "font-family: Consolas, monospace; font-size: 13px; border-radius: 4px;"
            )
        else:
            self.lbl_tel.setStyleSheet(
                "background-color: #f1c40f; color: #2c3e50; padding: 8px; font-weight: bold;"
                "font-family: Consolas, monospace; font-size: 13px; border-radius: 4px;"
            )

    def _ejecutar_accion(self):
        accion = self.combo_acciones.currentIndex()
        if accion == 0:    
            self._grabar()
        elif accion == 1:  
            self._play()
        elif accion == 2:  
            self._bucle()
        elif accion == 3:  
            self._borrar()

    def _go_home(self):
        home_pos = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.node.send_single_point(home_pos)
        self.gripper_val = 0.0
        
        for i, (sl, lbl, div) in enumerate(self.sliders):
            sl.blockSignals(True)
            sl.setValue(0)
            lbl.setText("+0.000")
            sl.blockSignals(False)

    def _set_gripper(self, val):
        self.gripper_val = val
        self._slider_moved()

    def _slider_moved(self):
        if len(self.sliders) < 4:
            return
            
        pos = []
        for sl, lbl, div in self.sliders:
            val = sl.value() / div
            lbl.setText(f"{val:+.3f}")
            pos.append(val)
        pos.append(self.gripper_val)
        self.node.send_single_point(pos)

    def _update_telemetry(self):
        cur = self.node.current_pos
        units = ['m  ', 'rad', 'rad', 'rad', 'val']
        texto = "ESTADO DEL ROBOT:\n" if self.left_panel.isEnabled() else "PRECAUCIÓN: ROBOT EN MOVIMIENTO AUTÓNOMO...\n"
        for i, (name, val, unit) in enumerate(zip(ALL_JOINTS, cur, units)):
            texto += f"{name[:15]:<15}: {val:+.3f} {unit} | "
            if i == 2: 
                texto += "\n"
        self.lbl_tel.setText(texto)

    def _grabar(self):
        # Toma los datos directamente de la Interfaz, NO de la telemetría, 
        # garantizando que capte si el gripper se acaba de abrir/cerrar.
        pos = []
        for sl, lbl, div in self.sliders:
            pos.append(sl.value() / div)
        pos.append(self.gripper_val)
        
        self.node.waypoints.append(pos)
        short = " ".join(f"{v:+.2f}" for v in pos)
        self.lista.addItem(f"P{len(self.node.waypoints)}: [{short}]")

    def _play(self):
        duration = self.node.play_sequence(1)
        if duration == 0.0:
            QMessageBox.warning(self, "Memoria Vacía", "Agregue al menos un waypoint a la lista.")
        else:
            self._set_ui_state(False)
            self.play_timer.start(int(duration * 1000))

    def _bucle(self):
        cycles = self.spin.value()
        duration = self.node.play_sequence(cycles)
        if duration == 0.0:
            QMessageBox.warning(self, "Memoria Vacía", "Agregue al menos un waypoint a la lista.")
        else:
            self._set_ui_state(False)
            self.play_timer.start(int(duration * 1000))

    def _borrar(self):
        self.node.waypoints.clear()
        self.lista.clear()

    def _estop(self):
        self.play_timer.stop() # Cancela el temporizador de bloqueo
        self._set_ui_state(True) # Devuelve el control manual inmediatamente
        
        current = self.node.current_pos
        self.node.send_single_point(current) # Frena en seco enviando la posición real
        
        # Actualiza los sliders a la posición donde el robot frenó
        for i, (sl, lbl, div) in enumerate(self.sliders):
            sl.blockSignals(True)
            sl.setValue(int(current[i] * div))
            lbl.setText(f"{current[i]:+.3f}")
            sl.blockSignals(False)
        self.gripper_val = current[4]
        QMessageBox.critical(self, "SISTEMA DETENIDO", "Se ha accionado el Paro de Emergencia.\nLos actuadores mantienen la postura actual.")

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