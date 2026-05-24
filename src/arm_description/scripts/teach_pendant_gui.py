#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import threading

from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QLabel, QListWidget, QSpinBox, QMessageBox, QSlider)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont

class TeachAndRepeatNode(Node):
    def __init__(self):
        super().__init__('teach_pendant_node')
        self.sub = self.create_subscription(JointState, '/joint_states', self.state_cb, 10)
        self.pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        
        self.current_pos = [0.0, 0.0, 0.0]
        self.joint_names = ['Motor_1', 'Motor_2', 'Motor_3']
        self.waypoints = []

    def state_cb(self, msg):
        # Mapear los nombres a los índices correctos si ROS los envía en desorden
        temp_pos = [0.0, 0.0, 0.0]
        for i, name in enumerate(msg.name):
            if name == 'Motor_1': temp_pos[0] = msg.position[i]
            elif name == 'Motor_2': temp_pos[1] = msg.position[i]
            elif name == 'Motor_3': temp_pos[2] = msg.position[i]
        self.current_pos = temp_pos

    def send_single_point(self, positions):
        """Envía una posición inmediata al robot"""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.nanosec = 100000000 # 0.1 segundos (respuesta rápida)
        msg.points.append(point)
        self.pub.publish(msg)

    def play_sequence(self, loop_count):
        if not self.waypoints: return False
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        t = 2.0
        for _ in range(loop_count):
            for wp in self.waypoints:
                point = JointTrajectoryPoint()
                point.positions = wp
                # Perfil suave de 5to orden
                point.velocities = [0.0] * 3
                point.accelerations = [0.0] * 3
                point.time_from_start.sec = int(t)
                msg.points.append(point)
                t += 2.0
        self.pub.publish(msg)
        return True

class TeachPendantGUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.initUI()
        self.timer = QTimer(); self.timer.timeout.connect(self.update_telemetry); self.timer.start(100)

    def initUI(self):
        self.setWindowTitle('Teach Pendant - STS3215 Control')
        self.resize(500, 800)
        layout_principal = QVBoxLayout()

        # --- SECCIÓN 1: TELEMETRÍA ---
        self.lbl_estado = QLabel("Ángulos Actuales: [0.0, 0.0, 0.0]")
        self.lbl_estado.setStyleSheet("background-color: #2c3e50; color: #ecf0f1; padding: 15px; font-family: Courier; font-size: 14px; border-radius: 5px;")
        layout_principal.addWidget(self.lbl_estado)

        # --- SECCIÓN 2: CONTROL MANUAL (SLIDERS) ---
        layout_principal.addWidget(QLabel("\nCONTROL MANUAL DE JUNTAS"))
        self.sliders = []
        for i in range(3):
            h_layout = QHBoxLayout()
            lbl = QLabel(f"Motor {i+1}:")
            lbl.setFixedWidth(60)
            slider = QSlider(Qt.Horizontal)
            slider.setRange(-260, 260) # Representa -2.6 a 2.6 radianes (rango STS3215)
            slider.setValue(0)
            slider.valueChanged.connect(self.slider_moved)
            val_lbl = QLabel("0.00")
            val_lbl.setFixedWidth(50)
            
            h_layout.addWidget(lbl); h_layout.addWidget(slider); h_layout.addWidget(val_lbl)
            layout_principal.addLayout(h_layout)
            self.sliders.append((slider, val_lbl))

        # --- SECCIÓN 3: LISTA DE WAYPOINTS ---
        layout_principal.addWidget(QLabel("\nLISTA DE PASOS GRABADOS"))
        self.lista_puntos = QListWidget()
        layout_principal.addWidget(self.lista_puntos)

        # --- SECCIÓN 4: BOTONES DE ACCIÓN ---
        btn_layout = QHBoxLayout()
        self.btn_grabar = QPushButton("GRABAR"); self.btn_grabar.clicked.connect(self.grabar_punto)
        self.btn_grabar.setStyleSheet("background-color: #3498db; color: white; height: 50px; font-weight: bold;")
        
        self.btn_play = QPushButton("REPRODUCIR"); self.btn_play.clicked.connect(self.reproducir)
        self.btn_play.setStyleSheet("background-color: #2ecc71; color: white; height: 50px; font-weight: bold;")
        
        btn_layout.addWidget(self.btn_grabar); btn_layout.addWidget(self.btn_play)
        layout_principal.addLayout(btn_layout)

        self.btn_borrar = QPushButton("🗑 BORRAR TODO"); self.btn_borrar.clicked.connect(self.borrar_memoria)
        self.btn_borrar.setStyleSheet("background-color: #e74c3c; color: white; font-weight: bold;")
        layout_principal.addWidget(self.btn_borrar)

        self.setLayout(layout_principal)

    def slider_moved(self):
        """Lee los sliders y envía la posición al robot en tiempo real"""
        posiciones = []
        for slider, lbl in self.sliders:
            val_rad = slider.value() / 100.0
            lbl.setText(f"{val_rad:.2f}")
            posiciones.append(val_rad)
        self.ros_node.send_single_point(posiciones)

    def update_telemetry(self):
        p = self.ros_node.current_pos
        self.lbl_estado.setText(f"M1: {p[0]:.3f} | M2: {p[1]:.3f} | M3: {p[2]:.3f}")

    def grabar_punto(self):
        p = list(self.ros_node.current_pos)
        self.ros_node.waypoints.append(p)
        self.lista_puntos.addItem(f"Punto {len(self.ros_node.waypoints)}: {p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}")

    def reproducir(self):
        if not self.ros_node.play_sequence(1):
            QMessageBox.warning(self, "Error", "Graba algunos puntos primero.")

    def borrar_memoria(self):
        self.ros_node.waypoints.clear(); self.lista_puntos.clear()

def main():
    rclpy.init(args=sys.argv)
    ros_node = TeachAndRepeatNode()
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True); ros_thread.start()
    app = QApplication(sys.argv); gui = TeachPendantGUI(ros_node); gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()