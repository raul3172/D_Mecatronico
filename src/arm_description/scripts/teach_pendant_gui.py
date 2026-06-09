#!/usr/bin/env python3
import sys, threading, math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from PyQt5.QtWidgets import (
    QApplication, QWidget, QDialog, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QListWidget, QSpinBox, QProgressBar,
    QMessageBox, QSlider, QGroupBox, QComboBox
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont

ARM_TOPIC      = '/arm_controller/joint_trajectory'
GRIPPER_TOPIC  = '/gripper_controller/joint_trajectory'
ARM_JOINTS     = ['moviento_en_x','disco_soportes_rotacion','brazo_motores','brazo_gripper_azul']
GRIPPER_JOINTS = ['gripper_mecanismo']
ALL_JOINTS     = ARM_JOINTS + GRIPPER_JOINTS

class TeachAndRepeatNode(Node):
    def __init__(self):
        super().__init__('teach_pendant_node')
        self.sub    = self.create_subscription(JointState, '/joint_states',   self._state_cb, 10)
        self.sub_fb = self.create_subscription(String,     '/esp32/feedback', self._fb_cb,    10)
        self.pub_arm  = self.create_publisher(JointTrajectory, ARM_TOPIC,     10)
        self.pub_grip = self.create_publisher(JointTrajectory, GRIPPER_TOPIC, 10)
        
        self._pos_map: dict      = {j: 0.0 for j in ALL_JOINTS}
        self.waypoints: list     = []
        self.last_feedback: str  = ''

    def _state_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self._pos_map:
                self._pos_map[name] = pos

    def _fb_cb(self, msg):
        self.last_feedback = msg.data

    @property
    def current_pos(self):
        return [self._pos_map[j] for j in ALL_JOINTS]

    def send_single_point(self, positions: list, duration_sec: float = 1.0):
        if len(positions) < 5: return
        for pub, names, pos in [
            (self.pub_arm,  ARM_JOINTS,     positions[:4]),
            (self.pub_grip, GRIPPER_JOINTS, [positions[4]]),
        ]:
            msg = JointTrajectory()
            msg.joint_names = names
            pt = JointTrajectoryPoint()
            pt.positions = pos
            pt.time_from_start.sec     = int(duration_sec)
            pt.time_from_start.nanosec = int((duration_sec % 1) * 1e9)
            msg.points.append(pt)
            pub.publish(msg)

    def play_sequence(self, loop_count: int = 1) -> float:
        if not self.waypoints: return 0.0
        msg_arm  = JointTrajectory(); msg_arm.joint_names  = ARM_JOINTS
        msg_grip = JointTrajectory(); msg_grip.joint_names = GRIPPER_JOINTS
        t = 2.0
        for _ in range(loop_count):
            for wp in self.waypoints:
                for msg, pos, n in [(msg_arm, wp[:4], 4), (msg_grip, [wp[4]], 1)]:
                    pt = JointTrajectoryPoint()
                    pt.positions     = pos
                    pt.velocities    = [0.0] * n
                    pt.accelerations = [0.0] * n
                    pt.time_from_start.sec     = int(t)
                    pt.time_from_start.nanosec = int((t % 1) * 1e9)
                    msg.points.append(pt)
                t += 2.0
        self.pub_arm.publish(msg_arm)
        self.pub_grip.publish(msg_grip)
        return t - 2.0

# ══════════════════════════════════════════════════════════════════
# DIÁLOGO DE CALIBRACIÓN
# ══════════════════════════════════════════════════════════════════
class CalibrationDialog(QDialog):
    # Formato: (delay_ms, mensaje UI, wait_esp_calib)
    STEPS = [
        ( 800, "Conectando con ESP32...", False),
        (4500, "Riel → final de carrera (máximo)", False),
        ( 500, "Riel → inicio de carrera (cero). Esperando ESP32...", True),
        (2000, "Verificando motores Hiwonder (30°)", False),
        (2000, "Motores → posición home", False),
        (1500, "Gripper: Cerrado", False),
        (1500, "Gripper: Abierto Pelota (77°)", False),
        (1500, "Gripper: Abierto Matraz (31°)", False),
        ( 600, "Sistema listo. Iniciando control...", False),
    ]

    def __init__(self, node: TeachAndRepeatNode, parent=None):
        super().__init__(parent)
        self.node  = node
        self._step = 0
        self._pos  = [0.0] * 5
        self.setWindowTitle("Calibrando...")
        self.setFixedSize(500, 230)
        self.setWindowFlags(Qt.Dialog | Qt.FramelessWindowHint)
        self.setStyleSheet("background:#ecf0f1; border:2px solid #2c3e50; border-radius:10px;")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(24, 24, 24, 24)
        lay.setSpacing(12)
        
        title = QLabel("⚙  INICIALIZANDO SISTEMA")
        title.setFont(QFont("Arial", 14, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        lay.addWidget(title)
        
        self.lbl = QLabel("Espere, calibrando...")
        self.lbl.setAlignment(Qt.AlignCenter)
        self.lbl.setStyleSheet("font-size:13px; color:#2c3e50; padding:6px;")
        lay.addWidget(self.lbl)
        
        self.bar = QProgressBar()
        self.bar.setRange(0, len(self.STEPS) - 1)
        self.bar.setValue(0)
        self.bar.setTextVisible(False)
        self.bar.setStyleSheet("QProgressBar{border-radius:4px; background:#bdc3c7;} QProgressBar::chunk{background:#27ae60; border-radius:4px;}")
        self.bar.setFixedHeight(14)
        lay.addWidget(self.bar)
        
        self.lbl_fb = QLabel("Esperando respuesta ESP32...")
        self.lbl_fb.setAlignment(Qt.AlignCenter)
        self.lbl_fb.setStyleSheet("font-size:11px; color:#7f8c8d;")
        lay.addWidget(self.lbl_fb)
        
        self._fb_timer = QTimer()
        self._fb_timer.timeout.connect(
            lambda: self.lbl_fb.setText(f"ESP32: {self.node.last_feedback}") if self.node.last_feedback else None
        )
        self._fb_timer.start(250)
        QTimer.singleShot(400, self._advance)

    def _advance(self):
        # Si el paso anterior requería validación del ESP32
        if self._step > 0 and self.STEPS[self._step - 1][2]:
            if "CALIBRADO" not in self.node.last_feedback:
                QTimer.singleShot(200, self._advance) # Loop esperando
                return
            else:
                self.node.last_feedback = "" # Limpiar bandera

        if self._step >= len(self.STEPS):
            self._fb_timer.stop()
            self.accept()
            return

        delay, msg, wait_esp = self.STEPS[self._step]
        self.lbl.setText(msg)
        self.bar.setValue(self._step)
        self._execute_step(self._step)

        self._step += 1
        
        if wait_esp:
            QTimer.singleShot(500, self._advance) # Dar tiempo de enviar el cero antes de escuchar
        else:
            QTimer.singleShot(delay, self._advance)

    def _execute_step(self, step):
        p = self._pos[:]
        if step == 1:                          # Riel al máximo (0.46m)
            p[0] = 0.46
            self.node.send_single_point(p, 4.0)
        elif step == 2:                        # Riel a home (0.0m)
            p[0] = 0.0
            self.node.send_single_point(p, 4.0)
        elif step == 3:                        # Hiwonder test 30°
            r = math.radians(-1*30)
            p[1] = p[2] = p[3] = r
            self.node.send_single_point(p, 1.8)
            self._pos[1:4] = [r, r, r]
        elif step == 4:                        # Hiwonder home
            p[1] = p[2] = p[3] = 0.0
            self.node.send_single_point(p, 1.8)
            self._pos[1:4] = [0.0, 0.0, 0.0]
        elif step == 5:                        # Gripper cerrado
            p[4] = 0.0
            self.node.send_single_point(p, 1.2)
        elif step == 6:                        # Gripper pelota
            p[4] = math.radians(77)
            self.node.send_single_point(p, 1.2)
        elif step == 7:                        # Gripper matraz
            p[4] = math.radians(31)
            self.node.send_single_point(p, 1.2)

# ══════════════════════════════════════════════════════════════════
# GUI PRINCIPAL
# ══════════════════════════════════════════════════════════════════
class TeachPendantGUI(QWidget):
    def __init__(self, ros_node: TeachAndRepeatNode):
        super().__init__()
        self.node             = ros_node
        self.target_positions = [0.0] * 5
        self._rail_moving     = False
        self.arm_configs = [
            ("Riel Base",  0,    460, 8,  " mm"),
            ("Cintura",   -150,  150, 10, "°"),
            ("Hombro",    -150,  150, 10, "°"),
            ("Codo",      -150,  150, 10, "°"),
        ]
        self.controls: list[tuple[QSlider, QSpinBox]] = []
        self.gripper_btns = []
        self.locked_joints = set()
        
        self.play_timer = QTimer(); self.play_timer.setSingleShot(True)
        self.play_timer.timeout.connect(lambda: self._set_ui_state(True))
        
        self.arrival_timer = QTimer()
        self.arrival_timer.timeout.connect(self._check_arrival)
        
        self.riel_timer = QTimer(); self.riel_timer.setSingleShot(True)
        self.riel_timer.timeout.connect(self._release_rail_lock)
        
        self.telemetry_timer = QTimer()
        self.telemetry_timer.timeout.connect(self._update_telemetry)
        
        self._build_ui()
        self.telemetry_timer.start(100)

    def _build_ui(self):
        self.setWindowTitle('Teach Pendant - Operador Industrial')
        self.setMinimumSize(850, 500)
        main = QVBoxLayout(self)
        
        self.lbl_tel = QLabel("Iniciando telemetría...")
        self.lbl_tel.setStyleSheet("background:#1e272e; color:#0be881; padding:8px; font-family:Consolas,monospace; font-size:13px; border-radius:4px;")
        main.addWidget(self.lbl_tel)
        
        body = QHBoxLayout()
        # Panel izquierdo
        self.left_panel = QGroupBox("Control Manual de Ejes")
        left = QVBoxLayout(self.left_panel)
        self.btn_home = QPushButton("IR A HOME (0, 0, 0, 0)")
        self.btn_home.setStyleSheet("background:#f39c12; color:white; height:35px; font-weight:bold;")
        self.btn_home.clicked.connect(self._go_home)
        left.addWidget(self.btn_home)
        left.addSpacing(8)
        
        for i, (label, lo, hi, step, suffix) in enumerate(self.arm_configs):
            row = QHBoxLayout()
            lbl = QLabel(label); lbl.setFixedWidth(80); lbl.setFont(QFont("Arial", 10, QFont.Bold))
            sl   = QSlider(Qt.Horizontal)
            sl.setRange(lo, hi); sl.setSingleStep(step)
            sl.setTickInterval(step); sl.setTickPosition(QSlider.TicksBelow); sl.setValue(0)
            spin = QSpinBox()
            spin.setRange(lo, hi); spin.setSingleStep(step)
            spin.setSuffix(suffix); spin.setFixedWidth(80)
            spin.setFont(QFont("Consolas", 11, QFont.Bold)); spin.setValue(0)
            
            sl.valueChanged.connect(lambda v, s=spin: s.setValue(v))
            spin.valueChanged.connect(lambda v, s=sl: s.setValue(v))
            sl.sliderReleased.connect(lambda idx=i: self._execute_movement(idx))
            spin.editingFinished.connect(lambda idx=i: self._execute_movement(idx))
            row.addWidget(lbl); row.addWidget(sl); row.addWidget(spin)
            left.addLayout(row)
            self.controls.append((sl, spin))
            
        left.addSpacing(10)
        
        gb_grip = QGroupBox("Efector Final (Gripper)")
        lay_grip = QHBoxLayout(gb_grip)
        for text, color, deg in [("Abierto (80°)", "#e74c3c", 80), ("Pelota (77°)", "#3498db", 77), ("Matraz (31°)", "#9b59b6", 31)]:
            btn = QPushButton(text)
            btn.setStyleSheet(f"background:{color}; color:white; height:35px; font-weight:bold;")
            btn.clicked.connect(lambda _, d=deg: self._set_gripper_preset(d))
            self.gripper_btns.append(btn)
            lay_grip.addWidget(btn)
        left.addWidget(gb_grip)
        left.addStretch()
        body.addWidget(self.left_panel, stretch=6)
        
        # Panel derecho
        right_panel = QGroupBox("Memoria de Trayectoria")
        right = QVBoxLayout(right_panel)
        self.lista = QListWidget()
        self.lista.setStyleSheet("font-family:Consolas,monospace; font-size:12px;")
        right.addWidget(self.lista)
        body.addWidget(right_panel, stretch=4)
        main.addLayout(body)
        
        # Footer
        footer = QHBoxLayout(); footer.setContentsMargins(0, 10, 0, 0)
        self.combo = QComboBox()
        self.combo.addItems(["Grabar Postura Actual","Reproducir Secuencia","Reproducir en Bucle","Vaciar Memoria"])
        self.combo.setStyleSheet("height:40px; font-size:14px; padding-left:10px;")
        footer.addWidget(self.combo, stretch=3)
        self.spin_loops = QSpinBox()
        self.spin_loops.setPrefix("Ciclos: "); self.spin_loops.setRange(1, 99); self.spin_loops.setValue(5)
        self.spin_loops.setStyleSheet("height:40px; font-size:14px;")
        footer.addWidget(self.spin_loops, stretch=1)
        self.btn_exec = QPushButton("EJECUTAR")
        self.btn_exec.setStyleSheet("background:#27ae60; color:white; height:40px; font-weight:bold;")
        self.btn_exec.clicked.connect(self._ejecutar_accion)
        footer.addWidget(self.btn_exec, stretch=2)
        footer.addSpacing(40)
        
        btn_estop = QPushButton("PARO DE EMERGENCIA")
        btn_estop.setStyleSheet("background:#c0392b; color:white; height:40px; font-weight:bold; border-radius:4px;")
        btn_estop.clicked.connect(self._estop)
        footer.addWidget(btn_estop, stretch=2)
        main.addLayout(footer)

    def _set_ui_state(self, enabled: bool):
        # Esto apaga completamente todos los motores e inputs (pero mantiene E-STOP operativo)
        self.left_panel.setEnabled(enabled)
        self.combo.setEnabled(enabled)
        self.spin_loops.setEnabled(enabled)
        self.btn_exec.setEnabled(enabled)
        color = "#1e272e" if enabled else "#f1c40f"
        txt   = "#0be881" if enabled else "#2c3e50"
        self.lbl_tel.setStyleSheet(f"background:{color}; color:{txt}; padding:8px; font-family:Consolas,monospace; font-size:13px; border-radius:4px;")

    def _set_all_except_rail(self, enabled: bool):
        # Apaga el panel de ejecución y los demás motores mientras el riel se desplaza
        for i in range(1, 4):
            self.controls[i][0].setEnabled(enabled)
            self.controls[i][1].setEnabled(enabled)
        for btn in self.gripper_btns:
            btn.setEnabled(enabled)
        self.btn_home.setEnabled(enabled)
        self.combo.setEnabled(enabled)
        self.btn_exec.setEnabled(enabled)

    def _execute_movement(self, idx: int):
        step = self.arm_configs[idx][3]
        raw  = self.controls[idx][1].value()
        raw  = round(raw / step) * step
        self.controls[idx][1].setValue(raw)
        
        if idx == 0:
            target_m = raw / 1000.0
            dist_m   = abs(target_m - self.node.current_pos[0])
            self.target_positions[0] = target_m
            
            hold = list(self.target_positions)
            self.node.send_single_point(hold, duration_sec=0.05)
            
            timeout_ms = max(800, int((dist_m / 0.02) * 1000))
            self._rail_moving = True
            self._set_all_except_rail(False) # Bloqueo absoluto a otros componentes
            self.controls[0][0].setEnabled(False)
            self.controls[0][1].setEnabled(False)
            
            self.riel_timer.start(timeout_ms)
            self.node.send_single_point(self.target_positions, duration_sec=timeout_ms / 1000.0)
        else:
            if self._rail_moving:
                self.controls[idx][1].setValue(round(math.degrees(self.target_positions[idx]) / step) * step)
                return
            target_rad = math.radians(raw)
            self.target_positions[idx] = target_rad
            self.controls[idx][0].setEnabled(False)
            self.controls[idx][1].setEnabled(False)
            self.locked_joints.add(idx)
            if not self.arrival_timer.isActive():
                self.arrival_timer.start(100)
            self.node.send_single_point(self.target_positions, duration_sec=1.0)

    def _release_rail_lock(self):
        self._rail_moving = False
        self._set_all_except_rail(True)
        self.controls[0][0].setEnabled(True)
        self.controls[0][1].setEnabled(True)

    def _check_arrival(self):
        if not self.locked_joints:
            self.arrival_timer.stop(); return
        current   = self.node.current_pos
        tolerance = math.radians(2.0)
        arrived   = [i for i in self.locked_joints if abs(current[i] - self.target_positions[i]) <= tolerance]
        for i in arrived:
            self.locked_joints.remove(i)
            self.controls[i][0].setEnabled(True)
            self.controls[i][1].setEnabled(True)
        if not self.locked_joints:
            self.arrival_timer.stop()

    def _set_gripper_preset(self, angle_deg: int):
        self.target_positions[4] = math.radians(angle_deg)
        self.node.send_single_point(self.target_positions, duration_sec=0.5)

    def _update_telemetry(self):
        cur   = self.node.current_pos
        rail_icon = "⏳" if self._rail_moving else " "
        moving    = "PRECAUCIÓN: ROBOT EN MOVIMIENTO..." if not self.left_panel.isEnabled() else "ESTADO DEL ROBOT:"
        texto  = f"{moving}\n"
        texto += f"Riel:{cur[0]*1000:+.0f}mm{rail_icon}  "
        names  = ['Cintura','Hombro','Codo']
        for i in range(1, 4):
            icon = "⏳" if i in self.locked_joints else " "
            texto += f"{names[i-1]}:{math.degrees(cur[i]):+.0f}°{icon}  "
        texto += f"\nGripper:{math.degrees(cur[4]):.0f}°"
        self.lbl_tel.setText(texto)

    def _ejecutar_accion(self):
        acc = self.combo.currentIndex()
        if   acc == 0: self._grabar()
        elif acc == 1: self._play(1)
        elif acc == 2: self._play(self.spin_loops.value())
        elif acc == 3: self._borrar()

    def _go_home(self):
        self.target_positions = [0.0] * 5
        self.node.send_single_point(self.target_positions, duration_sec=2.0)
        self._rail_moving = False
        self.locked_joints.clear()
        self.riel_timer.stop(); self.arrival_timer.stop()
        for i, (sl, sp) in enumerate(self.controls):
            sl.blockSignals(True); sp.blockSignals(True)
            sl.setValue(0); sp.setValue(0)
            sl.setEnabled(True); sp.setEnabled(True)
            sl.blockSignals(False); sp.blockSignals(False)

    def _grabar(self):
        pos = self.target_positions.copy()
        self.node.waypoints.append(pos)
        short = " ".join(f"{v:+.2f}" for v in pos)
        self.lista.addItem(f"P{len(self.node.waypoints)}: [{short}]")

    def _play(self, loops: int):
        duration = self.node.play_sequence(loops)
        if duration > 0.0:
            self._set_ui_state(False) # Solo el E-STOP queda funcional
            self.play_timer.start(int(duration * 1000))
        else:
            QMessageBox.warning(self, "Sin puntos", "Graba al menos una postura primero.")

    def _borrar(self):
        self.node.waypoints.clear(); self.lista.clear()

    def _estop(self):
        self.play_timer.stop(); self.arrival_timer.stop(); self.riel_timer.stop()
        self._rail_moving = False; self.locked_joints.clear()
        self._set_ui_state(True)
        current = self.node.current_pos
        self.node.send_single_point(current, duration_sec=0.1)
        self.target_positions = list(current)
        for i, (sl, sp) in enumerate(self.controls):
            sl.blockSignals(True); sp.blockSignals(True)
            sl.setEnabled(True);   sp.setEnabled(True)
            if i == 0:
                mm = round(int(current[0] * 1000) / 8) * 8
                sl.setValue(mm); sp.setValue(mm)
            else:
                deg = round(int(math.degrees(current[i])) / 10) * 10
                sl.setValue(deg); sp.setValue(deg)
            sl.blockSignals(False); sp.blockSignals(False)
        QMessageBox.critical(self, "SISTEMA DETENIDO", "Paro de Emergencia accionado.\nMotores detenidos.")

def main():
    rclpy.init(args=sys.argv)
    ros_node = TeachAndRepeatNode()
    threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True).start()
    app = QApplication(sys.argv)
    
    calib = CalibrationDialog(ros_node)
    calib.exec_()
    
    gui = TeachPendantGUI(ros_node)
    gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()