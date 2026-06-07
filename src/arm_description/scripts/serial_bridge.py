#!/usr/bin/env python3
"""
serial_bridge.py  (v3)
======================
Fix aplicado: 'gripper_mecanismo' con doble 'p' para coincidir
con el nombre que publica /joint_states.

Flujo:
  /joint_states → construye <V1,V2,V3,V4,V5> → ESP32 (50 Hz)
  ESP32 → respuesta serial → /esp32/feedback (publica cada línea recibida)
"""

import time
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import serial


# ── Orden ESTRICTO que espera el ESP32 ───────────────────────────
# Asegúrate de que coincide exactamente con los joint_names
# que publica /joint_states en tu sistema.
JOINT_NAMES = [
    'moviento_en_x',            # V1 — riel lineal
    'disco_soportes_rotacion',  # V2 — hombro
    'brazo_motores',            # V3 — codo
    'brazo_gripper_azul',       # V4 — muñeca
    'gripper_mecanismo',        # V5 — gripper  ← 2 'p' (igual al URDF/joint_states)
]

PUERTO_SERIAL = '/dev/ttyUSB0'
BAUDRATE      = 115200
FREQ_HZ       = 10          # Hz de envío al ESP32
LOG_CADA      = 50          # imprime trama cada N ciclos para no saturar terminal


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        self.current_pos = {name: 0.0 for name in JOINT_NAMES}
        self._ciclo  = 0
        self._reconx = 0
        self.esp32   = None

        # Publisher de respuestas del ESP32
        self.feedback_pub = self.create_publisher(String, '/esp32/feedback', 10)

        # Suscriptor de /joint_states
        self.sub = self.create_subscription(
            JointState, '/joint_states', self._state_cb, 10
        )

        # Conectar al ESP32
        self._conectar()

        # Hilo lector de respuestas del ESP32 (daemon)
        threading.Thread(target=self._read_loop, daemon=True).start()

        # Timer de envío a 10 Hz
        self.timer = self.create_timer(1.0 / FREQ_HZ, self._enviar_trama)

        self.get_logger().info(
            f'serial_bridge listo. Escuchando /joint_states → {PUERTO_SERIAL} a {FREQ_HZ} Hz'
        )

    # ── Conexión ──────────────────────────────────────────────────
    def _conectar(self):
        try:
            if self.esp32 and self.esp32.is_open:
                self.esp32.close()
            self.esp32 = serial.Serial(PUERTO_SERIAL, BAUDRATE, timeout=0.1)
            self.get_logger().info(
                f'✅ ENLACE ESTABLECIDO: ESP32 en {PUERTO_SERIAL} a {BAUDRATE} baud'
            )
        except serial.SerialException:
            if self._reconx % 50 == 0:
                self.get_logger().error(
                    f'❌ SIN CONEXIÓN: esperando ESP32 en {PUERTO_SERIAL}. '
                    'Revisa el cable USB.'
                )
            self.esp32 = None

    # ── Callback: actualiza posiciones ───────────────────────────
    def _state_cb(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            if name in self.current_pos:
                self.current_pos[name] = pos

    # ── Envío de trama (50 Hz) ────────────────────────────────────
    def _enviar_trama(self) -> None:
        if self.esp32 is None or not self.esp32.is_open:
            self._reconx += 1
            if self._reconx >= 50:
                self._conectar()
                self._reconx = 0
            return

        p = [self.current_pos[n] for n in JOINT_NAMES]
        trama = f'<{p[0]:.4f},{p[1]:.4f},{p[2]:.4f},{p[3]:.4f},{p[4]:.4f}>\n'

        try:
            self.esp32.write(trama.encode('utf-8'))
        except serial.SerialException:
            self.get_logger().error('⚠️  DESCONEXIÓN durante el envío.')
            self.esp32.close()
            self.esp32 = None
            return

        self._ciclo += 1
        if self._ciclo % LOG_CADA == 0:
            self.get_logger().info(
                f'📤 → {trama.strip()} | '
                f'riel={p[0]:.3f}m  hombro={p[1]:.3f}rad  '
                f'codo={p[2]:.3f}rad  muñeca={p[3]:.3f}rad  '
                f'gripper={p[4]:.3f}rad'
            )

    # ── Lector de respuestas del ESP32 ───────────────────────────
    def _read_loop(self) -> None:
        while rclpy.ok():
            if self.esp32 and self.esp32.is_open:
                try:
                    raw = self.esp32.readline()
                    if raw:
                        text = raw.decode('utf-8', errors='replace').strip()
                        if text:
                            self.get_logger().info(f'📩 [ESP32→RPi] {text}')
                            msg = String()
                            msg.data = text
                            self.feedback_pub.publish(msg)
                except serial.SerialException:
                    pass
                except Exception as exc:
                    self.get_logger().debug(f'Read error: {exc}')
            time.sleep(0.005)


# ══════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.esp32 and node.esp32.is_open:
            node.esp32.close()
            node.get_logger().info('🔌 Puerto serial cerrado limpiamente.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
