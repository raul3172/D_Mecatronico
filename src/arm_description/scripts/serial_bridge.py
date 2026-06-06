#!/usr/bin/env python3
"""
serial_bridge.py  (v2)
======================
Puente entre ROS 2 y los actuadores Hiwonder via ESP32.

Novedades respecto a v1:
  - Hilo lector: captura respuestas del ESP32 y las publica en /esp32/feedback
  - Log detallado: muestra trama enviada y acuse del ESP32
  - Corrección: 'griper_mecanismo' (1 'p') para coincidir con el URDF/controlador
  - Reconexión automática si se pierde el USB
"""

import time
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import serial


# Orden ESTRICTO en que el ESP32 espera los 5 valores
JOINT_NAMES = [
    'moviento_en_x',            # V1 — riel lineal
    'disco_soportes_rotacion',  # V2 — hombro
    'brazo_motores',            # V3 — codo
    'brazo_gripper_azul',       # V4 — muñeca
    'griper_mecanismo',         # V5 — gripper  ← 1 sola 'p' (igual al URDF)
]

PUERTO_SERIAL = '/dev/ttyUSB0'
BAUDRATE      = 115200
FREQ_HZ       = 50          # frecuencia de envío
LOG_CADA      = 50          # imprime trama cada N ciclos (reduce spam)


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        # Estado interno: última posición conocida de cada joint
        self.current_pos = {name: 0.0 for name in JOINT_NAMES}
        self._ciclo      = 0
        self.esp32       = None
        self._reconx     = 0

        # Publisher para respuestas del ESP32
        self.feedback_pub = self.create_publisher(String, '/esp32/feedback', 10)

        # Suscriptor de telemetría
        self.sub = self.create_subscription(
            JointState, '/joint_states', self._state_cb, 10
        )

        # Conectar al ESP32
        self._conectar()

        # Hilo lector de respuestas ESP32 (daemon — muere con el proceso)
        self._read_thread = threading.Thread(
            target=self._read_loop, daemon=True
        )
        self._read_thread.start()

        # Timer de envío a 50 Hz
        self.timer = self.create_timer(1.0 / FREQ_HZ, self._enviar_trama)

    # ── Conexión serial ───────────────────────────────────────────
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
                    f'Revisa el cable USB.'
                )
            self.esp32 = None

    # ── Callback: actualiza posiciones desde /joint_states ────────
    def _state_cb(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            if name in self.current_pos:
                self.current_pos[name] = pos

    # ── Envío de trama al ESP32 (50 Hz) ──────────────────────────
    def _enviar_trama(self) -> None:
        # Reconexión si es necesario
        if self.esp32 is None or not self.esp32.is_open:
            self._reconx += 1
            if self._reconx >= 50:
                self._conectar()
                self._reconx = 0
            return

        # Construir trama <V1,V2,V3,V4,V5>
        p = [self.current_pos[n] for n in JOINT_NAMES]
        trama = f'<{p[0]:.4f},{p[1]:.4f},{p[2]:.4f},{p[3]:.4f},{p[4]:.4f}>\n'

        try:
            self.esp32.write(trama.encode('utf-8'))
        except serial.SerialException:
            self.get_logger().error('⚠️  DESCONEXIÓN durante el envío. Reconectando...')
            self.esp32.close()
            self.esp32 = None
            return

        # Log periódico (no cada ciclo para no saturar)
        self._ciclo += 1
        if self._ciclo % LOG_CADA == 0:
            self.get_logger().info(
                f'📤 Trama enviada → {trama.strip()}\n'
                f'   riel={p[0]:.4f}m | hombro={p[1]:.4f}rad | '
                f'codo={p[2]:.4f}rad | muñeca={p[3]:.4f}rad | '
                f'gripper={p[4]:.4f}rad'
            )

    # ── Lector de respuestas ESP32 (hilo paralelo) ────────────────
    def _read_loop(self) -> None:
        """
        Lee líneas del puerto serial en segundo plano.
        El ESP32 debe responder con líneas de texto, por ejemplo:
          "OK:<posición_alcanzada>"
          "ERR:<código>"
          "POSE_OK"
        Cada respuesta se publica en /esp32/feedback y se loggea.
        """
        while rclpy.ok():
            if self.esp32 and self.esp32.is_open:
                try:
                    raw = self.esp32.readline()   # timeout=0.1 s (no bloquea)
                    if raw:
                        text = raw.decode('utf-8', errors='replace').strip()
                        if text:
                            self.get_logger().info(f'📩 [ESP32→RPi] {text}')
                            msg = String()
                            msg.data = text
                            self.feedback_pub.publish(msg)
                except serial.SerialException:
                    pass   # la reconexión la maneja _enviar_trama
                except Exception as exc:
                    self.get_logger().debug(f'Read error: {exc}')
            time.sleep(0.005)   # 200 Hz de polling máximo en el hilo


# ══════════════════════════════════════════════════════════════════
# MAIN
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