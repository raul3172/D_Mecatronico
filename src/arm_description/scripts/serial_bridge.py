#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import time

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        self.puerto_serial = '/dev/ttyUSB0' 
        self.baudrate = 115200 
        self.esp32 = None
        self.intentos_reconex = 0
        
        # El orden ESTRICTO en que el ESP32 espera los datos
        self.joint_names = [
            'moviento_en_x', 
            'disco_soportes_rotacion', 
            'brazo_motores', 
            'brazo_gripper_azul',
            'gripper_mecanismo'
        ]
        
        # Memoria interna para guardar la última posición conocida de cada motor
        self.current_pos = {name: 0.0 for name in self.joint_names}

        self.conectar_serial()

        # Nos suscribimos a la telemetría global
        self.sub = self.create_subscription(JointState, '/joint_states', self.state_cb, 10)
        
        # Reloj Maestro: Ejecuta la función de envío exactamente a 50 Hz (0.02 segundos)
        self.timer = self.create_timer(0.02, self.enviar_trama)

    def conectar_serial(self):
        try:
            if self.esp32 and self.esp32.is_open:
                self.esp32.close()
            self.esp32 = serial.Serial(self.puerto_serial, self.baudrate, timeout=0.1)
            self.get_logger().info(f"✅ ENLACE ESTABLECIDO: ESP32 conectado en {self.puerto_serial}")
        except serial.SerialException:
            # Solo imprime el error cada cierto tiempo para no saturar la terminal
            if self.intentos_reconex % 50 == 0:
                self.get_logger().error(f"❌ ENLACE PERDIDO: Esperando al ESP32 en {self.puerto_serial}... revisa el cable USB.")
            self.esp32 = None

    def state_cb(self, msg):
        # Actualizamos la memoria interna SOLAMENTE con los motores que vengan en este mensaje
        for i, name in enumerate(msg.name):
            if name in self.current_pos:
                self.current_pos[name] = msg.position[i]

    def enviar_trama(self):
        # 1. Si no hay conexión, intenta reconectar sin bloquear a ROS 2
        if self.esp32 is None or not self.esp32.is_open:
            self.intentos_reconex += 1
            if self.intentos_reconex >= 50: # Intenta cada 1 segundo (50 ciclos de 0.02s)
                self.conectar_serial()
                self.intentos_reconex = 0
            return

        # 2. Extraer las posiciones de la memoria en el orden exacto para el ESP32
        p = [self.current_pos[name] for name in self.joint_names]

        # 3. Empaquetar en formato <V1,V2,V3,V4,V5>\n
        trama = f"<{p[0]:.4f},{p[1]:.4f},{p[2]:.4f},{p[3]:.4f},{p[4]:.4f}>\n"
        
        # 4. Disparar por el puerto serial
        try:
            self.esp32.write(trama.encode('utf-8'))
        except serial.SerialException:
            self.get_logger().error("⚠️ DESCONEXIÓN FÍSICA DETECTADA durante el envío.")
            self.esp32.close()
            self.esp32 = None

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
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()