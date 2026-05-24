#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import time

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        # 1. Configurar el puerto USB
        # Usualmente el ESP32 aparece como ttyUSB0 o ttyACM0. 
        self.puerto_serial = '/dev/ttyUSB0' 
        self.baudrate = 115200
        
        try:
            self.arduino = serial.Serial(self.puerto_serial, self.baudrate, timeout=0.1)
            self.get_logger().info(f"Conectado al ESP32 en {self.puerto_serial}")
        except serial.SerialException:
            self.get_logger().error(f"No se pudo abrir {self.puerto_serial}. Revisa la conexión o los permisos.")
            self.arduino = None

        # 2. Suscribe a la telemetría del robot virtual
        self.sub = self.create_subscription(JointState, '/joint_states', self.state_cb, 10)
        self.joint_names = ['Motor_1', 'Motor_2', 'Motor_3']
        
        # Temporizador para limitar los envíos (50 Hz = 0.02 segundos)
        self.last_send_time = time.time()

    def state_cb(self, msg):
        if self.arduino is None:
            return

        # Control de frecuencia de envío
        current_time = time.time()
        if (current_time - self.last_send_time) < 0.02:
            return
        self.last_send_time = current_time

        # Extraer posiciones del mensaje asegurando el orden correcto
        posiciones = [0.0, 0.0, 0.0]
        try:
            for i, name in enumerate(self.joint_names):
                index = msg.name.index(name)
                posiciones[i] = msg.position[index]
        except ValueError:
            return # Si la trama está incompleta, la ignoramos por seguridad

        # 3. Empaquetar y disparar por el USB
        trama = f"<{posiciones[0]:.4f},{posiciones[1]:.4f},{posiciones[2]:.4f}>\n"
        self.arduino.write(trama.encode('utf-8'))

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.arduino and node.arduino.is_open:
            node.arduino.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()