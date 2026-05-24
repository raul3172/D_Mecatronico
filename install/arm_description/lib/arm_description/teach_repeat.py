#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import threading
import sys

class TeachAndRepeat(Node):
    def __init__(self):
        super().__init__('teach_and_repeat_node')
        # Nos suscribimos para "escuchar" dónde está el brazo
        self.sub = self.create_subscription(JointState, '/joint_states', self.state_cb, 10)
        # Publicamos para "mandar" la secuencia al controlador
        self.pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        
        self.current_pos = []
        self.joint_names = []
        self.waypoints = []

    def state_cb(self, msg):
        # Actualizamos la posición actual en tiempo real
        self.joint_names = msg.name
        self.current_pos = msg.position

    def record_waypoint(self):
        if not self.current_pos:
            self.get_logger().warning("Aún no hay datos de los motores...")
            return
        # Guardamos una copia exacta de los ángulos actuales
        self.waypoints.append(list(self.current_pos))
        print(f"\n[+] Punto {len(self.waypoints)} guardado con éxito! {self.current_pos}")

    def play_trajectory(self, loop_count=1):
        if not self.waypoints:
            print("\n[-] No has grabado ningún punto todavía.")
            return
            
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        tiempo_entre_puntos = 2 # Segundos que tarda en ir de un punto a otro
        tiempo_acumulado = 2
        
        # Armamos la coreografía completa
        for ciclo in range(loop_count):
            for wp in self.waypoints:
                point = JointTrajectoryPoint()
                point.positions = wp
                
                # --- MODIFICACIÓN CLAVE ---
                # Al forzar velocidad y aceleración a 0.0 en los extremos de cada punto, 
                # obligamos al JointTrajectoryController a usar un Spline Quíntico (5to orden)
                num_joints = len(self.joint_names)
                point.velocities = [0.0] * num_joints
                point.accelerations = [0.0] * num_joints
                # -------------------------
                
                point.time_from_start.sec = int(tiempo_acumulado)
                msg.points.append(point)
                tiempo_acumulado += tiempo_entre_puntos
        self.pub.publish(msg)
        print(f"\n[>] Ejecutando trayectoria ({loop_count} ciclos). Duración total: {tiempo_acumulado} segundos.")

def main(args=None):
    rclpy.init(args=args)
    node = TeachAndRepeat()
    
    # Creamos un hilo separado para el menú de la terminal para no bloquear ROS 2
    def menu_interactivo():
        print("\n--- MENÚ TEACH & REPEAT ---")
        print("[r] Grabar posición actual (Record)")
        print("[p] Reproducir secuencia 1 vez (Play)")
        print("[b] Reproducir en bucle 5 veces (Bucle)")
        print("[c] Borrar todos los puntos (Clear)")
        print("[q] Salir")
        
        while rclpy.ok():
            comando = input("\nIngresa comando: ").strip().lower()
            if comando == 'r':
                node.record_waypoint()
            elif comando == 'p':
                node.play_trajectory(loop_count=1)
            elif comando == 'b':
                node.play_trajectory(loop_count=5)
            elif comando == 'c':
                node.waypoints.clear()
                print("Memoria borrada.")
            elif comando == 'q':
                print("Cerrando programa...")
                break

    # Iniciamos el menú en segundo plano
    hilo_menu = threading.Thread(target=menu_interactivo, daemon=True)
    hilo_menu.start()
    
    try:
        # Mantenemos a ROS 2 escuchando los tópicos
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

if __name__ == '__main__':
    main()