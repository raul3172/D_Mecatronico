#!/usr/bin/env python3
"""
teach_repeat.py  (v3)
=====================
Control individual de actuadores Hiwonder via ESP32.

Flujo por motor:
  Selección → Ingreso de pose (rad o grados) → Vista previa de trama ESP32
  → Envío al controlador → Barra de progreso (bloqueante)
  → Confirmación de éxito / timeout → Menú de nuevo

Incluye además: grabar/reproducir trayectorias completas y E-STOP.
"""

import math, time, threading, sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ══════════════════════════════════════════════════════════════════
# DEFINICIÓN DE JOINTS
# (id, etiqueta, joint_name, min, max, unidad, tolerancia_convergencia)
# ══════════════════════════════════════════════════════════════════
JOINT_DEFS = [
    (1, 'Riel   ', 'moviento_en_x',            0.000,  0.300, 'm  ', 0.005),
    (2, 'Hombro ', 'disco_soportes_rotacion',  -2.094,  2.094, 'rad', 0.050),
    (3, 'Codo   ', 'brazo_motores',            -2.094,  2.094, 'rad', 0.050),
    (4, 'Muñeca ', 'brazo_gripper_azul',       -2.094,  2.094, 'rad', 0.050),
    (5, 'Gripper', 'griper_mecanismo',          0.000,  1.000, 'rad', 0.050),
]

ARM_JOINTS    = [d[2] for d in JOINT_DEFS[:4]]
GRIPPER_JOINT = JOINT_DEFS[4][2]
ALL_JOINTS    = [d[2] for d in JOINT_DEFS]

ARM_TOPIC      = '/arm_controller/joint_trajectory'
GRIPPER_TOPIC  = '/gripper_controller/joint_trajectory'
FEEDBACK_TOPIC = '/esp32/feedback'

MOVE_TIMEOUT = 12.0   # segundos máximos esperando convergencia
SEP          = '─' * 58


# ══════════════════════════════════════════════════════════════════
# NODO ROS 2
# ══════════════════════════════════════════════════════════════════
class ControlNode(Node):
    def __init__(self):
        super().__init__('teach_repeat_node')
        self.sub = self.create_subscription(
            JointState, '/joint_states', self._state_cb, 10)
        self.feedback_sub = self.create_subscription(
            String, FEEDBACK_TOPIC, self._feedback_cb, 10)
        self.arm_pub     = self.create_publisher(JointTrajectory, ARM_TOPIC,     10)
        self.gripper_pub = self.create_publisher(JointTrajectory, GRIPPER_TOPIC, 10)

        self._pos:     dict = {j: 0.0 for j in ALL_JOINTS}
        self._ready:   bool = False
        self._last_fb: str  = ''
        self.waypoints:list = []   # waypoints solo de ARM_JOINTS

    def _state_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self._pos:
                self._pos[name] = pos
        self._ready = True

    def _feedback_cb(self, msg):
        self._last_fb = msg.data

    @property
    def arm_pos(self):
        return [self._pos[j] for j in ARM_JOINTS]

    @property
    def gripper_pos(self):
        return self._pos[GRIPPER_JOINT]

    def esp32_frame(self, override=None):
        """Construye <V1,V2,V3,V4,V5> con posiciones actuales y override opcional."""
        vals = [(override or {}).get(jn, self._pos[jn]) for jn in ALL_JOINTS]
        return '<' + ','.join(f'{v:.4f}' for v in vals) + '>'

    def send_arm(self, positions, t_sec=2.0):
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions     = positions
        pt.velocities    = [0.0] * len(ARM_JOINTS)
        pt.accelerations = [0.0] * len(ARM_JOINTS)
        pt.time_from_start.sec     = int(t_sec)
        pt.time_from_start.nanosec = int((t_sec % 1) * 1e9)
        msg.points.append(pt)
        self.arm_pub.publish(msg)

    def send_gripper(self, position, t_sec=1.5):
        msg = JointTrajectory()
        msg.joint_names = [GRIPPER_JOINT]
        pt = JointTrajectoryPoint()
        pt.positions     = [position]
        pt.velocities    = [0.0]
        pt.time_from_start.sec = int(t_sec)
        msg.points.append(pt)
        self.gripper_pub.publish(msg)

    def emergency_stop(self):
        self.send_arm(self.arm_pos, t_sec=0.05)
        self.send_gripper(self.gripper_pos, t_sec=0.05)

    def play_sequence(self, loop_count=1):
        if not self.waypoints:
            return False
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS
        t = 2.0
        for _ in range(loop_count):
            for wp in self.waypoints:
                pt = JointTrajectoryPoint()
                pt.positions     = wp
                pt.velocities    = [0.0] * len(ARM_JOINTS)
                pt.accelerations = [0.0] * len(ARM_JOINTS)
                pt.time_from_start.sec     = int(t)
                pt.time_from_start.nanosec = int((t % 1) * 1e9)
                msg.points.append(pt)
                t += 2.0
        self.arm_pub.publish(msg)
        return True

    def wait_convergence(self, joint_name, target, tolerance, unit):
        """
        Bloquea el hilo del menú mostrando una barra de progreso hasta que
        el joint llegue a target±tolerance o se agote MOVE_TIMEOUT.
        Retorna (logrado:bool, segundos:float).
        """
        t0 = time.time()
        e0 = abs(self._pos[joint_name] - target)
        if e0 < 1e-6:
            print(f"  [{'█'*20}]  Δ=0.0000 {unit.strip()}  0.0s  ✅ YA EN POSICIÓN")
            return True, 0.0

        while True:
            elapsed = time.time() - t0
            error   = abs(self._pos[joint_name] - target)
            prog    = max(0.0, min(1.0, 1.0 - error / e0))
            bar     = '█' * int(prog * 20) + '░' * (20 - int(prog * 20))

            print(f'\r  [{bar}]  Δ={error:.4f} {unit.strip()}  {elapsed:.1f}s ',
                  end='', flush=True)

            if error < tolerance:
                print(f'\r  [{"█"*20}]  Δ={error:.4f} {unit.strip()}  {elapsed:.1f}s  ✅ LOGRADO')
                return True, elapsed

            if elapsed > MOVE_TIMEOUT:
                print(f'\r  [{bar}]  Δ={error:.4f} {unit.strip()}  ⏱ TIMEOUT ({MOVE_TIMEOUT:.0f}s)')
                return False, elapsed

            time.sleep(0.05)


# ══════════════════════════════════════════════════════════════════
# PARSEADOR DE POSICIÓN
# ══════════════════════════════════════════════════════════════════
def parse_position(raw, lo, hi, unit):
    """
    Acepta:   1.57   → float en la unidad del joint
              90d    → grados, convertido a radianes
    Retorna: (valor, '') si válido  |  (None, mensaje) si error
    """
    raw = raw.strip()
    if not raw:
        return None, 'cancelado'
    in_deg = raw.lower().endswith('d')
    try:
        num = float(raw[:-1] if in_deg else raw)
    except ValueError:
        return None, f"'{raw}' no es un número válido."

    if in_deg:
        if unit.strip() == 'm':
            return None, 'El riel usa metros. Ingresa un valor en m (ej: 0.15).'
        val = math.radians(num)
        print(f'\n  → Convirtiendo: {num}° → {val:.4f} rad')
    else:
        val = num

    if not (lo - 1e-6 <= val <= hi + 1e-6):
        return None, f'Fuera de rango. Límites: [{lo:.4f},  {hi:.4f}]  {unit.strip()}.'
    return val, ''


# ══════════════════════════════════════════════════════════════════
# FLUJO DE MOVIMIENTO INDIVIDUAL
# ══════════════════════════════════════════════════════════════════
def mover_motor(node, jdef):
    jid, label, jname, lo, hi, unit, tol = jdef
    is_gripper = (jname == GRIPPER_JOINT)
    current    = node._pos[jname]
    us         = unit.strip()

    print(f'\n{SEP}')
    print(f'  Motor seleccionado : [{jid}] {label.strip()}')
    print(f'  Joint name         : {jname}')
    print(f'  Rango válido       : [{lo:.4f},  {hi:.4f}]  {us}')
    print(f'  Posición actual    : {current:+.4f}  {us}')
    print(SEP)

    hint = ('en metros (ej: 0.15)' if us == 'm'
            else 'en rad (ej: 1.57)  ó  en grados con "d" (ej: 90d)')
    raw = input(f'  Ingresa la pose deseada {hint}\n  [Enter = cancelar] > ').strip()

    target, err = parse_position(raw, lo, hi, unit)
    if target is None:
        print(f'\n  ⚠  {err}')
        print(SEP)
        return

    # Vista previa de trama ESP32
    frame    = node.esp32_frame({jname: target})
    desglose = '  |  '.join(
        f"{d[1].strip()}={target if d[2]==jname else node._pos[d[2]]:.4f}{d[5].strip()}"
        for d in JOINT_DEFS
    )
    print(f'\n  → Trama que recibirá el ESP32:')
    print(f'    {frame}')
    print(f'    {desglose}')

    # Envío al controlador
    print(f'\n  → Enviando comando al controlador ROS 2 ... ', end='', flush=True)
    if is_gripper:
        node.send_gripper(target)
    else:
        new_arm = list(node.arm_pos)
        new_arm[ARM_JOINTS.index(jname)] = target
        node.send_arm(new_arm, t_sec=2.0)
    print('✓')
    print(f'  → serial_bridge enviará  {frame}  al ESP32 (ciclo 50 Hz).')
    print(f'  → Esperando que el actuador alcance la pose objetivo ...\n')

    # Espera bloqueante con barra de progreso
    logrado, elapsed = node.wait_convergence(jname, target, tol, unit)

    if logrado:
        print(f'\n  ✅ Pose {target:.4f} {us} alcanzada en {elapsed:.1f} s.')
        if node._last_fb:
            print(f'  📩 Última respuesta ESP32: "{node._last_fb}"')
    else:
        print(f'\n  ⚠  El actuador no llegó a la pose en {MOVE_TIMEOUT:.0f} s.')
        print(f'     Verifica: cable USB, firmware ESP32, rango mecánico del motor.')
    print(SEP)


# ══════════════════════════════════════════════════════════════════
# MENÚ PRINCIPAL
# ══════════════════════════════════════════════════════════════════
def mostrar_menu(node):
    estado = '  '.join(
        f'[{d[0]}]{d[1].strip()}:{node._pos[d[2]]:+.3f}{d[5].strip()}'
        for d in JOINT_DEFS
    )
    print(f'\n{"═"*58}')
    print(f'  CONTROL BRAZO CARRO  —  Hiwonder via ESP32')
    print(f'{"═"*58}')
    print(f'  {estado}')
    print(f'{"─"*58}')
    print(f'  CONTROL INDIVIDUAL  (selecciona el número del motor):')
    for d in JOINT_DEFS:
        print(f'    [{d[0]}]  {d[1]}  [{d[3]:.3f} … {d[4]:.3f}] {d[5].strip()}')
    print(f'{"─"*58}')
    print(f'  TRAYECTORIAS COMPLETAS:')
    print(f'    [r]  Grabar posición actual   '
          f'[{len(node.waypoints)} waypoint(s) grabado(s)]')
    print(f'    [p]  Reproducir 1 vez')
    print(f'    [b]  Reproducir en bucle (N ciclos)')
    print(f'    [l]  Listar waypoints          [c]  Borrar todo')
    print(f'{"─"*58}')
    print(f'    [s]  Estado completo de joints')
    print(f'    [e]  ⛔ PARO DE EMERGENCIA')
    print(f'    [q]  Salir')
    print(f'{"═"*58}')


def menu_loop(node):
    print('\n  Esperando /joint_states ', end='', flush=True)
    while not node._ready and rclpy.ok():
        print('.', end='', flush=True)
        time.sleep(0.3)
    print(' ✓\n')

    jdef_map = {str(d[0]): d for d in JOINT_DEFS}

    while rclpy.ok():
        mostrar_menu(node)
        try:
            cmd = input('\n  Comando > ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if cmd in jdef_map:
            mover_motor(node, jdef_map[cmd])

        elif cmd == 's':
            print(f'\n  {"Joint":<32} {"Actual":>9}  Unidad')
            print(f'  {"─"*48}')
            for d in JOINT_DEFS:
                print(f'  {d[2]:<32} {node._pos[d[2]]:>+9.4f}  {d[5].strip()}')
            if node._last_fb:
                print(f'\n  📩 Última respuesta ESP32: "{node._last_fb}"')

        elif cmd == 'r':
            wp = list(node.arm_pos)
            node.waypoints.append(wp)
            vals = '  '.join(f'{v:+.3f}' for v in wp)
            print(f'\n  ✓ Waypoint [{len(node.waypoints)}] grabado: {vals}')

        elif cmd == 'p':
            if node.play_sequence(1):
                print(f'\n  ▶ Ejecutando {len(node.waypoints)} waypoint(s) ...')
            else:
                print('\n  ⚠  No hay waypoints. Graba alguno con [r].')

        elif cmd == 'b':
            try:
                n = int(input('  Ciclos [3]: ').strip() or '3')
            except ValueError:
                n = 3
            if node.play_sequence(n):
                print(f'\n  🔁 {n} ciclos × {len(node.waypoints)} waypoints en marcha ...')
            else:
                print('\n  ⚠  No hay waypoints.')

        elif cmd == 'l':
            if not node.waypoints:
                print('\n  Sin waypoints grabados.')
            else:
                headers = '  '.join(f'{n.split("_")[0]:>8}' for n in ARM_JOINTS)
                print(f'\n  {"#":<4}  {headers}')
                print(f'  {"─"*44}')
                for i, wp in enumerate(node.waypoints, 1):
                    row = '  '.join(f'{v:>+8.3f}' for v in wp)
                    print(f'  {i:<4}  {row}')

        elif cmd == 'c':
            node.waypoints.clear()
            print('\n  ✓ Waypoints borrados.')

        elif cmd == 'e':
            node.emergency_stop()
            print('\n  ⛔ PARO DE EMERGENCIA enviado.')

        elif cmd == 'q':
            print('\n  Cerrando...')
            break
        else:
            print(f"\n  Comando '{cmd}' no reconocido.")


# ══════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    threading.Thread(target=menu_loop, args=(node,), daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

if __name__ == '__main__':
    main()