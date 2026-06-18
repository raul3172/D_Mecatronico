# Universidad Autónoma de México — Facultad de Ingeniería
## Diseño Mecatrónico
### Brazo Robótico de 4 GDL + Gripper + Base Lineal con Motor a Pasos — Operado en ROS 2
Diseñado y ensamblado por: 
-Fuentes Yañez Ivan
-Garduño Cruz Ingrid Paola
-Huarancca Panana Irvin Daymont
-López Cruz Marino
-Ortíz Martínez Raúl

---

## Tabla de Contenidos

- [Descripción del Proyecto](#descripción-del-proyecto)
- [Arquitectura del Sistema](#arquitectura-del-sistema)
- [Estructura de Archivos](#estructura-de-archivos)
- [Descripción de los Códigos](#descripción-de-los-códigos)
  - [arm_description_v2.xacro](#1-arm_description_v2xacro--descripción-cinemática-del-robot)
  - [arm_controllers.yaml](#2-arm_controllersyaml--configuración-de-controladores)
  - [teach_repeat.py](#3-teach_repeatpy--control-por-terminal-enseñar-y-repetir)
  - [teach_pendant_gui.py](#4-teach_pendant_guipy--interfaz-gráfica-de-control-pyqt5)
  - [IntegradoMotoresYBase.ino](#5-integradomoresybaseino--firmware-esp32-control-de-hardware-real)
- [Interconexión entre Módulos](#interconexión-entre-módulos)
- [Requisitos Previos](#requisitos-previos)
- [Instalación y Configuración](#instalación-y-configuración)
- [Instrucciones para Echar a Andar el Robot](#instrucciones-para-echar-a-andar-el-robot)
- [Control del Robot](#control-del-robot)
- [Solución de Problemas Comunes](#solución-de-problemas-comunes)
- [Notas Técnicas](#notas-técnicas)

---

## Descripción del Proyecto

Este proyecto implementa el control completo de un **brazo robótico serial de 3 grados de libertad (GDL)** montado sobre una **base lineal accionada por motor a pasos**, con un **gripper** al final del efector. El sistema es operado mediante **ROS 2** y puede visualizarse y simularse en **RViz** y **Gazebo**.

### Configuración Física

| Componente | Tipo | Articulación ROS 2 |
|---|---|---|
| Base lineal (riel) | Prismática — traslación en eje Y | `Motor_Base` |
| Hombro | Revoluta | `Motor_1` |
| Codo | Revoluta | `Motor_2` |
| Muñeca | Revoluta | `Motor_3` |
| Gripper | (efector final, no articulado en control) | — |

---

## Arquitectura del Sistema

```
┌─────────────────────────────────────────────────────────────────┐
│                     RASPBERRY PI — ROS 2                        │
│                                                                 │
│  ┌──────────────────┐        ┌───────────────────────────────┐ │
│  │ teach_pendant_   │        │       teach_repeat.py         │ │
│  │    gui.py        │        │   (Control por terminal)      │ │
│  │  (GUI PyQt5)     │        │                               │ │
│  └────────┬─────────┘        └─────────────┬─────────────────┘ │
│           │                                │                   │
│           └───────────────┬────────────────┘                   │
│                           │  publica /joint_states             │
│                           ▼                                    │
│            ┌──────────────────────────┐                        │
│            │     /joint_states topic  │                        │
│            │  sensor_msgs/JointState  │                        │
│            └──────┬───────────────────┘                        │
│                   │                                            │
│        ┌──────────┴──────────┐                                 │
│        ▼                     ▼                                 │
│  ┌──────────────┐    ┌───────────────────┐                     │
│  │ robot_state_ │    │  serial_bridge    │                     │
│  │ publisher    │    │  (nodo ROS 2)     │                     │
│  │ (XACRO/TF)  │    │  espera CALIBRADO │                     │
│  └──────┬───────┘    └────────┬──────────┘                     │
│         │                     │  <riel,h,c,m,g> por UART      │
│         ▼                     │  115200 baud                   │
│  ┌─────────────┐              │                                │
│  │    RViz     │              │                                │
│  │ (simulación)│              │                                │
│  └─────────────┘              │                                │
└───────────────────────────────┼────────────────────────────────┘
                                │  USB/UART serial
                                ▼
┌───────────────────────────────────────────────────────────────┐
│               ESP32 — IntegradoMotoresYBase.ino               │
│                                                               │
│  setup() → ejecutarHoming() → xTaskCreate(tareaMotorPasos)   │
│                                                               │
│  loop() (core 1)              tareaMotorPasos (core 0)        │
│  ├─ Parser serial <...>       └─ stepper.run() + WDT          │
│  └─ procesarTrama()                                           │
│       ├─ moverAMilimetros()  → A4988 → Motor a pasos (riel)  │
│       ├─ radToHiwonder()     → Bus serial → Servos Hiwonder   │
│       │                          ID1:Hombro  ID2:Codo         │
│       │                          ID3:Muñeca                   │
│       └─ servoGripper.write()→ PWM → SG90 (gripper)          │
│                                                               │
│  GPIO 26: FC_INICIO ──── homing                              │
│  GPIO 27: FC_FIN    ──── homing                              │
└───────────────────────────────────────────────────────────────┘
```

---

## Estructura de Archivos

```
robot_arm_ws/
├── src/
│   └── arm_description/
│       ├── urdf/
│       │   └── arm_description_v2.xacro       # Descripción cinemática del robot
│       ├── config/
│       │   └── arm_controllers.yaml           # Configuración de controladores ROS 2
│       ├── launch/
│       │   └── display.launch.py              # Launch para RViz
│       ├── scripts/
│       │   ├── teach_repeat.py                # Control por terminal (enseñar y repetir)
│       │   └── teach_pendant_gui.py           # Control por interfaz gráfica PyQt5
│       ├── meshes/                            # Mallas 3D del robot (.STL/.DAE)
│       ├── package.xml
│       └── CMakeLists.txt
├── firmware/
│   └── IntegradoMotoresYBase/
│       └── IntegradoMotoresYBase.ino          # Firmware ESP32 — control de hardware real
└── README.md
```

---

## Descripción de los Códigos

### 1. `arm_description_v2.xacro` — Descripción Cinemática del Robot

**Propósito:** Define la geometría, cinemática, masa e inercias del robot para ROS 2.

**Contenido principal:**

- **Links** (eslabones): declaran la geometría visual, de colisión y propiedades de masa/inercia de cada pieza física. Las inercias están expresadas en coordenadas **relativas al link**, no globales.
- **Joints** (articulaciones): definen cómo se conectan los links entre sí. Los orígenes están en **metros** y los ángulos en **radianes**.

**Articulaciones definidas:**

| Joint | Tipo | Eje | Rango |
|---|---|---|---|
| `Motor_Base` | `prismatic` | Y | 0.0 – 0.5 m |
| `Motor_1` | `revolute` | Z | −π – +π rad |
| `Motor_2` | `revolute` | Z | −π/2 – +π/2 rad |
| `Motor_3` | `revolute` | Z | −π – +π rad |

> **Nota importante:** Los archivos exportados desde CAD frecuentemente usan milímetros en lugar de metros y coordenadas globales en los orígenes de inercia. Esta versión (`v2`) corrige ambos problemas.

---

### 2. `arm_controllers.yaml` — Configuración de Controladores

**Propósito:** Configura el plugin `ros2_control` para exponer los cuatro joints del robot como un `JointTrajectoryController`.

**Estructura:**

```yaml
controller_manager:
  ros__parameters:
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

arm_controller:
  ros__parameters:
    joints:
      - Motor_Base
      - Motor_1
      - Motor_2
      - Motor_3
    command_interfaces: [position]
    state_interfaces: [position, velocity]
```

**Por qué los cuatro joints:** Una configuración incompleta (solo 3 joints) haría que `ros2_control` ignorara `Motor_Base`, dejando la base lineal sin control activo.

---

### 3. `teach_repeat.py` — Control por Terminal (Enseñar y Repetir)

**Propósito:** Nodo ROS 2 en Python que permite al usuario definir posturas del robot ("enseñar") mediante comandos de terminal y luego reproducirlas en secuencia ("repetir").

**Funcionamiento interno:**

```
Usuario escribe postura → Nodo recibe parámetros →
Publica en /joint_states → ros2_control ejecuta movimiento
```

**Características clave:**

- Usa un `dict` (`_pos_map`) para mapear nombre de joint → posición, en lugar de índices fijos. Esto lo hace **robusto ante mensajes `JointState` que lleguen en orden arbitrario**.
- Permite guardar una secuencia de waypoints y reproducirlos en bucle.
- Las posiciones de `Motor_Base` se especifican en **metros** y las de `Motor_1/2/3` en **radianes**.

**Uso básico:**

```bash
ros2 run arm_description teach_repeat.py
# Dentro del nodo:
# > teach   → guarda postura actual
# > repeat  → reproduce todas las posturas guardadas
# > clear   → borra la secuencia
```

---

### 4. `teach_pendant_gui.py` — Interfaz Gráfica de Control (PyQt5)

**Propósito:** Nodo ROS 2 con interfaz gráfica que emula un *teach pendant* industrial: panel de control con sliders para mover cada articulación en tiempo real.

**Funcionamiento interno:**

```
Slider GUI (PyQt5) → callback onChange →
Publica JointState en /joint_states →
robot_state_publisher actualiza TF →
RViz visualiza postura
```

**Características de la GUI:**

| Control | Rango | Unidad |
|---|---|---|
| Slider `Motor_Base` | 0.0 – 0.5 | metros (m) |
| Slider `Motor_1` | −3.14 – +3.14 | radianes (rad) |
| Slider `Motor_2` | −1.57 – +1.57 | radianes (rad) |
| Slider `Motor_3` | −3.14 – +3.14 | radianes (rad) |
| Botón **Teach** | — | Guarda postura actual |
| Botón **Repeat** | — | Reproduce secuencia |
| Botón **Clear** | — | Limpia secuencia |

> Los sliders usan escala interna en enteros y convierten al rango físico al publicar, garantizando precisión y fluidez visual.

---

### 5. `IntegradoMotoresYBase.ino` — Firmware ESP32: Control de Hardware Real

**Propósito:** Firmware que corre directamente en el microcontrolador **ESP32** y actúa como la capa más baja del sistema. Recibe tramas de posición desde la Raspberry Pi (vía `serial_bridge` de ROS 2) y las traduce en movimientos físicos reales de todos los actuadores.

> Este archivo es el **puente entre el mundo ROS 2 (software) y el hardware físico del robot**. Sin él, el sistema solo funciona en simulación.

---

#### Mapa de hardware

| GPIO | Periférico | Función |
|---|---|---|
| 16 (RX2) | Bus serial Hiwonder | Recibir telemetría de servos |
| 17 (TX2) | Bus serial Hiwonder | Enviar comandos a servos |
| 18 | Servo SG90 (gripper) | PWM 50 Hz, 500–2400 µs |
| 25 | Driver A4988 — DIR | Dirección del motor a pasos |
| 33 | Driver A4988 — STEP | Pulsos del motor a pasos |
| 26 | Final de carrera INICIO | Detecta el límite de origen del riel |
| 27 | Final de carrera FIN | Detecta el límite máximo del riel |

---

#### Librerías utilizadas

| Librería | Propósito |
|---|---|
| `LobotSerialServoControl` | Protocolo de bus serial para servos Hiwonder (ID 1, 2, 3) |
| `AccelStepper` | Control suave del motor a pasos con aceleración y desaceleración |
| `ESP32Servo` | Generación de PWM para servo SG90 compatible con ESP32 |
| FreeRTOS (integrado en ESP-IDF) | Multitarea: tarea dedicada al motor en core 0 |

---

#### Flujo de ejecución

```
Encendido del ESP32
       │
       ▼
setup()
  ├─ Inicializa serial, GPIO, servo, stepper y mutex
  └─ ejecutarHoming()          ← bloquea hasta completar calibración
          │
          ├─ Fase 1: avanza hasta FC_FIN  → mide recorrido total
          ├─ Retroceso 3 mm (suelta sensor)
          ├─ Fase 2: regresa hasta FC_INICIO → define posición 0
          ├─ Avance 2 mm (despega del sensor) → redefine cero real
          ├─ Test gripper (0° → 110° → 0°)
          └─ Envía "CALIBRADO" por serial → Raspberry Pi libera ROS 2
               │
               ▼
       xTaskCreatePinnedToCore(tareaMotorPasos, core 0)
               │
               ▼
loop() — core 1 (en paralelo con la tarea)
  └─ Lee serial byte a byte buscando tramas <...>
       └─ procesarTrama() cuando trama válida
```

---

#### Secuencia de Homing — `ejecutarHoming()`

El homing es la **calibración automática de la base lineal** al encender el robot. Es indispensable porque el motor a pasos no tiene encoder: sin homing, el ESP32 no sabe en qué posición está el carro del riel.

```
  INICIO ◄────────────────────────────────────────── FIN
  [FC_INICIO]  2mm ◄── cero real ──► 3mm  [FC_FIN]
     │                                              │
     │← ← ← ← Fase 2: regresa ← ← ← ← ← ← ← ← ←│
     │                                              │
     │─ ─ ─ ─ Fase 1: avanza ─ ─ ─ ─ ─ ─ ─ ─ ─ →│
```

**Pasos detallados:**

1. **Avanza hacia FC_FIN** a velocidad constante (300 pasos/s). Mide cuántos pasos recorre — ese valor es `recorridoMaxPasos`, el límite físico del riel.
2. **Retrocede 3 mm** para soltar el sensor final (evita presión mecánica continua).
3. **Regresa hacia FC_INICIO** a velocidad constante inversa.
4. **Define `currentPosition = 0`** al tocar el inicio.
5. **Avanza 2 mm** para despegar el sensor, luego **redefine ese punto como el nuevo cero**. De esta forma el riel nunca queda presionando un final de carrera.
6. **Test del gripper**: ciclo 0° → 110° → 0° para verificar que el SG90 responde correctamente.
7. **Envía `"CALIBRADO"`** por serial. El `serial_bridge` en la Raspberry Pi espera este mensaje antes de comenzar a enviar tramas de posición, garantizando que ROS 2 y el ESP32 están sincronizados.

> **Decisión de diseño crítica:** La tarea FreeRTOS del motor se crea **después** del homing, no antes. Si se creara antes, la tarea y `ejecutarHoming()` competirían por el stepper simultáneamente (race condition), causando movimientos erráticos o bloqueos del mutex.

---

#### Protocolo de comunicación serial — Formato de trama

La Raspberry Pi envía tramas a **115200 baud** con la siguiente estructura:

```
<riel_m,hombro_rad,codo_rad,muneca_rad,gripper_norm>
```

| Campo | Tipo | Unidad | Ejemplo |
|---|---|---|---|
| `riel_m` | float | metros | `0.235` |
| `hombro_rad` | float | radianes | `1.047` |
| `codo_rad` | float | radianes | `-0.524` |
| `muneca_rad` | float | radianes | `0.000` |
| `gripper_norm` | float | normalizado [0.0–1.0] | `0.750` |

**Ejemplo de trama completa:**
```
<0.235,1.047,-0.524,0.000,0.750>
```

El parser en `loop()` detecta `<` para abrir y `>` para cerrar la trama, valida que tenga exactamente **4 comas** y usa `sscanf()` para parsear los 5 valores flotantes de forma robusta. Tramas malformadas o con ruido se descartan silenciosamente.

**Mecanismo de timeout de trama:** Si transcurren más de 50 ms entre `<` y `>` sin cerrar la trama, el buffer se descarta y se espera la siguiente. Esto protege contra fragmentación del stream serial.

---

#### Flush post-homing — `tramasFlushPendientes`

El `serial_bridge` de ROS 2 opera a ~50 tramas/segundo de forma continua. Durante el homing, ROS 2 sigue enviando la posición objetivo inicial (típicamente `0.46 m` de riel), aunque el robot aún no está calibrado. Si el ESP32 procesara esas tramas residuales inmediatamente después del homing, el motor intentaría ir a `0.46 m` antes de que ROS 2 actualizara su estado.

**Solución:** Se descartan las primeras **150 tramas** post-homing (≈ 3 segundos), suficiente para que ROS 2 termine de procesar la trayectoria de calibración y comience a enviar posiciones válidas.

```cpp
if (tramasFlushPendientes > 0) {
    tramasFlushPendientes--;
    return;   // descarta la trama silenciosamente
}
```

---

#### Conversión de unidades — `radToHiwonder()`

Los servos Hiwonder usan una escala propietaria de **0 a 1000** (donde 500 = centro = 0°). La función convierte radianes a esta escala considerando el límite mecánico de ±120° (±2.0944 rad):

```cpp
int radToHiwonder(float rad) {
    return constrain(500 + (int)(rad * 500.0f / LIM_RAD), 0, 1000);
}
```

| Entrada (rad) | Salida (Hiwonder) | Ángulo físico |
|---|---|---|
| −2.0944 | 0 | −120° (límite izquierdo) |
| 0.0 | 500 | 0° (centro) |
| +2.0944 | 1000 | +120° (límite derecho) |

---

#### Deadband de servos

Para evitar vibraciones y desgaste prematuro de los servos Hiwonder por ruido en la señal de ROS 2, se aplica un **deadband de 4 unidades**: solo se envía un nuevo comando al servo si la posición destino difiere en más de 4 unidades de la última enviada.

```cpp
if (abs(posHombro - lastHombro) > DEADBAND) {
    BusServo.LobotSerialServoMove(ID_HOMBRO, posHombro, MOVE_TIME_MS);
    lastHombro = posHombro;
}
```

---

#### Multitarea FreeRTOS — `tareaMotorPasos()`

El motor a pasos requiere pulsos de STEP muy frecuentes (cada pocos microsegundos a alta velocidad). Si estos pulsos se generaran en `loop()` junto con el parsing serial, las interrupciones del serial retrasarían los pulsos y el motor perdería pasos.

**Solución:** La función `stepper.run()` corre en una **tarea dedicada en el core 0** del ESP32 (core independiente al de `loop()`). El acceso al objeto `stepper` está protegido por un **mutex FreeRTOS** (`stepperMutex`) para evitar que `loop()` y la tarea modifiquen el stepper simultáneamente.

```
Core 0                          Core 1
──────────────────────          ──────────────────────
tareaMotorPasos()               loop()
  └─ stepper.run()                └─ Serial.read()
     (pulsos STEP)                   procesarTrama()
     mutex protege acceso            mutex protege acceso
```

La tarea también alimenta el **Watchdog Timer (WDT)** del core 0 en cada iteración con `esp_task_wdt_reset()`, previniendo reinicios del ESP32 por timeout del watchdog.

---

#### Gripper SG90 — Escala normalizada

El gripper recibe valores normalizados `[0.0, 1.0]` desde ROS 2, que se mapean a `[0°, 110°]` del servo SG90:

```cpp
int grados = constrain((int)round(valores[4] * 110.0f), 0, 180);
servoGripper.write(grados);
```

| Valor ROS 2 | Ángulo SG90 | Estado gripper |
|---|---|---|
| `0.0` | 0° | Cerrado |
| `0.5` | 55° | Medio abierto |
| `1.0` | 110° | Abierto total |

---

#### Instrucciones para cargar el firmware

**Requisitos:**
- Arduino IDE 2.x con soporte para ESP32 (board: `esp32 by Espressif Systems`)
- Librerías instaladas en Arduino IDE:
  - `AccelStepper` (Mike McCauley)
  - `ESP32Servo` (Kevin Harrington)
  - `LobotSerialServoControl` (Lobot / Hiwonder) — instalar manualmente

**Pasos:**

```
1. Abrir Arduino IDE
2. Seleccionar placa: Herramientas → Placa → ESP32 Dev Module
3. Seleccionar puerto: Herramientas → Puerto → /dev/ttyUSB0 (o el que corresponda)
4. Abrir IntegradoMotoresYBase.ino
5. Verificar (Ctrl+R) — debe compilar sin errores
6. Subir (Ctrl+U) — mantener presionado BOOT en el ESP32 si no sube automáticamente
7. Abrir Monitor Serial a 115200 baud
8. Verificar que aparezca "ESP32 ONLINE - Iniciando homing automatico..."
9. Observar secuencia de homing; al final debe aparecer "CALIBRADO"
```

> **Importante:** El `serial_bridge` de ROS 2 (en la Raspberry Pi) **espera el mensaje `"CALIBRADO"`** antes de comenzar a publicar tramas. Si el homing falla o el mensaje no llega, el sistema ROS 2 permanece en espera.

---

## Interconexión entre Módulos

El siguiente diagrama muestra el flujo de datos completo entre todos los archivos del proyecto, desde la interfaz de usuario hasta los actuadores físicos:

```
teach_pendant_gui.py  ──┐
teach_repeat.py         ├──► /joint_states topic
                        │         │
                        │         ├──► robot_state_publisher ──► /tf ──► RViz
                        │         │         (arm_description_v2.xacro)
                        │         │
                        │         └──► serial_bridge (nodo ROS 2)
                        │                   │
                        │         arm_controllers.yaml
                        │         (configura JointTrajectoryController)
                        │
                        └──────────────────────────────────────────────────────┐
                                                                               │
                                                               UART 115200 baud│
                                                                               ▼
                                                         IntegradoMotoresYBase.ino (ESP32)
                                                               │
                                      ┌────────────────────────┼──────────────────────────┐
                                      ▼                        ▼                          ▼
                               A4988 + Stepper          Bus Hiwonder               PWM SG90
                               (riel lineal)      (hombro, codo, muñeca)          (gripper)
```

**Flujo de comunicación paso a paso:**

1. `arm_description_v2.xacro` es parseado al arrancar `robot_state_publisher`, definiendo la geometría y los frames TF del robot.
2. El ESP32 ejecuta el homing automático al encender y envía `"CALIBRADO"` por serial cuando termina.
3. El `serial_bridge` de ROS 2 (en la Raspberry Pi) espera `"CALIBRADO"` antes de empezar a enviar tramas de posición.
4. `teach_repeat.py` o `teach_pendant_gui.py` publican mensajes `sensor_msgs/JointState` en `/joint_states`.
5. `robot_state_publisher` consume `/joint_states` y publica las transformadas TF de cada eslabón.
6. **RViz** suscribe a `/tf` y `/robot_description` para renderizar el robot en 3D (simulación visual).
7. El `serial_bridge` también suscribe a `/joint_states` y convierte cada mensaje en una trama `<riel_m,h_rad,c_rad,m_rad,gripper_norm>` que envía al ESP32 por UART.
8. El **ESP32** recibe la trama, la parsea, convierte unidades y envía comandos directos a cada actuador físico.

---

## Requisitos Previos

### Sistema Operativo
- **Raspberry Pi:** Ubuntu 22.04 LTS (recomendado para ROS 2 Humble)
- **PC de desarrollo / Arduino IDE:** Windows, macOS o Linux

### Software — Capa ROS 2 (Raspberry Pi)
- **ROS 2 Humble Hawksbill** (o superior)
- **Python 3.10+**
- **PyQt5** (para la GUI)
- **Gazebo Fortress** o **Ignition** (para simulación)
- **colcon** (herramienta de compilación de ROS 2)

### Software — Firmware ESP32
- **Arduino IDE 2.x**
- Board support: `esp32 by Espressif Systems` (vía Boards Manager)
- Librerías Arduino:
  - `AccelStepper` (Mike McCauley) — Library Manager
  - `ESP32Servo` (Kevin Harrington) — Library Manager
  - `LobotSerialServoControl` (Hiwonder) — instalación manual desde `.zip`

### Instalación de dependencias ROS 2

```bash
sudo apt update && sudo apt install -y \
    python3-colcon-common-extensions \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-rviz2 \
    python3-pyqt5
```

### Permisos de puerto serial (para comunicación con ESP32)

```bash
sudo usermod -a -G dialout $USER
# Cerrar sesión y volver a entrar para que aplique
```

---

## Instalación y Configuración

### 1. Crear el workspace

```bash
mkdir -p ~/robot_arm_ws/src
cd ~/robot_arm_ws/src
```

### 2. Clonar o copiar el paquete

```bash
# Si tienes el repositorio en GitHub:
git clone https://github.com/<tu_usuario>/arm_description.git

# O copia manualmente la carpeta arm_description/ dentro de src/
```

### 3. Instalar dependencias de ROS 2

```bash
cd ~/robot_arm_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Compilar el workspace

```bash
cd ~/robot_arm_ws
colcon build --symlink-install
```

### 5. Cargar el entorno

```bash
source ~/robot_arm_ws/install/setup.bash

# Agrega esta línea a tu ~/.bashrc para no repetirla cada sesión:
echo "source ~/robot_arm_ws/install/setup.bash" >> ~/.bashrc
```

---

## Instrucciones para Echar a Andar el Robot

Abre **cuatro terminales** separadas. En cada una ejecuta `source ~/robot_arm_ws/install/setup.bash` antes de cualquier comando.

---

### Terminal 1 — Publicar la Descripción del Robot

```bash
source ~/robot_arm_ws/install/setup.bash

ros2 launch arm_description display.launch.py
```

Esto inicia:
- `robot_state_publisher` (carga el XACRO y publica `/robot_description`)
- `RViz` (abre la visualización 3D del brazo)

> Verifica en RViz que el robot aparezca correctamente en el panel 3D. Si ves errores de TF, espera unos segundos a que los demás nodos levanten.

---

### Terminal 2 — Iniciar el Controlador de Joints

```bash
source ~/robot_arm_ws/install/setup.bash

ros2 run controller_manager spawner arm_controller
ros2 run controller_manager spawner joint_state_broadcaster
```

O si usas el launch completo con Gazebo:

```bash
ros2 launch arm_description gazebo.launch.py
```

---

### Terminal 3 — Opción A: Control por Terminal

```bash
source ~/robot_arm_ws/install/setup.bash

ros2 run arm_description teach_repeat.py
```

Comandos disponibles dentro del nodo:

| Comando | Acción |
|---|---|
| `teach` | Guarda la postura actual en la secuencia |
| `repeat` | Reproduce todas las posturas guardadas |
| `clear` | Borra la secuencia de posturas |
| `set <joint> <valor>` | Mueve un joint a una posición específica |

Ejemplo de sesión:

```
> set Motor_Base 0.2
> set Motor_1 1.57
> set Motor_2 -0.5
> teach
> set Motor_1 0.0
> set Motor_2 0.0
> teach
> repeat
```

---

### Terminal 3 — Opción B: Control por Interfaz Gráfica (Teach Pendant)

```bash
source ~/robot_arm_ws/install/setup.bash

ros2 run arm_description teach_pendant_gui.py
```

Se abrirá la ventana de la GUI con:
- **4 sliders** para mover cada articulación en tiempo real
- **Botones Teach / Repeat / Clear** para programar movimientos
- Indicadores numéricos del valor actual de cada joint

> Mueve los sliders y observa el robot actualizarse en RViz simultáneamente.

---

### Terminal 4 — Monitoreo (Opcional)

```bash
# Ver todos los topics activos
ros2 topic list

# Ver los estados de los joints en tiempo real
ros2 topic echo /joint_states

# Ver el árbol de transformadas TF
ros2 run tf2_tools view_frames
```

---

## Solución de Problemas Comunes

### El robot no aparece en RViz
- Verifica que `robot_state_publisher` esté corriendo: `ros2 node list | grep robot_state`
- Asegúrate de que el XACRO no tenga errores: `xacro arm_description_v2.xacro`
- En RViz, confirma que el **Fixed Frame** esté configurado como `base_link` o `world`

### Error: "Could not load description" al parsear el XACRO
- El error más común es tener valores en **milímetros** en lugar de **metros**. Revisa que todos los `<origin xyz="..."/>` usen valores menores a 2.0 para un robot de tamaño normal de laboratorio.
- Las inercias deben estar en coordenadas relativas al link, no globales.

### Los joints no responden al publicar en `/joint_states`
- Verifica que `arm_controllers.yaml` liste los **cuatro joints**: `Motor_Base`, `Motor_1`, `Motor_2`, `Motor_3`.
- Confirma que el controlador esté activo: `ros2 control list_controllers`

### La GUI se abre pero el robot no se mueve
- Verifica que el nodo `robot_state_publisher` esté corriendo en otra terminal.
- Confirma que `/joint_states` tenga publicaciones: `ros2 topic hz /joint_states`

### Error de importación de PyQt5
```bash
pip3 install PyQt5
# o bien:
sudo apt install python3-pyqt5
```

### El ESP32 no aparece como puerto serial
```bash
# Verificar que el sistema detecta el dispositivo
lsusb | grep -i "cp210\|ch340\|ftdi"
ls /dev/ttyUSB*

# Si no aparece, instalar el driver CP2102 (chip USB-UART del ESP32)
sudo apt install linux-modules-extra-$(uname -r)
```

### El homing no termina / el motor no se mueve
- Verificar conexión física de DIR (GPIO 25) y STEP (GPIO 33) al A4988.
- Confirmar que el A4988 tiene tensión de referencia de motor (VMOT) y lógica (VDD) correctas.
- Abrir Monitor Serial a 115200 baud y verificar que aparezca `"HOMING: Iniciando..."`.
- Si el motor zumba pero no gira, la corriente del A4988 puede estar demasiado baja — ajustar el potenciómetro del driver.

### El ESP32 nunca envía "CALIBRADO"
- Verificar que los finales de carrera estén conectados a GPIO 26 (FC_INICIO) y GPIO 27 (FC_FIN) con `INPUT_PULLUP`.
- Probar los finales de carrera con un multímetro: deben dar continuidad al presionarse.
- Si el riel llega al final de carrera pero el ESP32 no lo detecta, verificar que el sensor sea normalmente abierto (NA) — el código espera que el pin lea `HIGH` cuando libre y `LOW` al activarse.

### Los servos Hiwonder no responden
- Verificar que el bus serial use GPIO 16 (RX2) y GPIO 17 (TX2) a 115200 baud.
- Confirmar que cada servo tenga su ID configurado correctamente (ID1: hombro, ID2: codo, ID3: muñeca) usando el software de configuración Hiwonder.
- El bus serial Hiwonder es half-duplex: TX y RX del ESP32 se conectan al mismo cable de datos del servo a través de un resistor o lógica de bus.

### El gripper no se mueve o tiembla
- Verificar que el SG90 esté conectado a GPIO 18.
- El servo necesita alimentación de 5V externa — no directamente del ESP32 (que da 3.3V en sus pines GPIO y capacidad de corriente limitada en 5V).
- Si el servo tiembla en posición fija, reducir el ruido en la señal PWM asegurando que la alimentación del servo tenga un capacitor de desacople (100 µF) cerca del conector.

---

## Notas Técnicas

### Conversión de unidades CAD → ROS 2
Los archivos XACRO exportados directamente desde software CAD (SolidWorks, Fusion 360, etc.) frecuentemente tienen:
- Coordenadas en **milímetros** → deben convertirse a **metros** (dividir entre 1000)
- Orígenes de inercia en **coordenadas globales** → deben expresarse en **coordenadas relativas al link**
- Rotaciones de malla visual incorrectas → corregir solo en `<visual>`, **nunca** en `<joint>` para no afectar la cinemática

### Robustez en lectura de JointState
Los mensajes `sensor_msgs/JointState` no garantizan un orden fijo en el arreglo de nombres y posiciones. Por esta razón, los scripts de control usan un **diccionario** (`_pos_map: {nombre_joint: posición}`) en lugar de acceso por índice, garantizando que cada joint reciba el valor correcto independientemente del orden de llegada.

### Escalado del slider de Motor_Base
La articulación prismática `Motor_Base` opera en metros (rango 0.0–0.5 m), mientras que las articulaciones revoluta operan en radianes (±π). La GUI escala internamente los sliders a sus respectivas unidades físicas para garantizar precisión y una experiencia de control natural.

---

## Licencia

Proyecto académico — Universidad Autónoma de México, Facultad de Ingeniería.  
Uso educativo y de investigación.

---

*Desarrollado con ROS 2 Humble · Python 3 · PyQt5 · XACRO · RViz · Gazebo · ESP32 · Arduino IDE · AccelStepper · FreeRTOS*
