/*
 * IntegradoMotoresYBase.ino  
 *
 * Correcciones:
 *   1. La tarea FreeRTOS ahora se crea DESPUÉS del homing (elimina race condition).
 *   2. Forward declarations añadidas (necesarias en .cpp; buena práctica en .ino).
 *   3. distanceToGo() leído dentro del mutex.
 *
 * Hardware:
 *   GPIO 16/17 : Servos Hiwonder (serial bus)
 *   GPIO 18    : Gripper SG90 (PWM)
 *   GPIO 25/33 : Motor a pasos A4988 (DIR/STEP)
 *   GPIO 26    : Final de carrera INICIO
 *   GPIO 27    : Final de carrera FIN
 */

#include "LobotSerialServoControl.h"
#include <AccelStepper.h>
#include <ESP32Servo.h>

// ── Forward declarations (bug 2) ──────────────────────────────────
void ejecutarHoming();
void procesarTrama(String payload);
void moverAMilimetros(float milimetros);
int  radToHiwonder(float rad);

// ── Servos Hiwonder ───────────────────────────────────────────────
#define RXD2 16
#define TXD2 17

HardwareSerial SerialPort(2);
LobotSerialServoControl BusServo(SerialPort);

#define ID_HOMBRO 1
#define ID_CODO   2
#define ID_MUNECA 3

const int   MOVE_TIME_MS = 20;
const int   DEADBAND     = 4;
const float LIM_RAD      = 2.0944f;   // ±120°

int lastHombro = -9999;
int lastCodo   = -9999;
int lastMuneca = -9999;

// ── Gripper SG90 ──────────────────────────────────────────────────
#define PIN_GRIPPER 18

Servo servoGripper;
int lastGripperGrados = -999;

// ── Finales de carrera ────────────────────────────────────────────
#define FC_INICIO 26
#define FC_FIN    27

// ── Motor a pasos ─────────────────────────────────────────────────
#define STEP_PIN 33
#define DIR_PIN  25

const float PASOS_POR_MM = 25.0f;

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
SemaphoreHandle_t stepperMutex;

long recorridoMaxPasos = 0;

// ── Buffer serial ─────────────────────────────────────────────────
String buffer = "";
bool   leyendo = false;

// Contador de tramas a descartar tras el homing.
// El serial_bridge manda 50 tramas/seg; descartamos las primeras 3 segundos
// (150 tramas) para que ROS termine de procesar la trayectoria de calibración
// y los valores residuales de 0.46m no lleguen al motor.
int tramasFlushPendientes = 150;

// ════════════════════════════════════════════════════════════════════
// TAREA FREERTOS — core 0
// Se crea DESPUÉS del homing → nunca compite con ejecutarHoming().
// ════════════════════════════════════════════════════════════════════
void tareaMotorPasos(void* pvParameters) {
  // ── Registrar la tarea en el WDT del core 0 ──────────────────────
  esp_task_wdt_add(NULL);

  for (;;) {
    bool idle = false;
    if (xSemaphoreTake(stepperMutex, portMAX_DELAY) == pdTRUE) {
      stepper.run();
      idle = (stepper.distanceToGo() == 0);
      xSemaphoreGive(stepperMutex);
    }

    // ── Alimenta el WDT en cada ciclo, no solo cuando idle ──────────
    esp_task_wdt_reset();

    if (idle) {
      vTaskDelay(1);  // cede CPU al scheduler cuando no hay movimiento
    }
    // Si hay movimiento activo, no cedemos CPU para máxima resolución de pasos
  }
}

// ════════════════════════════════════════════════════════════════════
// HOMING — bloquea setup(); la tarea FreeRTOS aún NO existe aquí.
// ════════════════════════════════════════════════════════════════════
void ejecutarHoming() {
  Serial.println("HOMING: Iniciando secuencia de calibración...");

  // Fase 1: avanzar hasta FC_FIN
  Serial.println("HOMING: Buscando tope final (FC_FIN)...");
  stepper.setSpeed(300);
  while (digitalRead(FC_FIN) == HIGH) {
    stepper.runSpeed();
    while (Serial.available() > 0) Serial.read();   // descartar tramas de ROS
  }

  stepper.stop();
  recorridoMaxPasos = stepper.currentPosition();
  Serial.print("HOMING: FC_FIN. Recorrido real: ");
  Serial.print(recorridoMaxPasos / PASOS_POR_MM, 1);
  Serial.println(" mm");

  // Retroceso 3 mm para soltar el sensor
  stepper.setMaxSpeed(600); stepper.setAcceleration(300);
  stepper.moveTo(recorridoMaxPasos - round(3.0f * PASOS_POR_MM));
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    while (Serial.available() > 0) Serial.read();
  }
  delay(300);

  // Fase 2: regresar hasta FC_INICIO
  Serial.println("HOMING: Regresando al origen (FC_INICIO)...");
  stepper.setSpeed(-300);
  while (digitalRead(FC_INICIO) == HIGH) {
    stepper.runSpeed();
    while (Serial.available() > 0) Serial.read();
  }

  stepper.stop();
  stepper.setCurrentPosition(0);
  Serial.println("HOMING: FC_INICIO. Posicion 0 establecida.");

  // Avance 2 mm para soltar el sensor y redefinir el cero despegado
  stepper.setMaxSpeed(600); stepper.setAcceleration(300);
  stepper.moveTo(round(2.0f * PASOS_POR_MM));
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    while (Serial.available() > 0) Serial.read();
  }
  stepper.setCurrentPosition(0);

  // ── Test gripper post-homing ──────────────────────────────────────
  // Manda el servo a 0° para verificar que el SG90 responde
  servoGripper.write(0);
  delay(1000);
  // Luego a 110° (abierto total)
  servoGripper.write(110);
  delay(1000);
  // Regresa a 0° y se queda ahí
  servoGripper.write(0);
  delay(500);

  // Handshake con la Raspberry Pi
  Serial.println("CALIBRADO");
  //Serial.print("HOMING: Completo. Recorrido util: ");
  //Serial.print(recorridoMaxPasos / PASOS_POR_MM, 1);
  //Serial.println(" mm. Sistema listo.");
}

// ════════════════════════════════════════════════════════════════════
// SETUP
// ════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200, SERIAL_8N1);
  SerialPort.begin(115200, SERIAL_8N1, RXD2, TXD2);

  pinMode(FC_INICIO, INPUT_PULLUP);
  pinMode(FC_FIN,    INPUT_PULLUP);

  servoGripper.setPeriodHertz(50);
  servoGripper.attach(PIN_GRIPPER, 500, 2400);

  stepper.setMaxSpeed(600);
  stepper.setAcceleration(300);

  stepperMutex = xSemaphoreCreateMutex();

  delay(1000);
  Serial.println("ESP32 ONLINE - Iniciando homing automatico...");

  // ── Homing primero, tarea después (bug 1 corregido) ──────────────
  ejecutarHoming();

  // La tarea solo se crea cuando el homing ya terminó.
  // A partir de aquí, TODO acceso al stepper debe pasar por el mutex.
  xTaskCreatePinnedToCore(tareaMotorPasos, "MotorPasos",
                          2048, NULL, 2, NULL, 0);
}

// ════════════════════════════════════════════════════════════════════
// LOOP — core 1
// ════════════════════════════════════════════════════════════════════

// Añade estas variables globales junto a las otras:
// String buffer = "";
// bool   leyendo = false;
unsigned long tiempoInicioTrama = 0;  // ← AÑADIR GLOBAL
const unsigned long TRAMA_TIMEOUT_MS = 50; // ← AÑADIR GLOBAL

void loop() {
  // Timeout: si llevamos >50ms construyendo una trama sin cerrarla,
  // algo se corrompió — descartamos y esperamos la siguiente.
  if (leyendo && (millis() - tiempoInicioTrama > TRAMA_TIMEOUT_MS)) {
    buffer  = "";
    leyendo = false;
    // No imprimir error aquí para no saturar el serial con el bridge
  }

  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '<') {
      // Siempre reinicia al detectar '<', incluso si ya estábamos leyendo
      // (significa que la trama anterior llegó incompleta)
      buffer            = "";
      leyendo           = true;
      tiempoInicioTrama = millis();  // ← marca el inicio

    } else if (c == '>' && leyendo) {
      leyendo = false;

      // Verificación rápida: una trama válida tiene exactamente 4 comas
      int nComas = 0;
      for (int i = 0; i < (int)buffer.length(); i++) {
        if (buffer[i] == ',') nComas++;
      }

      if (nComas == 4) {
        procesarTrama(buffer);
      }
      // Si no tiene 4 comas, la trama llegó fragmentada — silenciosamente descartamos
      buffer = "";

    } else if (leyendo) {
      buffer += c;
      if (buffer.length() > 80) {  // overflow guard
        buffer  = "";
        leyendo = false;
      }
    }
    // Caracteres fuera de trama (\n, \r, ruido) se ignoran implícitamente
  }
}

// ════════════════════════════════════════════════════════════════════
// PROCESAR TRAMA <riel_m, hombro_rad, codo_rad, muneca_rad, gripper_norm>
// ════════════════════════════════════════════════════════════════════
void procesarTrama(String payload) {
  float valores[5];
  int   index = 0;

  // ── FIX: usar sscanf en lugar del parser manual con indexOf/substring.
  // El parser original podía dejar index < 5 si la trama traía espacios,
  // \r, o si el último campo no terminaba en coma — causando ERR:FALTAN_VALORES.
  int leidos = sscanf(payload.c_str(), "%f,%f,%f,%f,%f",
                      &valores[0], &valores[1], &valores[2],
                      &valores[3], &valores[4]);

  if (leidos != 5) {
    Serial.print("ERR:FALTAN_VALORES:"); Serial.println(leidos);
    return;
  }

  // Motor a pasos — acceso protegido por mutex
  float rielMM     = valores[0] * 1000.0f;

  // Descartar tramas residuales post-homing mientras ROS se asienta
  if (tramasFlushPendientes > 0) {
    tramasFlushPendientes--;
    if (tramasFlushPendientes == 0) {
      Serial.println("INFO: Flush post-homing completo.");
    }
    return;
  }

  long  pasosNuevo = round(rielMM * PASOS_POR_MM);
  if (xSemaphoreTake(stepperMutex, portMAX_DELAY) == pdTRUE) {
    if (pasosNuevo != stepper.targetPosition()) {
      moverAMilimetros(rielMM);
    }
    xSemaphoreGive(stepperMutex);
  }

  // Servos Hiwonder
  int posHombro = radToHiwonder(valores[1]);
  int posCodo   = radToHiwonder(valores[2]);
  int posMuneca = radToHiwonder(valores[3]);

  if (abs(posHombro - lastHombro) > DEADBAND) {
    BusServo.LobotSerialServoMove(ID_HOMBRO, posHombro, MOVE_TIME_MS);
    lastHombro = posHombro;
  }
  if (abs(posCodo - lastCodo) > DEADBAND) {
    BusServo.LobotSerialServoMove(ID_CODO, posCodo, MOVE_TIME_MS);
    lastCodo = posCodo;
  }
  if (abs(posMuneca - lastMuneca) > DEADBAND) {
    BusServo.LobotSerialServoMove(ID_MUNECA, posMuneca, MOVE_TIME_MS);
    lastMuneca = posMuneca;
  }

  // Gripper SG90
  // valores[4] viene en escala [0.0, 1.0] desde la GUI. 
  // Multiplicamos por 110 para que 1 = 110 grados y 0 = 0 grados.
  int grados = constrain((int)round(valores[4] * 110.0f), 0, 180);
  
  if (abs(grados - lastGripperGrados) >= 1) {
    servoGripper.write(grados);
    lastGripperGrados = grados;
  }
}

// ════════════════════════════════════════════════════════════════════
// UTILIDADES
// ════════════════════════════════════════════════════════════════════
int radToHiwonder(float rad) {
  return constrain(500 + (int)(rad * 500.0f / LIM_RAD), 0, 1000);
}

// Llamar solo con el mutex ya tomado
void moverAMilimetros(float milimetros) {
  long pasos = constrain((long)round(milimetros * PASOS_POR_MM), 0L, recorridoMaxPasos);
  stepper.moveTo(pasos);
}
