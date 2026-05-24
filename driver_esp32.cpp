#include <SCServo.h>

// Objeto para controlar la familia de servos STS de Feetech
SMS_STS sts_servos;

// IDs de tus motores (configúralos previamente con la herramienta de Feetech)
const byte MOTOR_1_ID = 1;
const byte MOTOR_2_ID = 2;
const byte MOTOR_3_ID = 3;

// Variables para almacenar los ángulos que llegan desde ROS 2
float angulo_1 = 0.0, angulo_2 = 0.0, angulo_3 = 0.0;

void setup() {
  // 1. Iniciar puerto USB (Raspberry Pi -> ESP32)
  Serial.begin(115200);
  
  // 2. Iniciar puerto Hardware Serial 1 (ESP32 -> Servos STS3215)
  // Velocidad por defecto de Feetech: 1000000 baudios. Pines RX=16, TX=17
  Serial1.begin(1000000, SERIAL_8N1, 16, 17);
  sts_servos.pSerial = &Serial1;

  // Pequeña pausa para estabilizar
  delay(1000);
  Serial.println("ESP32 Listo. Esperando trayectorias de ROS 2...");
}

void loop() {
  // Esperamos un formato de texto empaquetado así: <angulo1,angulo2,angulo3>
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    
    // Verificamos que el paquete tenga el formato correcto
    if (data.startsWith("<") && data.endsWith(">")) {
      // Quitamos los símbolos < y >
      data = data.substring(1, data.length() - 1);
      
      // Separamos por comas
      int primera_coma = data.indexOf(',');
      int segunda_coma = data.indexOf(',', primera_coma + 1);
      
      if (primera_coma > 0 && segunda_coma > 0) {
        // Extraemos los radianes
        angulo_1 = data.substring(0, primera_coma).toFloat();
        angulo_2 = data.substring(primera_coma + 1, segunda_coma).toFloat();
        angulo_3 = data.substring(segunda_coma + 1).toFloat();
        
        // Mover los motores
        moverMotores(angulo_1, angulo_2, angulo_3);
      }
    }
  }
}

// Función para traducir Radianes a Pasos Feetech (0 a 4095)
void moverMotores(float rad1, float rad2, float rad3) {
  // El STS3215 tiene 4096 pasos para 360 grados (2*PI radianes)
  // Posición central (0 rad) = 2048
  int paso_1 = 2048 + (rad1 * 4096.0 / (2.0 * PI));
  int paso_2 = 2048 + (rad2 * 4096.0 / (2.0 * PI));
  int paso_3 = 2048 + (rad3 * 4096.0 / (2.0 * PI));

  // Limitamos por seguridad para no dañar la estructura (ej. colisiones)
  paso_1 = constrain(paso_1, 0, 4095);
  paso_2 = constrain(paso_2, 0, 4095);
  paso_3 = constrain(paso_3, 0, 4095);

  // Enviamos la instrucción sincronizada a los 3 motores (Velocidad 0, Aceleración 0)
  // Al enviar 0, dejamos que el ESP32 ejecute el "paso a paso" dictado por la Raspberry
  // que ya trae el polinomio de 5to orden incrustado.
  sts_servos.SyncWritePosEx(MOTOR_1_ID, 4095, 0, 0); // (Reemplazar con SyncWrite real)
  
  // Para los Feetech, el comando de escritura síncrona real se arma así:
  byte ID[3] = {MOTOR_1_ID, MOTOR_2_ID, MOTOR_3_ID};
  s16 Position[3] = {(s16)paso_1, (s16)paso_2, (s16)paso_3};
  u16 Speed[3] = {0, 0, 0}; // 0 = máxima velocidad para alcanzar el setpoint inmediato
  byte ACC[3] = {0, 0, 0};  // La rampa ya la calculó ROS 2
  
  sts_servos.SyncWritePosEx(ID, 3, Position, Speed, ACC);
}