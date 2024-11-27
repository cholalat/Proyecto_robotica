#include "DualVNH5019MotorShield.h"

// Inicialización de MotorShield
DualVNH5019MotorShield md;

// Definición de PINs de los motores y encoders
#define encoder0PinA  19
#define encoder0PinB  18
#define encoder1PinA  20
#define encoder1PinB  21

// Variables de Tiempo
unsigned long time_ant = 0;
const int Period = 10000;   // 10 ms = 100Hz
const float dt = Period * 0.000001f; // Tiempo en segundos
float motorout1 = 0.0;
float motorout2 = 0.0;

// Variables de los Encoders y posición
volatile long encoder0Pos = 0;
volatile long encoder1Pos = 0;
long newposition0;
long oldposition0 = 0;
long newposition1;
long oldposition1 = 0;
unsigned long newtime;
float vel0;
float vel1;

// Variables PID Motor 0 (izquierdo)
float setPoint0 = -150.0;  // RPM inicial (negativo para dirección)
float Kp0 = 3.0;           // Ganancia proporcional
float Ki0 = 0.5;           // Ganancia integral
float Kd0 = 0.02;          // Ganancia derivativa
float integral0 = 0.0;
float previousError0 = 0.0;
const float imax_motor0 = 100.0; // Límite para la integral del motor 0

// Variables PID Motor 1 (derecho)
float setPoint1 = 150.0;   // RPM inicial (positivo para dirección)
float Kp1 = 3.0;           // Ganancia proporcional
float Ki1 = 0.5;           // Ganancia integral
float Kd1 = 0.02;          // Ganancia derivativa
float integral1 = 0.0;
float previousError1 = 0.0;
const float imax_motor1 = 100.0; // Límite para la integral del motor 1

// Variables de Comunicación Bluetooth
const char msgEnd = ';';    // Marca el fin del mensaje
String instruccion;         // Almacena el mensaje recibido
bool newMsg = false;        // Flag para indicar que hay un nuevo mensaje

// Definición del puerto serial para Bluetooth (Serial3)
#define BT_SERIAL Serial3

// Funciones de Interrupción para los Encoders
void doEncoder0A() {
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }
}

void doEncoder0B() {
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos--;
  } else {
    encoder0Pos++;
  }
}

void doEncoder1A() {
  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) {
    encoder1Pos++;
  } else {
    encoder1Pos--;
  }
}

void doEncoder1B() {
  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) {
    encoder1Pos--;
  } else {
    encoder1Pos++;
  }
}

// Función para Leer el Buffer de Bluetooth
String readBuff() {
  String buffArray = "";

  while (BT_SERIAL.available() > 0) { // Mientras haya datos disponibles
    char buff = BT_SERIAL.read();      // Leer un byte
    if (buff == msgEnd) {              // Si encuentra el delimitador
      newMsg = true;                   // Indicar que hay un nuevo mensaje completo
      break;                           // Salir del loop
    } else {
      buffArray += buff;               // Agregar el carácter al buffer
    }
    delay(10); // Pequeña demora para evitar sobrecarga
  }

  return buffArray;  // Retornar el mensaje completo
}

// Funciones de Control PID para Motor 0
float controlPID0(float currentRPM0) {
  float error0 = setPoint0 - currentRPM0;           // Error entre referencia y velocidad actual
  float proportional0 = Kp0 * error0;               // Término proporcional
  integral0 += error0 * dt;                          // Acumular el error para el término integral
  integral0 = constrain(integral0, -imax_motor0, imax_motor0);  // Limitar la integral
  float integralTerm0 = Ki0 * integral0;             // Término integral
  float derivative0 = (error0 - previousError0) / dt; // Derivada del error
  float derivativeTerm0 = Kd0 * derivative0;        // Término derivativo
  float output0 = proportional0 + integralTerm0 + derivativeTerm0; // Salida PID
  output0 = constrain(output0, -400, 400);           // Limitar la salida
  previousError0 = error0;                           // Guardar el error actual para la próxima iteración
  return output0;                                    // Retornar la salida PID
}

// Funciones de Control PID para Motor 1
float controlPID1(float currentRPM1) {
  float error1 = setPoint1 - currentRPM1;           // Error entre referencia y velocidad actual
  float proportional1 = Kp1 * error1;               // Término proporcional
  integral1 += error1 * dt;                          // Acumular el error para el término integral
  integral1 = constrain(integral1, -imax_motor1, imax_motor1);  // Limitar la integral
  float integralTerm1 = Ki1 * integral1;             // Término integral
  float derivative1 = (error1 - previousError1) / dt; // Derivada del error
  float derivativeTerm1 = Kd1 * derivative1;        // Término derivativo
  float output1 = proportional1 + integralTerm1 + derivativeTerm1; // Salida PID
  output1 = constrain(output1, -400, 400);           // Limitar la salida
  previousError1 = error1;                           // Guardar el error actual para la próxima iteración
  return output1;                                    // Retornar la salida PID
}

void setup() {
  // Iniciar Serial para depuración (Monitor Serial)
  Serial.begin(115200);
  Serial.println("Iniciando sistema...");

  // Configurar MotorShield
  md.init();

  // Configurar Encoders como entradas con resistencias pull-up internas
  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA, HIGH);       // Activar pull-up
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinB, HIGH);       // Activar pull-up
  pinMode(encoder1PinA, INPUT);
  digitalWrite(encoder1PinA, HIGH);       // Activar pull-up
  pinMode(encoder1PinB, INPUT);
  digitalWrite(encoder1PinB, HIGH);       // Activar pull-up

  // Adjuntar interrupciones para los encoders
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoder0A, CHANGE);  // Encoder 0 PIN A
  attachInterrupt(digitalPinToInterrupt(encoder0PinB), doEncoder0B, CHANGE);  // Encoder 0 PIN B
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoder1A, CHANGE);  // Encoder 1 PIN A
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), doEncoder1B, CHANGE);  // Encoder 1 PIN B

  // Configurar Bluetooth Serial
  BT_SERIAL.begin(38400); // Baud Rate debe ser el mismo que el archivo.py
  Serial.println("Bluetooth iniciado y listo para recibir comandos.");
}

void loop() {
  // ** Comunicación Bluetooth **
  if (BT_SERIAL.available() > 0) {      // Verificar si hay datos disponibles en Bluetooth
    instruccion = readBuff();            // Leer el mensaje entrante
    if (newMsg) {                        // Si se recibió un mensaje completo
      newMsg = false;                    // Reiniciar la flag
      Serial.print("Mensaje recibido: ");
      Serial.println(instruccion);
      
      if (instruccion.startsWith("A")) { // Verificar que el mensaje empiece con 'A'
        String valorStr = instruccion.substring(1); // Obtener la parte numérica del mensaje
        int valor = valorStr.toInt();                // Convertir el valor a int
        Serial.print("Valor de velocidad recibido: ");
        Serial.println(valor);

        // Actualizar los setPoints del PID
        setPoint0 = -valor; // Motor 0 (izquierdo)
        Serial.print("SetPoint0 actualizado a: ");
        Serial.println(setPoint0);
      }
      if (instruccion.startsWith("B")) { // Verificar que el mensaje empiece con 'B'
        String valorStr = instruccion.substring(1); // Obtener la parte numérica del mensaje
        int valor = valorStr.toInt();                // Convertir el valor a int
        Serial.print("Valor de velocidad recibido: ");
        Serial.println(valor);

        // Actualizar los setPoints del PID
        setPoint1 = valor;  // Motor 1 (derecho)
        Serial.print("SetPoint1 actualizado a: ");
        Serial.println(setPoint1);
      }

      else {
        Serial.println("Formato de mensaje no reconocido.");
      }
    }
  }

  // ** Control PID de Velocidad **
  if ((micros() - time_ant) >= Period) { // Ejecutar cada Period (10 ms)
    newtime = micros();                  // Guardar el tiempo actual

    // Actualizando información de los encoders
    newposition0 = encoder0Pos;
    newposition1 = encoder1Pos;

    // Calcular la velocidad del motor en unidades de RPM
    float rpm = 31250.0; // Ajusta según tu configuración
    vel0 = (float)(newposition0 - oldposition0) * rpm / (newtime - time_ant); // RPM Motor 0
    vel1 = (float)(newposition1 - oldposition1) * rpm / (newtime - time_ant); // RPM Motor 1
    oldposition0 = newposition0;
    oldposition1 = newposition1;

    // Control PID de velocidad para ambos motores
    motorout1 = controlPID0(vel0);  // PID para motor 0
    motorout2 = controlPID1(vel1);  // PID para motor 1

    // Limitar la salida del control a valores válidos para los motores
    motorout1 = constrain(motorout1, -400, 400);
    motorout2 = constrain(motorout2, -400, 400);

    // Aplicar las velocidades a los motores
    md.setM1Speed(motorout1);  // Aplicar la velocidad calculada al motor 1
    md.setM2Speed(motorout2);  // Aplicar la velocidad calculada al motor 2

    // Reportar datos en el Monitor Serial para depuración
    //Serial.print("Tiempo: ");
    //Serial.print(newtime);
    //Serial.print(" | Motor0_Pos: ");
    //Serial.print(newposition0);
    //Serial.print(" | Motor1_Pos: ");
   // Serial.print(newposition1);
    Serial.print(" | Motor0_RPM: ");
    Serial.print(vel0);
    Serial.print(" | Motor1_RPM: ");
    Serial.print(vel1);
    Serial.print(" | Motor0_Output: ");
    Serial.print(motorout1);
    Serial.print(" | Motor1_Output: ");
    Serial.println(motorout2);

    time_ant = newtime; // Actualizar el tiempo anterior
  }
}
