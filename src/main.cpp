#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <PID_v1.h>

//================================================================
// 1. DEFINIÇÃO DE PINOS (Hardware)
//================================================================

// --- Pinos do Sensor IMU (MPU6050) ---
// **ALTERADO:** Pinos I2C padrão da ESP32 DevKit clássica
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// --- Pinos da Ponte H (Ex: L298N ou TB6612FNG) ---
// Motores N20 750RPM
// Estes pinos (10-15) geralmente são seguros na DevKit.
// Cuidado: O GPIO 12 é um pino de boot. Geralmente funciona
// como saída PWM após o boot, mas se tiver problemas, troque-o.

// Motor A (Direito)
#define MOTOR_A_IN1 10 // Pino IN1 da Ponte H
#define MOTOR_A_IN2 11 // Pino IN2 da Ponte H
#define MOTOR_A_PWM 12 // Pino ENA (PWM para velocidade) da Ponte H

// Motor B (Esquerdo)
#define MOTOR_B_IN1 13 // Pino IN3 da Ponte H
#define MOTOR_B_IN2 14 // Pino IN4 da Ponte H
#define MOTOR_B_PWM 15 // Pino ENB (PWM para velocidade) da Ponte H

// Configurações do PWM (ESP32 usa LEDC)
#define PWM_FREQUENCY 5000 // Frequência em Hz
#define PWM_RESOLUTION 8   // Resolução de 8 bits (0-255)
#define MOTOR_A_PWM_CHANNEL 0 // Canal LEDC 0
#define MOTOR_B_PWM_CHANNEL 1 // Canal LEDC 1

//================================================================
// 2. VARIÁVEIS GLOBAIS
//================================================================

// --- Sensor IMU ---
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;
float angle_pitch = 0.0;
float gyro_pitch_rate_offset = 0.0; // Offset do giroscópio

// --- Filtro Complementar ---
float angle_accel = 0.0;
unsigned long last_filter_time = 0;

// --- Controlador PID ---
// **AVISO: Estes valores (Kp, Ki, Kd) SÃO EXEMPLOS!**
// **Você DEVE sintonizar (fazer o "tuning") deles para o seu robô!**
double Kp = 20.0;  // Proporcional
double Ki = 0.5;   // Integral
double Kd = 15.0;  // Derivativo

double pid_setpoint = 0.0; // Setpoint (ponto de equilíbrio), 0 graus.
double pid_input = 0.0;    // Entrada (ângulo atual do robô)
double pid_output = 0.0;   // Saída (esforço de controle para os motores)

PID pid(&pid_input, &pid_output, &pid_setpoint, Kp, Ki, Kd, REVERSE);

//================================================================
// 3. FUNÇÃO DE CONTROLE DOS MOTORES
//================================================================

/**
 * @brief Controla os motores com base na saída do PID.
 * @param speed O valor de saída do PID (pode ser negativo ou positivo).
 */
void controlMotors(double speed) {
  // Limita a saída do PID para a faixa de PWM (0-255)
  int pwm_output = constrain(abs(speed), 0, 255);

  // Define a direção com base no sinal do 'speed'
  if (speed > 0) {
    // Mover para "frente" (para corrigir a queda)
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, LOW);
    digitalWrite(MOTOR_B_IN1, HIGH);
    digitalWrite(MOTOR_B_IN2, LOW);
  } else {
    // Mover para "trás" (para corrigir a queda)
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, HIGH);
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, HIGH);
  }

  // Aplica a velocidade (PWM) aos motores
  ledcWrite(MOTOR_A_PWM_CHANNEL, pwm_output);
  ledcWrite(MOTOR_B_PWM_CHANNEL, pwm_output);
}

//================================================================
// 4. FUNÇÃO DE CALIBRAÇÃO
//================================================================

/**
 * @brief Calibra o giroscópio ao ligar.
 * Fica parado por alguns segundos e calcula o "drift" (offset).
 */
void calibrateGyro() {
  Serial.println("Calibrando Giroscópio... Mantenha o robô parado!");
  delay(1000);
  float total_rate = 0.0;
  for (int i = 0; i < 2000; i++) {
    mpu.getEvent(&a, &g, &temp);
    total_rate += g.gyro.y; // Usamos o eixo Y para o pitch
    delay(1);
  }
  gyro_pitch_rate_offset = total_rate / 2000.0;
  Serial.print("Offset do Giroscópio (Eixo Y) calculado: ");
  Serial.println(gyro_pitch_rate_offset);
}

//================================================================
// 5. SETUP (Configuração Inicial)
//================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("\nIniciando Robô de Auto-Equilíbrio...");

  // --- Inicializa I2C ---
  // Passamos os pinos personalizados para o Wire.begin()
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // --- Inicializa MPU6050 ---
  if (!mpu.begin()) {
    Serial.println("Falha ao encontrar MPU6050. Verifique as conexões!");
    Serial.println("Verifique se SDA está no Pino 21 e SCL no Pino 22.");
    while (1)
      delay(10);
  }
  Serial.println("MPU6050 encontrado!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // --- Calibra o Giroscópio ---
  calibrateGyro();

  // --- Inicializa Pinos dos Motores ---
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_A_PWM, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);

  // --- Configura os canais PWM (LEDC) ---
  ledcSetup(MOTOR_A_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_A_PWM, MOTOR_A_PWM_CHANNEL);
  ledcSetup(MOTOR_B_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_B_PWM, MOTOR_B_PWM_CHANNEL);

  // --- Configura o PID ---
  pid.SetMode(AUTOMATIC);        // Liga o PID
  pid.SetOutputLimits(-255, 255); // Define a saída min/max
  pid.SetSampleTime(10);         // Define o tempo de amostragem (10ms)

  last_filter_time = micros();
  Serial.println("Setup completo. Iniciando loop principal.");
}

//================================================================
// 6. LOOP (Loop Principal)
//================================================================

void loop() {
  // --- Cálculo do Tempo (dt) ---
  unsigned long current_time = micros();
  float dt = (current_time - last_filter_time) / 1000000.0; // delta T em segundos
  last_filter_time = current_time;

  // --- Leitura do Sensor ---
  mpu.getEvent(&a, &g, &temp);

  // --- Cálculo do Ângulo (Filtro Complementar) ---

  // 1. Ângulo pelo Acelerômetro (confiável a longo prazo)
  angle_accel = atan2(a.acceleration.x, a.acceleration.z) * RAD_TO_DEG;

  // 2. Taxa de rotação pelo Giroscópio (confiável a curto prazo)
  float gyro_pitch_rate = g.gyro.y - gyro_pitch_rate_offset;

  // 3. Fusão com Filtro Complementar
  angle_pitch = 0.98 * (angle_pitch + gyro_pitch_rate * dt) + 0.02 * (angle_accel);

  // --- Cálculo do PID ---
  pid_input = angle_pitch; // A entrada do PID é o ângulo que calculamos
  pid.Compute();           // O PID calcula o 'pid_output'

  // --- Controle dos Motores ---
  // Se o robô estiver "caído" (ex: mais de 30 graus), desliga os motores
  if (abs(angle_pitch) > 30) {
    controlMotors(0); // Desliga os motores
  } else {
    controlMotors(pid_output); // Aplica a correção do PID
  }

  // --- Debug (Impressão no Serial) ---
  // Use isso para o "tuning" do PID!
  // Serial.print("Angle:");
  // Serial.print(angle_pitch);
  // Serial.print("\t Output:");
  // Serial.println(pid_output);
}