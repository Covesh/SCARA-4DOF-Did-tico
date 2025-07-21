// =================================================================
//
//   Editar
//
// =================================================================

#include <AccelStepper.h>
#include <Servo.h> 
#include <math.h>

// ======== Definições de Pinos e Constantes Físicas ========
#define EN_PIN 8
#define LIMIT_J1_PIN 9
#define LIMIT_J2_PIN 11
#define LIMIT_J3_PIN 10
#define LIMIT_J4_PIN 17

// --- Definições para o Servo da Garra ---
#define SERVO_PIN A0
const int SERVO_OPEN_ANGLE = 45;
const int SERVO_CLOSE_ANGLE = 175;


// --- Constantes Físicas do Robô (Calibradas) ---
// a1: Altura total desde a base até à junta do efetuador quando d2=0.
// a3: Comprimento vertical do link do efetuador.
// a5: Altura da garra.
// L1, L2: Comprimento dos braços principais do SCARA.
const double a1 = 370.7, a2 = 200.0, a3 = 53.7, a4 = 200.0;
const double a5 = 147.0;
const double L1 = 200.0, L2 = 200.0;

// --- Constantes dos Motores de Passo ---
const double STEPS_PER_REV_BASE = 200.0;
const int     GEAR_RATIO_J1 = 11, GEAR_RATIO_J3 = 3, GEAR_RATIO_J4 = 3;
const int     MICROSTEP_J1 = 16, MICROSTEP_J2 = 8, MICROSTEP_J3 = 16, MICROSTEP_J4 = 8;
const double LEAD_SCREW_PITCH_J2 = 8.0; // Passo de 8mm do fuso da junta 2
const double STEPS_PER_MM_J2 = (STEPS_PER_REV_BASE * MICROSTEP_J2) / LEAD_SCREW_PITCH_J2;

// ======== Constantes de Homing ========
const float  HOMING_SPEED_FAST = 1000.0, HOMING_SPEED_SLOW = 100.0;
const float  HOMING_SPEED_SLOW_J3 = 250.0;
const float  HOMING_ACCELERATION = 1000.0;
const double HOMING_BACKOFF_DEG  = 5.0, HOMING_BACKOFF_MM = 5.0;
const int    LIMIT_SWITCH_ACTIVE_STATE = LOW;

// ======== Posições e Offsets de Home ========
const double J1_HOME_POS_DEG = 0.0, J2_HOME_POS_MM  = 0.0, J3_HOME_POS_DEG = 0.0, J4_HOME_POS_DEG = 0.0;
const double J1_HOME_OFFSET_DEG = 5.0, J2_HOME_OFFSET_MM  = 2.0, J3_HOME_OFFSET_DEG = 136.0, J4_HOME_OFFSET_DEG = 143.0;

// ======== Limites de Software ========
const double J1_MIN_DEG = 0.0,   J1_MAX_DEG = 180.0;
const double J2_MIN_MM  = 0.0,   J2_MAX_MM  = 170.0; 
const double J3_MIN_DEG = -135.0, J3_MAX_DEG = 135.0;
const double J4_MIN_DEG = -135.0, J4_MAX_DEG = 135.0;

// ======== Constantes para HMI Avançada ========
const float SPEED_J1 = 6000, SPEED_J2 = 4000, SPEED_J3 = 4000, SPEED_J4 = 1500;
const float ACCEL_J1 = 2500, ACCEL_J2 = 2000, ACCEL_J3 = 2000, ACCEL_J4 = 1000;

// ======== Instâncias e Variáveis Globais ========
AccelStepper stp1(1, 2, 5); AccelStepper stp2(1, 3, 6);
AccelStepper stp3(1, 4, 7); AccelStepper stp4(1, 12, 13);
Servo gripperServo; 

bool is_moving = false, is_calibrated = false, is_calibrating = false, motors_enabled = false;
String inputString = "";
float speed_multiplier = 1.0;
double current_j1=0, current_d2=0, current_j3=0, current_j4=0, current_x=0, current_y=0, current_z=0;
int current_servo_angle = SERVO_OPEN_ANGLE; 

// Variáveis para o temporizador não-bloqueante da garra
bool is_gripper_timer_active = false;
unsigned long gripper_timer_start = 0;
const unsigned long GRIPPER_MOVE_TIME = 600; // Tempo em ms para a garra se mover

// ======== Funções Principais ========
void setup() {
  Serial.begin(9600); inputString.reserve(200);
  pinMode(EN_PIN, OUTPUT); digitalWrite(EN_PIN, HIGH);
  pinMode(LIMIT_J1_PIN, INPUT_PULLUP); pinMode(LIMIT_J2_PIN, INPUT_PULLUP);
  pinMode(LIMIT_J3_PIN, INPUT_PULLUP); pinMode(LIMIT_J4_PIN, INPUT_PULLUP);
  
  //stp1.setEnablePin(EN_PIN);
  stp1.setPinsInverted(false, false, true);
  //stp2.setEnablePin(EN_PIN); 
  stp2.setPinsInverted(false, false, true);
  //stp3.setEnablePin(EN_PIN); 
  stp3.setPinsInverted(true, false, true);
  //stp4.setEnablePin(EN_PIN); 
  stp4.setPinsInverted(true, false, true);
  updateMotorSpeeds(); Serial.println("MSG:Robo SCARA pronto.");
}

void loop() {
  // Verifica se há novos comandos na porta serial
  if (Serial.available() > 0) {
    char receivedChar = Serial.read();
    if (receivedChar == '\n') { 
      handleCommand(inputString); 
      inputString = ""; 
    } else { 
      inputString += receivedChar; 
    }
  }
  
  // Se os motores estiverem em movimento, executa o passo seguinte
  if (is_moving) {
    bool m1 = stp1.run(), m2 = stp2.run(), m3 = stp3.run(), m4 = stp4.run();
    // Se todos os motores pararam, o movimento terminou
    if (!m1 && !m2 && !m3 && !m4) {
      is_moving = false; 
      updateCurrentPosition(); 
      reportPosition(); 
      Serial.println("MSG:Movimento concluido.");
    }
  }

  // Gestor do temporizador não-bloqueante da garra
  if (is_gripper_timer_active) {
    // Verifica se já passou o tempo definido para o movimento da garra
    if (millis() - gripper_timer_start >= GRIPPER_MOVE_TIME) {
      is_gripper_timer_active = false; // Desativa o temporizador
      // Se a posição final for a de "aberto", desliga o servo para economizar energia
      if (current_servo_angle == SERVO_OPEN_ANGLE) {
        gripperServo.detach();
      }
    }
  }
}

/**
 * @brief Interpreta e executa os comandos recebidos via serial.
 * @param command A string de comando recebida da HMI.
 */
void handleCommand(String command) {
  command.trim();
  if (!command.startsWith("GOTO_IK")) { Serial.print("Comando Recebido: "); Serial.println(command); }
  
  if (command == "E_STOP") {
    stp1.stop(); stp2.stop(); stp3.stop(); stp4.stop();
    is_moving = false;
    digitalWrite(EN_PIN, HIGH); motors_enabled = false;
    Serial.println("MSG:PARAGEM DE EMERGENCIA.");
    Serial.println("MSG:Motores DESLIGADOS");
  } else if (command == "AUTO_CAL") {
    if (is_calibrating) { Serial.println("MSG:Calibracao ja em andamento."); }
    else { runAutoCalibration(); }
  } else if (command.startsWith("GOTO_IK:")) {
    if (!is_calibrated || !motors_enabled) { Serial.println("ERR:NOT_READY"); return; }
    command.remove(0, 8); char cmdBuffer[50]; command.toCharArray(cmdBuffer, 50);
    float x=atof(strtok(cmdBuffer,",")); float y=atof(strtok(NULL,","));
    float z=atof(strtok(NULL,",")); float u=atof(strtok(NULL,","));
    moveToPosition(x, y, z, u);
  } else if (command.startsWith("GOTO_JOINTS:")) {
    if (!is_calibrated || !motors_enabled) { Serial.println("ERR:NOT_READY"); return; }
    command.remove(0, 12); char cmdBuffer[50]; command.toCharArray(cmdBuffer, 50);
    float t1=atof(strtok(cmdBuffer,",")); float d2=atof(strtok(NULL,","));
    float t3=atof(strtok(NULL,",")); float t4=atof(strtok(NULL,","));
    moveToJoints(t1, d2, t3, t4);
  } else if (command.startsWith("GOTO_HOME")) {
    if (!is_calibrated || !motors_enabled) { Serial.println("ERR:NOT_READY"); return; }
    moveToJoints(J1_HOME_POS_DEG, J2_HOME_POS_MM, J3_HOME_POS_DEG, J4_HOME_POS_DEG);
  } else if (command.startsWith("SET_SPEED_MULT:")) {
    command.remove(0, 15); speed_multiplier = command.toFloat();
    if (speed_multiplier < 0.5) speed_multiplier = 0.5; if (speed_multiplier > 1.5) speed_multiplier = 1.5;
    updateMotorSpeeds(); Serial.print("MSG:Speed multiplier set to "); Serial.println(speed_multiplier);
  } else if (command.startsWith("MOTORS:")) {
      command.remove(0, 7); int state = command.toInt();
      if(state == 1 && is_calibrated) {
          digitalWrite(EN_PIN, LOW); motors_enabled = true; Serial.println("MSG:Motores LIGADOS");
      } else {
          digitalWrite(EN_PIN, HIGH); motors_enabled = false; Serial.println("MSG:Motores DESLIGADOS");
      }
  } else if (command == "GRIPPER_OPEN") {
      setGripperAngle(SERVO_OPEN_ANGLE);
      Serial.println("MSG:Garra aberta.");
      reportPosition();
  } else if (command == "GRIPPER_CLOSE") {
      setGripperAngle(SERVO_CLOSE_ANGLE);
      Serial.println("MSG:Garra fechada.");
      reportPosition();
  } else if (command.startsWith("GRIPPER_SET:")) {
      command.remove(0, 12);
      int angle = command.toInt();
      setGripperAngle(angle);
      reportPosition();
  } else { Serial.println("ERR:UNKNOWN_COMMAND"); }
}

/**
 * @brief Move o servo da garra e ativa um temporizador para o desligar.
 * Esta função não usa delay(), permitindo que os outros motores continuem a funcionar.
 * @param angle O ângulo alvo para o servo.
 */
void setGripperAngle(int angle) {
  angle = constrain(angle, SERVO_OPEN_ANGLE, SERVO_CLOSE_ANGLE);

  if (!gripperServo.attached()) {
    gripperServo.attach(SERVO_PIN);
    delay(15); 
  }

  gripperServo.write(angle);
  current_servo_angle = angle; 

  is_gripper_timer_active = true;
  gripper_timer_start = millis();
}

// Atualiza a velocidade e aceleração de todos os motores.

void updateMotorSpeeds() {
  stp1.setMaxSpeed(SPEED_J1 * speed_multiplier); stp1.setAcceleration(ACCEL_J1 * speed_multiplier);
  stp2.setMaxSpeed(SPEED_J2 * speed_multiplier); stp2.setAcceleration(ACCEL_J2 * speed_multiplier);
  stp3.setMaxSpeed(SPEED_J3 * speed_multiplier); stp3.setAcceleration(ACCEL_J3 * speed_multiplier);
  stp4.setMaxSpeed(SPEED_J4 * speed_multiplier); stp4.setAcceleration(ACCEL_J4 * speed_multiplier);
}

/**
 * @brief Lê a posição atual dos motores (em passos) e converte para unidades de engenharia (graus/mm).
 */
void updateCurrentPosition() {
  current_j1 = (double)stp1.currentPosition() * 360.0 / (STEPS_PER_REV_BASE * MICROSTEP_J1 * GEAR_RATIO_J1);
  current_d2 = (double)stp2.currentPosition() / STEPS_PER_MM_J2;
  current_j3 = (double)stp3.currentPosition() * 360.0 / (STEPS_PER_REV_BASE * MICROSTEP_J3 * GEAR_RATIO_J3);
  current_j4 = (double)stp4.currentPosition() * 360.0 / (STEPS_PER_REV_BASE * MICROSTEP_J4 * GEAR_RATIO_J4);
  calculateForwardKinematics(current_j1, current_d2, current_j3, current_x, current_y, current_z);
}

//Executa a rotina de calibração automática para todas as juntas.
 
void runAutoCalibration() {
  is_calibrating = true; motors_enabled = true; digitalWrite(EN_PIN, LOW); delay(100);
  Serial.println("MSG:Iniciando auto-calibracao...");
  Serial.println("MSG:Homing J2...");
  performHomingForPrismaticJoint(stp2, LIMIT_J2_PIN, STEPS_PER_MM_J2, J2_HOME_OFFSET_MM, J2_HOME_POS_MM, -1);
  Serial.println("MSG:Homing J1...");
  performHomingForJoint(stp1, LIMIT_J1_PIN, GEAR_RATIO_J1, MICROSTEP_J1, J1_HOME_OFFSET_DEG, J1_HOME_POS_DEG, -1, HOMING_SPEED_SLOW);
  Serial.println("MSG:Homing J3...");
  performHomingForJoint(stp3, LIMIT_J3_PIN, GEAR_RATIO_J3, MICROSTEP_J3, J3_HOME_OFFSET_DEG, J3_HOME_POS_DEG, -1, HOMING_SPEED_SLOW_J3);
  Serial.println("MSG:Homing J4...");
  performHomingForJoint(stp4, LIMIT_J4_PIN, GEAR_RATIO_J4, MICROSTEP_J4, J4_HOME_OFFSET_DEG, J4_HOME_POS_DEG, -1, HOMING_SPEED_SLOW);
  
  is_calibrated = true; 
  updateCurrentPosition(); 
  reportPosition();


  Serial.println("MSG:Testando garra...");
  for (int i = 0; i < 2; i++) {
    setGripperAngle(SERVO_CLOSE_ANGLE);
    delay(700); 
    setGripperAngle(SERVO_OPEN_ANGLE);
    delay(700); 
  }

  Serial.println("MSG:Calibracao concluida."); 
  is_calibrating = false;
  Serial.println("MSG:Motores LIGADOS");
}

void performHomingForJoint(AccelStepper &stepper, int limitSwitchPin, int gearRatio, int microstepFactor, double homeOffsetDeg, double finalHomeDeg, int homingDirection, float slow_speed) {
  stepper.setMaxSpeed(HOMING_SPEED_FAST); stepper.setAcceleration(HOMING_ACCELERATION);
  stepper.move(homingDirection * 1000000);
  while (digitalRead(limitSwitchPin) != LIMIT_SWITCH_ACTIVE_STATE) { stepper.run(); }
  stepper.stop(); delay(50); stepper.setCurrentPosition(0);
  long backoff_steps = (long)((-homingDirection) * HOMING_BACKOFF_DEG / 360.0 * (STEPS_PER_REV_BASE * microstepFactor * gearRatio));
  stepper.moveTo(backoff_steps);
  while (stepper.distanceToGo() != 0) { stepper.run(); }
  delay(50);
  stepper.move(homingDirection * 2 * 1000000); stepper.setMaxSpeed(slow_speed);
  while (digitalRead(limitSwitchPin) != LIMIT_SWITCH_ACTIVE_STATE) { stepper.run(); }
  stepper.stop(); delay(50); stepper.setCurrentPosition(0);
  long offset_steps = (long)((homeOffsetDeg / 360.0) * (STEPS_PER_REV_BASE * microstepFactor * gearRatio));
  stepper.moveTo(offset_steps); stepper.setMaxSpeed(HOMING_SPEED_FAST);
  while (stepper.distanceToGo() != 0) { stepper.run(); }
  long final_home_steps = (long)((finalHomeDeg / 360.0) * (STEPS_PER_REV_BASE * microstepFactor * gearRatio));
  stepper.setCurrentPosition(final_home_steps);
  Serial.println("  Homing da junta concluido.");
}

void performHomingForJoint(AccelStepper &stepper, int limitSwitchPin, int gearRatio, int microstepFactor, double homeOffsetDeg, double finalHomeDeg, int homingDirection) {
  performHomingForJoint(stepper, limitSwitchPin, gearRatio, microstepFactor, homeOffsetDeg, finalHomeDeg, homingDirection, HOMING_SPEED_SLOW);
}

void performHomingForPrismaticJoint(AccelStepper &stepper, int limitSwitchPin, double stepsPerMm, double homeOffsetMm, double finalHomeMm, int homingDirection) {
  stepper.setMaxSpeed(HOMING_SPEED_FAST); stepper.setAcceleration(HOMING_ACCELERATION);
  stepper.move(homingDirection * 1000000);
  while (digitalRead(limitSwitchPin) != LIMIT_SWITCH_ACTIVE_STATE) { stepper.run(); }
  stepper.stop(); delay(50); stepper.setCurrentPosition(0);
  long backoff_steps = (long)((-homingDirection) * HOMING_BACKOFF_MM * stepsPerMm);
  stepper.moveTo(backoff_steps);
  while (stepper.distanceToGo() != 0) { stepper.run(); }
  delay(50);
  stepper.move(homingDirection * 2 * 1000000); stepper.setMaxSpeed(HOMING_SPEED_SLOW);
  while (digitalRead(limitSwitchPin) != LIMIT_SWITCH_ACTIVE_STATE) { stepper.run(); }
  stepper.stop(); delay(50); stepper.setCurrentPosition(0);
  long offset_steps = (long)(homeOffsetMm * stepsPerMm);
  stepper.moveTo(offset_steps); stepper.setMaxSpeed(HOMING_SPEED_FAST);
  while (stepper.distanceToGo() != 0) { stepper.run(); }
  long final_home_steps = (long)(finalHomeMm * stepsPerMm);
  stepper.setCurrentPosition(final_home_steps);
  Serial.println("  Homing da junta concluido.");
}


// Calcula a cinemática inversa: converte coordenadas (X,Y,Z) em ângulos das juntas (t1,d2,t3).
// true se a solução for válida e dentro dos limites, false caso contrário.

bool calculateInverseKinematics(double x, double y, double z, double target_teta4_deg, bool elbow_up_preference, double *t1_ptr, double *d2_ptr, double *t3_ptr, double *t4_ptr) {
    double d2_calc = a1 - a3 - a5 - z;
    double r_sq = x*x + y*y;

    if (r_sq > (L1 + L2) * (L1 + L2)) return false;

    double cos_t3 = (r_sq - L1*L1 - L2*L2) / (2 * L1 * L2);

    if (cos_t3 > 1.0) cos_t3 = 1.0;
    if (cos_t3 < -1.0) cos_t3 = -1.0;

    for (int i = 0; i < 2; ++i) {
        bool use_elbow_up = (i == 0) ? elbow_up_preference : !elbow_up_preference;
        
        double t3_rad = acos(cos_t3);
        if (use_elbow_up) {
            t3_rad = -t3_rad;
        }

        double k1 = L1 + L2 * cos(t3_rad);
        double k2 = L2 * sin(t3_rad);
        double t1_rad = atan2(y, x) - atan2(k2, k1);

        double t1_deg = degrees(t1_rad);
        double t3_deg = degrees(t3_rad);

        if (t1_deg >= J1_MIN_DEG && t1_deg <= J1_MAX_DEG &&
            d2_calc >= J2_MIN_MM  && d2_calc <= J2_MAX_MM &&
            t3_deg >= J3_MIN_DEG && t3_deg <= J3_MAX_DEG) {
            
            *t1_ptr = t1_deg;
            *d2_ptr = d2_calc;
            *t3_ptr = t3_deg;
            *t4_ptr = target_teta4_deg;
            return true;
        }
    }
    return false;
}

// Move o robô para uma posição cartesiana (X,Y,Z,U).

void moveToPosition(double x, double y, double z, double u) {
    double t1, d2, t3, t4;
    bool success = calculateInverseKinematics(x, y, z, u, false, &t1, &d2, &t3, &t4); 
    
    if (success) {
        if (t4 < J4_MIN_DEG || t4 > J4_MAX_DEG) { Serial.println("ERR:LIMIT_J4"); return; }
        moveToJoints(t1, d2, t3, t4);
    } else {
        Serial.println("ERR:UNREACHABLE_OR_LIMITS");
    }
}


// juntas individuais para os seus ângulos/posições alvo.

void moveToJoints(double t1, double d2, double t3, double t4) {
  is_moving = true;
  long steps1 = (t1 / 360.0) * (STEPS_PER_REV_BASE * MICROSTEP_J1 * GEAR_RATIO_J1);
  long steps2 = d2 * STEPS_PER_MM_J2;
  long steps3 = (t3 / 360.0) * (STEPS_PER_REV_BASE * MICROSTEP_J3 * GEAR_RATIO_J3);
  long steps4 = (t4 / 360.0) * (STEPS_PER_REV_BASE * MICROSTEP_J4 * GEAR_RATIO_J4);
  stp1.moveTo(steps1); stp2.moveTo(steps2); stp3.moveTo(steps3); stp4.moveTo(steps4);
}


// @brief Calcula a cinemática direta: converte ângulos das juntas (j1,d2,j3) em coordenadas (X,Y,Z).

void calculateForwardKinematics(double j1, double d2, double j3, double& x, double& y, double& z) {
  double t1_rad = radians(j1); double t3_rad = radians(j3);
  x = L1 * cos(t1_rad) + L2 * cos(t1_rad + t3_rad);
  y = L1 * sin(t1_rad) + L2 * sin(t1_rad + t3_rad);
  z = a1 - d2 - a3 - a5;
}


// @brief Envia a posição atual do robô (juntas e cartesiana) pela porta serial.
void reportPosition() {
  Serial.print("POS:"); Serial.print(current_j1, 2); Serial.print(","); Serial.print(current_d2, 2); Serial.print(",");
  Serial.print(current_j3, 2); Serial.print(","); Serial.println(current_j4, 2);
  Serial.print("WPOS:"); Serial.print(current_x, 2); Serial.print(","); Serial.print(current_y, 2); Serial.print(",");
  Serial.print(current_z, 2); Serial.print(","); Serial.println(current_j4, 2);
  Serial.print("GRIPPER:"); Serial.println(current_servo_angle);
}
