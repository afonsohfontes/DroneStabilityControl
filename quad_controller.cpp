#include "quad_controller.h"
#include "quad_wifi_config.h"
#include "quad_esc_config.h"

#include <MPU6050_tockn.h>
#include <Wire.h>
float measures[3] = {0, 0, 0};

// ------------- Global variables used for PID controller --------------------
float errors[3];                     // Error medido : [Yaw, Pitch, Roll]
float error_sum[3]      = {0, 0, 0}; // Soma dos erros para o componente de integral: [Yaw, Pitch, Roll]
float previous_error[3] = {0, 0, 0}; // Ultimo erro para o componente derivativo : [Yaw, Pitch, Roll]

float Kp[3]        = {5.0, 2.3, 2.3};    // P : Yaw, Pitch, Roll
float Ki[3]        = {0.02, 0.04, 0.04}; // I : Yaw, Pitch, Roll
float Kd[3]        = {0, 10, 10};        // D : Yaw, Pitch, Roll

/*Inicio para PID*/
float instruction[4] = {0, 0, 0, 1400}; /*YAW, PITCH, ROLL, Throlle*/

int pulse_length_esc1 = 1000,
    pulse_length_esc2 = 1000,
    pulse_length_esc3 = 1000,
    pulse_length_esc4 = 1000;
// ---------------------------------------------------------------------------

MPU6050 mpu6050(Wire);
int controller_mode = 0;

/**

   (A) (B)     x
     \ /     z ↑
      X       \|
     / \       +----→ y
   (C) (D)

   Motors A & D estao sentido horario.
   Motors B & C estao sentido anti horario.


   @return void
*/

void PID_calculateOutput() {

  float delta_err[3] = {0, 0, 0};          // Error delta: Yaw, Pitch, Roll
  float yaw_pid      = 0;
  float pitch_pid    = 0;
  float roll_pid     = 0;

  // Initialize motor commands with throttle
  pulse_length_esc1 = instruction[THROTTLE];
  pulse_length_esc2 = instruction[THROTTLE];
  pulse_length_esc3 = instruction[THROTTLE];
  pulse_length_esc4 = instruction[THROTTLE];

  
  if (instruction[THROTTLE] >= 1012) {
    // Calcula soma dos erros Coeficientes da Integral:
    error_sum[YAW]   += errors[YAW];
    error_sum[PITCH] += errors[PITCH];
    error_sum[ROLL]  += errors[ROLL];

    // Calcula o erro delta: Coefficient derivativo
    delta_err[YAW]   = errors[YAW]   - previous_error[YAW];
    delta_err[PITCH] = errors[PITCH] - previous_error[PITCH];
    delta_err[ROLL]  = errors[ROLL]  - previous_error[ROLL];

    // Salva o erro para a proxima medida
    previous_error[YAW]   = errors[YAW];
    previous_error[PITCH] = errors[PITCH];
    previous_error[ROLL]  = errors[ROLL];

    // PID = e.Kp + ∫e.Ki + Δe.Kd
    yaw_pid   = (errors[YAW]   * Kp[YAW])   + (error_sum[YAW]   * Ki[YAW])   + (delta_err[YAW]   * Kd[YAW]);
    pitch_pid = (errors[PITCH] * Kp[PITCH]) + (error_sum[PITCH] * Ki[PITCH]) + (delta_err[PITCH] * Kd[PITCH]);
    roll_pid  = (errors[ROLL]  * Kp[ROLL])  + (error_sum[ROLL]  * Ki[ROLL])  + (delta_err[ROLL]  * Kd[ROLL]);

    
    pulse_length_esc1 = instruction[THROTTLE] + roll_pid + pitch_pid - yaw_pid;
    pulse_length_esc2 = instruction[THROTTLE] - roll_pid + pitch_pid + yaw_pid;
    pulse_length_esc3 = instruction[THROTTLE] + roll_pid - pitch_pid + yaw_pid;
    pulse_length_esc4 = instruction[THROTTLE] - roll_pid - pitch_pid - yaw_pid;
  }
  //
  pulse_length_esc1 = minMax(pulse_length_esc1, 1100, 1600);
  pulse_length_esc2 = minMax(pulse_length_esc2, 1100, 1600);
  pulse_length_esc3 = minMax(pulse_length_esc3, 1100, 1600);
  pulse_length_esc4 = minMax(pulse_length_esc4, 1100, 1600);

}

/**
   Calcula o erro do Yaw, Pitch & Roll: Apenas a diferenca entre a medicao e o comando
   @return void
*/
void PID_calculateErrors() {
  errors[YAW]   = instruction[YAW]   - measures[YAW];
  errors[PITCH] = instruction[PITCH] - measures[PITCH];
  errors[ROLL]  = instruction[ROLL]  - measures[ROLL];
}


int minMax(float value, float min_value, float max_value) {
  if (value > max_value) {
    value = max_value;
  } else if (value < min_value) {
    value = min_value;
  }

  return value;
}

/*Update IMU readings*/

void IMU_updateVars() {
  measures[YAW] = mpu6050.getAngleZ();
  measures[PITCH] = -mpu6050.getAngleY();
  measures[ROLL] = -mpu6050.getAngleX();
}

/*Setup MPU6050*/
void IMU_setup() {
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}
/*Controller Setup Function*/
void controller_setup() {
  IMU_setup();
  ESC_setup();
}




/*Controller Periodic Fucntion chamado no loop*/
void controller_periodic() {
  if (controller_mode == 1) {
    mpu6050.update();
    IMU_updateVars();
    PID_calculateErrors();
    PID_calculateOutput();
    ESC_periodic();
  }
  else {
    ESC_stopAll();
    delay(10);
  }
}

/*Disarm controller */
void controller_disarm() {
  controller_mode = 0;
  ESC_stopAll();
  errors[YAW]   = 0;
  errors[PITCH] = 0;
  errors[ROLL]  = 0;

  error_sum[YAW]   = 0;
  error_sum[PITCH] = 0;
  error_sum[ROLL]  = 0;

  previous_error[YAW]   = 0;
  previous_error[PITCH] = 0;
  previous_error[ROLL]  = 0;
}

/*Arm controller */
void controller_arm() {
  ESC_stopAll();
  errors[YAW]   = 0;
  errors[PITCH] = 0;
  errors[ROLL]  = 0;

  error_sum[YAW]   = 0;
  error_sum[PITCH] = 0;
  error_sum[ROLL]  = 0;

  previous_error[YAW]   = 0;
  previous_error[PITCH] = 0;
  previous_error[ROLL]  = 0;
  controller_mode = 1;
}

/*Period ESC function*/
void ESC_periodic() {
  ESC_write(motA, pulse_length_esc1);
  ESC_write(motB, pulse_length_esc1);
  ESC_write(motC, pulse_length_esc1);
  ESC_write(motD, pulse_length_esc1);
}

void controller_info() {
  Serial.print("Roll=");
  Serial.print(measures[ROLL]);
  Serial.print(" Pitch=");
  Serial.print(measures[PITCH]);
  Serial.print(" Yaw=");
  Serial.print(measures[YAW]);

  Serial.print(" ESC1=");
  Serial.print(pulse_length_esc1);
  Serial.print(" ESC2=");
  Serial.print(pulse_length_esc2);
  Serial.print(" ESC3=");
  Serial.print(pulse_length_esc3);
  Serial.print(" ESC4=");
  Serial.println(pulse_length_esc4);
}
