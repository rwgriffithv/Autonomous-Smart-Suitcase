#include <Wire.h>
#include <Adafruit_Sensor.h>

#include <Adafruit_LSM303.h>
#include <Adafruit_LSM303_U.h>

#include <Adafruit_L3GD20.h>
#include <Adafruit_L3GD20_U.h>

#include <Adafruit_9DOF.h>

#include "vector.h"


/* Assign a unique ID to the sensors */
Adafruit_9DOF                dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);

sensors_event_t accel_event;
sensors_event_t gyro_event;

unsigned long millis_prev;
unsigned long millis_curr;
const unsigned long period = 10;

unsigned long steps;

struct vector accel_bias;
struct vector velocity_trans_curr;

struct vector velocity_ang_curr;
struct vector velocity_ang_bias;

/* PID 
   v_y and w_z will decide turning
   v_xy mag will decide speed
*/
float v_y_goal;
float v_y_curr;

float v_xy_mag_goal;
float v_xy_mag_curr;

float w_z_goal;
float w_z_curr;

const float K_p_v_y = 0.2;
const float K_d_v_y = 0.05;
const float K_p_v_xy_mag = 0.3;
const float K_d_v_xy_mag = 0.1;
const float K_p_w_z = 4;
const float K_d_w_z = 2;

const float alpha = 20.0;
const float PWM_MAX = 255;

void computeBias() {
  accel_bias = (struct vector){.x = 0, .y = 0, .z = 0};
  velocity_ang_bias = (struct vector){.x = 0, .y = 0, .z = 0};
  
  for (int i = 0; i < 999; i++) {
    accel.getEvent(&accel_event);
    gyro.getEvent(&gyro_event);
    accel_bias.x += accel_event.acceleration.x;
    accel_bias.y += accel_event.acceleration.y;
    accel_bias.z += accel_event.acceleration.z;

    gyro.getEvent(&gyro_event);
    velocity_ang_bias.x += gyro_event.gyro.x;
    velocity_ang_bias.y += gyro_event.gyro.y;
    velocity_ang_bias.z += gyro_event.gyro.z;
  }
     
  vector_multiply(&accel_bias, (-1.0/1000.0), &accel_bias);
  vector_multiply(&velocity_ang_bias, (-1.0/1000.0), &velocity_ang_bias);

  /* OUTPUT BIAS CALC
  Serial.print(F("Accel X Bias: "));
  Serial.print(accel_bias.x);
  Serial.print(F("; "));
  Serial.print(F("Accel Y Bias: "));
  Serial.print(accel_bias.y);
  Serial.print(F("; "));
  Serial.print(F("Accel Z Bias: "));
  Serial.print(accel_bias.z);
  Serial.print(F("V_Ang X Bias: "));
  Serial.print(accel_bias.x);
  Serial.print(F("; "));
  Serial.print(F("V_Ang Y Bias: "));
  Serial.print(accel_bias.y);
  Serial.print(F("; "));
  Serial.print(F("V_Ang Z Bias: "));
  Serial.print(accel_bias.z);
  Serial.println(F(""));
  */
}


void setup() {
  Serial.begin(9600);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // signal user to be still, is configuring
  delay(2000);
  
  accel.begin();
  gyro.enableAutoRange(true);
  gyro.begin();
  computeBias();
  velocity_trans_curr = (struct vector){.x = 0, .y = 0, .z = 0};
  velocity_ang_curr = (struct vector){.x = 0, .y = 0, .z = 0};

  delay(1000);
  digitalWrite(LED_BUILTIN, LOW); // configuration over
  steps = 0;
  v_y_goal = 0;
  v_xy_mag_goal = 0;
  w_z_goal = 0;
  millis_prev = millis();
}


void loop() {
  millis_curr = millis();
  
  accel.getEvent(&accel_event);
  /* summing (integrating) instantaneous acceleration to get velocity */
  velocity_trans_curr.x += accel_event.acceleration.x;
  velocity_trans_curr.y += accel_event.acceleration.y;
  velocity_trans_curr.z += accel_event.acceleration.z;
  vector_add(&velocity_trans_curr, &accel_bias, &velocity_trans_curr);
  /* just taking average of angular velocity */
  gyro.getEvent(&gyro_event);
  velocity_ang_curr.x += gyro_event.gyro.x;
  velocity_ang_curr.y += gyro_event.gyro.y;
  velocity_ang_curr.z += gyro_event.gyro.z;
  
  ++steps;
  
  if (millis_curr - millis_prev >= period) {
    millis_prev = millis_curr;

    /* get translational velocity by integrating average acceleration */
    vector_multiply(&velocity_trans_curr, 1.0*period/steps, &velocity_trans_curr);
    /* take average angular velocity */
    vector_multiply(&velocity_ang_curr, 1.0/steps, &velocity_ang_curr);
    vector_add(&velocity_ang_curr, &velocity_ang_bias, &velocity_ang_curr);

    /* TODO: read bluetooth, byte by byte into goals */
    float v_y_goal_prev = v_y_goal;
    // v_y_goal = bluetooth_read.y;
    v_y_goal = 0.0; // temp for testing
    
    float v_xy_mag_goal_prev = v_xy_mag_goal;
    // v_xy_mag_goal = bluetooth_read.xy;
    v_xy_mag_goal = 3.0; // temp for testing
    
    float w_z_goal_prev = w_z_goal;
    // w_z_goal = bluetooth_read.w;
    w_z_goal = 0.0; // temp for testing
    
    float v_y_prev = v_y_curr;
    v_y_curr = velocity_trans_curr.y;
    
    float v_xy_mag_prev = v_xy_mag_curr;
    v_xy_mag_curr = vector_xy_magnitude(&velocity_trans_curr);
    
    float w_z_prev = w_z_curr;
    w_z_curr = velocity_ang_curr.z;

    float v_y_error_p = v_y_goal - v_y_curr;
    float v_xy_mag_error_p = v_xy_mag_goal - v_xy_mag_curr;
    float w_z_error_p = w_z_goal - w_z_curr;
    
    float v_y_error_d = (v_y_goal - v_y_goal_prev) - (v_y_curr - v_y_prev);
    float v_xy_mag_error_d = (v_xy_mag_goal - v_xy_mag_goal_prev) - (v_xy_mag_curr - v_xy_mag_prev);
    float w_z_error_d = (w_z_goal - w_z_goal_prev) - (w_z_curr - w_z_prev);
    
    float pwm_l = (K_p_v_xy_mag * v_xy_mag_error_p - K_p_v_y * v_y_error_p - K_p_w_z * w_z_error_p);
    pwm_l += (K_d_v_xy_mag * v_xy_mag_error_d - K_d_v_y * v_y_error_d - K_d_w_z * w_z_error_d);

    float pwm_r = (K_p_v_xy_mag * v_xy_mag_error_p + K_p_v_y * v_y_error_p + K_p_w_z * w_z_error_p);
    pwm_r += (K_d_v_xy_mag * v_xy_mag_error_d + K_d_v_y * v_y_error_d + K_d_w_z * w_z_error_d);

    uint8_t wheel_dir_r = 0; // 0 for forward, 1 for backwards
    uint8_t wheel_dir_l = 0; 

    if (pwm_l < 0) {
      wheel_dir_l = 1;
      pwm_l = -1.0 * pwm_l;
    }
    if (pwm_r < 0) {
      wheel_dir_r = 1;
      pwm_r = -1.0 * pwm_r;
    }

    pwm_l *= alpha;
    pwm_r *= alpha;

    if (pwm_l > PWM_MAX) {
       pwm_l = 200;
    }
    if (pwm_r > PWM_MAX) {
       pwm_r = 200;
    }

    /* OUTPUT ERROR AND PWM OUTPUTS */
    Serial.print(F("PL: "));
    Serial.print(pwm_l);
    Serial.print(F("; "));
    Serial.print(F("DL: "));
    Serial.print(wheel_dir_l);
    Serial.print(F("; "));
    Serial.print(F("PR: "));
    Serial.print(pwm_r);
    Serial.print(F("; "));
    Serial.print(F("DR: "));
    Serial.print(wheel_dir_r);
    Serial.print(F("; "));
    Serial.print(F("W error P: "));
    Serial.print(w_z_error_p);
    Serial.print(F("; "));
    Serial.print(F("Y error P: "));
    Serial.print(v_y_error_p);
    Serial.print(F("; "));
    Serial.print(F("XY error P: "));
    Serial.print(v_xy_mag_error_p);
    Serial.println(F(""));
    /**/

    /* reset so error doesn't accumulate */
    velocity_trans_curr = (struct vector){.x = 0, .y = 0, .z = 0};
    velocity_ang_curr = (struct vector){.x = 0, .y = 0, .z = 0};
    
    steps = 0;
  }
  
}
