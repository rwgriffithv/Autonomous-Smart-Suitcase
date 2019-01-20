#include <Wire.h>
#include <Adafruit_Sensor.h>

#include <Adafruit_LSM303.h>
#include <Adafruit_LSM303_U.h>

#include <Adafruit_L3GD20.h>
#include <Adafruit_L3GD20_U.h>

#include <Adafruit_9DOF.h>

#include <SoftwareSerial.h>

#include "vector.h"


/* Motor config */
const uint8_t MOTOR_L_PWM_PIN = 10;
const uint8_t MOTOR_R_PWM_PIN = 11;
const uint8_t MOTOR_L_F_PIN = 8;
const uint8_t MOTOR_L_B_PIN = 9;
const uint8_t MOTOR_R_F_PIN = 12;
const uint8_t MOTOR_R_B_PIN = 13;

/* Bluetooth config */
const uint8_t RxD = 6;
const uint8_t TxD = 7;
SoftwareSerial BTserial(RxD, TxD);
char c_buf = ' ';
union BTData {
  char c[12];
  float f[3];
};
union BTData goal_data;

/* Assign a unique ID to the sensors */
/* Adafruit_9DOF                dof   = Adafruit_9DOF(); */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);

sensors_event_t accel_event;
sensors_event_t gyro_event;

unsigned long millis_prev;
unsigned long millis_curr;
const unsigned long PERIOD = 10;

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

float pwm_l;
float pwm_r;

const float K_p_v_y = 0.2;
const float K_d_v_y = 0.05;
const float K_p_v_xy_mag = 0.3;
const float K_d_v_xy_mag = 0.1;
const float K_p_w_z = 4;
const float K_d_w_z = 2;

const float ALPHA = 20.0;
const float PWM_MAX = 255;
const float STRAIGHT_RATIO = 200.0/203.0;

bool following;


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

void reset() {
  goal_data.f[0] = 0.0;
  goal_data.f[1] = 0.0;
  goal_data.f[2] = 0.0;

  velocity_trans_curr = (struct vector){.x = 0, .y = 0, .z = 0};
  velocity_ang_curr = (struct vector){.x = 0, .y = 0, .z = 0};

  steps = 0;

  v_y_goal = 0.0;
  v_xy_mag_goal = 0.0;
  w_z_goal = 0.0;

  pwm_l = 0.0;
  pwm_r = 0.0;

  following = false;

  digitalWrite(MOTOR_L_PWM_PIN, 0);
  digitalWrite(MOTOR_R_PWM_PIN, 0);
  digitalWrite(MOTOR_L_F_PIN, HIGH);
  digitalWrite(MOTOR_L_B_PIN, LOW);
  digitalWrite(MOTOR_R_F_PIN, HIGH);
  digitalWrite(MOTOR_R_B_PIN, LOW);
}


void setup() {
  Serial.begin(9600);
  BTserial.begin(38400);

  pinMode(MOTOR_L_PWM_PIN, OUTPUT);
  pinMode(MOTOR_R_PWM_PIN, OUTPUT);
  pinMode(MOTOR_L_F_PIN, OUTPUT);
  pinMode(MOTOR_L_B_PIN, OUTPUT);
  pinMode(MOTOR_R_F_PIN, OUTPUT);
  pinMode(MOTOR_R_B_PIN, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // signal user to be still, is configuring
  delay(2000);
  
  accel.begin();
  gyro.enableAutoRange(true);
  gyro.begin();
  computeBias();

  delay(1000);
  digitalWrite(LED_BUILTIN, LOW); // configuration over

  reset();
  millis_prev = millis();
}


void loop() {  
  if (following) { /* only calculate if following */
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
  
    if (millis_curr - millis_prev >= PERIOD) {
      millis_prev = millis_curr;

      /* get translational velocity by integrating average acceleration */
      vector_multiply(&velocity_trans_curr, 1.0*PERIOD/steps, &velocity_trans_curr);
      /* take average angular velocity */
      vector_multiply(&velocity_ang_curr, 1.0/steps, &velocity_ang_curr);
      vector_add(&velocity_ang_curr, &velocity_ang_bias, &velocity_ang_curr);

      /* read bluetooth, byte by byte into goals
	 order: b, v_y_goal, v_xy_mag_goal, w_z_goal
      */
      uint8_t state = 0;
      uint8_t spin = 0;
      uint8_t stop_state = 0;
      while ((state < 13) && (spin < 49)) {
	if (BTserial.available()) {
	  c_buf = BTserial.read();
	  if (((c_buf == 'S') && !stop_state) ||
	      ((c_buf == 'T') && (stop_state == 1)) ||
	      ((c_buf == 'O') && (stop_state == 2))) {
	    ++stop_state;
	  } else if ((c_buf == 'P') && (stop_state == 3)) {
	    reset();
	  }

	  if ((c_buf == 'b') && !state) {
	    ++state;
	  } else if (state) {
	    goal_data.c[state-1] = c_buf;
	    ++state;
	  } else {
	    ++spin;
	  }
	} else {
	  ++spin;
	}
      }

      if (following) {
	float v_y_goal_prev = v_y_goal;
	v_y_goal = goal_data.f[0];
    
	float v_xy_mag_goal_prev = v_xy_mag_goal;
	v_xy_mag_goal = goal_data.f[1];
    
	float w_z_goal_prev = w_z_goal;
	w_z_goal = goal_data.f[2];
    
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
    
	pwm_l = (K_p_v_xy_mag * v_xy_mag_error_p - K_p_v_y * v_y_error_p - K_p_w_z * w_z_error_p);
	pwm_l += (K_d_v_xy_mag * v_xy_mag_error_d - K_d_v_y * v_y_error_d - K_d_w_z * w_z_error_d);

	pwm_r = (K_p_v_xy_mag * v_xy_mag_error_p + K_p_v_y * v_y_error_p + K_p_w_z * w_z_error_p);
	pwm_r += (K_d_v_xy_mag * v_xy_mag_error_d + K_d_v_y * v_y_error_d + K_d_w_z * w_z_error_d);

	if (pwm_l < 0) {
	  digitalWrite(MOTOR_L_F_PIN, LOW);
	  digitalWrite(MOTOR_L_B_PIN, HIGH);
	  pwm_l = -1.0 * pwm_l;
	} else {
	  digitalWrite(MOTOR_L_F_PIN, HIGH);
	  digitalWrite(MOTOR_L_B_PIN, LOW);
	}

	if (pwm_r < 0) {
	  digitalWrite(MOTOR_R_F_PIN, LOW);
	  digitalWrite(MOTOR_R_B_PIN, HIGH);
	  pwm_r = -1.0 * pwm_r;
	} else {
	  digitalWrite(MOTOR_R_F_PIN, HIGH);
	  digitalWrite(MOTOR_R_B_PIN, LOW);
	}

	pwm_l *= ALPHA;
	pwm_r *= ALPHA;

	if (pwm_l > PWM_MAX) {
	  pwm_l = 200;
	}
	if (pwm_r > PWM_MAX) {
	  pwm_r = 200;
	}

	/* multiply pwm by constant ratio b/c motors run at diff speeds */
	pwm_r *= 200.0/203.0;
	digitalWrite(MOTOR_L_PWM_PIN, (int)pwm_l);
	digitalWrite(MOTOR_R_PWM_PIN, (int)pwm_r);
	

	/* OUTPUT ERROR AND PWM OUTPUTS
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
	*/

	/* reset so error doesn't accumulate */
	velocity_trans_curr = (struct vector){.x = 0, .y = 0, .z = 0};
	velocity_ang_curr = (struct vector){.x = 0, .y = 0, .z = 0};
    
	steps = 0;
      }
    }
    
  } else { /* not following, check for start signal */
    uint8_t spin = 0;
    uint8_t start_state = 0;
    while (spin < 49) {
      if (BTserial.available()) {
	c_buf = BTserial.read();
	if (((c_buf == 'S') && !start_state) ||
	    ((c_buf == 'T') && (start_state == 1)) ||
	    ((c_buf == 'A') && (start_state == 2)) ||
	    ((c_buf == 'R') && (start_state == 3))) {
	  ++start_state;
	} else if ((c_buf == 'T') && (start_state == 4)) {
	  following = true;
	} else {
	  ++spin;
	}
      } else {
	++spin;
      }
    }
  }

}
