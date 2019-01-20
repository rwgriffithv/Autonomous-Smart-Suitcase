#include <Wire.h>
#include <Adafruit_Sensor.h>

#include <Adafruit_LSM303.h>
#include <Adafruit_LSM303_U.h>

#include <Adafruit_L3GD20.h>
#include <Adafruit_L3GD20_U.h>

#include <Adafruit_9DOF.h>

#include <SoftwareSerial.h>

#include "vector.h"

/* capacitive touch */
const uint8_t TOUCH_BTN_PIN = 7;
uint8_t btn_state;
uint8_t btn_state_last;
unsigned long last_debounce_millis;
const unsigned long DEBOUNCE_DELAY = 1000;

/* Bluetooth config */
const uint8_t RxD = 2;
const uint8_t TxD = 3;
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

bool following = false;


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
  btn_state = 0;
  btn_state_last = 0;

  goal_data.f[0] = 0.0;
  goal_data.f[1] = 0.0;
  goal_data.f[2] = 0.0;

  velocity_trans_curr = (struct vector){.x = 0, .y = 0, .z = 0};
  velocity_ang_curr = (struct vector){.x = 0, .y = 0, .z = 0};

  steps = 0;

  following = false;
}


void setup() {
  Serial.begin(9600);
  BTserial.begin(38400);
  
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
  last_debounce_millis = millis_prev;
}


void loop() {
  millis_curr = millis();
  uint8_t btn_state_new = digitalRead(TOUCH_BTN_PIN);
  
  if ((millis_curr - last_debounce_millis) > DEBOUNCE_DELAY) {
    if (btn_state_new != btn_state) {
      Serial.print(F("BUTTON PRESS   ;  "));
      btn_state = btn_state_new;
      last_debounce_millis = millis_curr;
      if (btn_state_new == HIGH) { /* was pressed (rising edge) */
	if (following) { /* send STOP */
	  char msg[5] = "STOP";
	  for (uint8_t i = 0; i < 4; ++i) {
	    Serial.print(F("sending char "));
	    Serial.print(msg[i]);
	    BTserial.write(msg[i]);
	  }
	} else { /* send START */
	  Serial.print(F("sending START"));
	  Serial.println(F(""));
	  char msg[6] = "START";
	  for (uint8_t i = 0; i < 5; ++i) {
	    BTserial.write(msg[i]);
	  }
	}

	following = !following;
      }
    }
  }
  

  if (following) {
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

      /* write bluetooth, byte by byte into goals
	 order: b, v_y_goal, v_xy_mag_goal, w_z_goal
      */
      goal_data.f[0] = velocity_trans_curr.y;
      goal_data.f[1] = vector_xy_magnitude(&velocity_trans_curr);
      goal_data.f[2] = velocity_ang_curr.z;

      uint8_t i = 0;
      for (i = 0; i < 13; ++i) {
	if (i) {
	  c_buf = goal_data.c[i-1];
	} else {
	  c_buf = 'b';
	}
	BTserial.write(c_buf);
      }

      /* reset so error doesn't accumulate */
      velocity_trans_curr = (struct vector){.x = 0, .y = 0, .z = 0};
      velocity_ang_curr = (struct vector){.x = 0, .y = 0, .z = 0};
    
      steps = 0;
    }

  }
}
