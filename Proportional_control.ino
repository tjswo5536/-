#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

// Framework setting
#define _DIST_TARGET 255
#define _DIST_MIN 100
#define _DIST_MAX 410

// Distance sensor
#define _DIST_ALPHA 0.6

// Servo range
#define _DUTY_MIN 1340
#define _DUTY_NEU 1500
#define _DUTY_MAX 1800

// Servo speed control
#define _SERVO_ANGLE 30.0
#define _SERVO_SPEED 60.0

// Event periods
#define _INTERVAL_DIST 20
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100

// PID parameters
#define _KP 1.1

#define a 82
#define b 265

//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval;
int duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

void setup() {
// initialize GPIO pins for LED and attach servo
  myservo.attach(PIN_SERVO); // attach servo
  pinMode(PIN_LED,OUTPUT); // initialize GPIO pins

// move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU);
  duty_curr = _DUTY_NEU;

// initialize serial port
  Serial.begin(57600);

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / _SERVO_ANGLE) * (_INTERVAL_SERVO / 1000.0);
}

void loop() {

/////////////////////
// Event generator //
///////////////////// 

  unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }

  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO ) {
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }

  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL ) {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }

////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
    event_dist = false;

  // get a distance reading from the distance sensor
    dist_ema = ir_distance_filtered();
   
  // PID control logic
    error_curr = dist_ema - _DIST_TARGET;
    pterm = error_curr;
    control = _KP * pterm;
  }

  // duty_target = f(duty_neutral, control)
  if(control > 0){
    duty_target = _DUTY_NEU + control + (_DUTY_MAX - _DUTY_NEU) * (_SERVO_ANGLE / 180.0);
  }
  else {
    duty_target = _DUTY_NEU + control + (_DUTY_NEU - _DUTY_MIN) * (_SERVO_ANGLE / 180.0);
  }

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
  if(duty_target > _DUTY_MAX) {
    duty_target = _DUTY_MAX;
  }
  else if(duty_target < _DUTY_MIN) {
    duty_target = _DUTY_MIN;
  }
  
  if(event_servo) {
    event_servo = false;

  // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }
    else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }

  // update servo position
    myservo.writeMicroseconds(_DUTY_MAX + _DUTY_MIN - duty_curr);
  }

  if(event_serial) {
    event_serial = false;
    Serial.print("Min:0,Low:200,dist:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(pterm);
    Serial.print(",duty_target:");
    Serial.print(duty_target);
    Serial.print(",duty_curr:");
    Serial.print(duty_curr);
    Serial.println(",High:310,Max:2000");
  }
}

float ir_distance(void) { // return value unit: mm
  float value;
  float volt = float(analogRead(PIN_IR));
  value = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return 100 + 300.0 / (b - a) * (value - a);
}

float ir_distance_filtered(void) { // return value unit: mm
  dist_raw = ir_distance();
  return _DIST_ALPHA * dist_raw + (1 - _DIST_ALPHA) * dist_ema;
}
