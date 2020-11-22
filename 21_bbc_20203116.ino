#include <Servo.h>

// Arduino pin assignment
#define PIN_SERVO 10
#define PIN_IR A0

#define _DIST_TARGET 255

#define _DUTY_MIN 1700
#define _DUTY_NEU 1580
#define _DUTY_MAX 1460
#define _DIST_ALPHA 0.5

int a, b;

float duty_min, duty_max;
float dist_raw, dist_cail, dist_ema, alpha;
Servo myservo;

void setup() {
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);
  
// initialize USS related variables
  duty_min = _DUTY_MIN; 
  duty_max = _DUTY_MAX;
  dist_raw = dist_cail = dist_ema = 0.0; // raw distance output from USS (unit: mm)
  alpha = _DIST_ALPHA;
  
// initialize serial port
  Serial.begin(57600);

  a = 82;
  b = 265;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  dist_cail = 100 + 300.0 / (b - a) * (ir_distance() - a);
  dist_ema = (alpha*dist_cail) + ((1-alpha)*dist_ema);
  dist_raw = dist_ema;

  if(dist_raw > 255) {
    myservo.writeMicroseconds(duty_max);
    delay(100);
  }
  else {
    myservo.writeMicroseconds(duty_min);
    delay(100);
  }

  Serial.print("min:0,max:500,dist:");
  Serial.println(dist_raw);
}
