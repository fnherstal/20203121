#include <Servo.h>
Servo myservo;
// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

#define SND_VEL 346.0
#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 430 // maximum distance to be measured (unit: mm)
#define _DIST_ALPHA 0.5

float timeout;  
float dist_min, dist_max, raw_dist, alpha; // unit: mm
float scale;

int a, b; // unit: mm

void setup() {
// initialize GPIO pins
  myservo.attach(PIN_SERVO); 
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);

  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  alpha = _DIST_ALPHA;
  raw_dist = 0.0;
  scale = 0.001 * 0.5 * SND_VEL;
  
// initialize serial port
  Serial.begin(57600);

  a = 80;
  b = 340;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  float distance=dist_cali-230.0;

//  raw_dist = USS_measure(int PIN_IR);
//  raw_dist=float(analogRead(PIN_IR));
//  dist_cali=alpha * raw_dist+((1-alpha) * dist_cali);
  
  if(dist_cali>230.0){
    myservo.write(85-0.2*distance); //lowest=45
  }
  else{
  myservo.write(85-0.3*distance); //highest=125
  }
  
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);
  if(raw_dist > 156 && raw_dist <224) digitalWrite(PIN_LED, 0);
  else digitalWrite(PIN_LED, 255);
  delay(20);
}

//float USS_measure(int PIN_IR) //이동 평균 필터를 적용하고자 하는데 초음파 센서와 적외선 센서의 매개변수 값 자체가 달라서 이해하려고 노력중입니다.
//{
//  float reading;
//  digitalWrite(PIN_IR, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(PIN_IR, LOW);
//  reading = pulseIn(PIN_IR, HIGH, timeout) * scale; // unit: mm
//  if(reading < dist_min || reading > dist_max) reading = 0.0; // return 0 when out of range.
//  return reading;
//}
