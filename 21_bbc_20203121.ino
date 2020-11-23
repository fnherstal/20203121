#include <Servo.h>
Servo myservo;
// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

int a, b; // unit: mm

void setup() {
// initialize GPIO pins
  myservo.attach(PIN_SERVO); 
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);

  
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
  if(dist_cali>230.0){
    myservo.write(80);
  }
  else{
  myservo.write(110);
  }
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);
  if(raw_dist > 156 && raw_dist <224) digitalWrite(PIN_LED, 0);
  else digitalWrite(PIN_LED, 255);
  delay(20);
}
