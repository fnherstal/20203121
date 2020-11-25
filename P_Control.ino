#include <Servo.h>
#define PIN_SERVO 10 // [3110] 10번핀 서보 연결
#define PIN_IR A0 //[3104] 적외선 거리센서 PIN - Analog0 정의 

// Framework setting
#define _DIST_TARGET 230 //[3104] 탁구공을 위치 시킬 목표 
#define _DIST_MIN 100 //[3117] 거리 최소값 //[3104] 측정 거리 최소치 
#define _DIST_MAX 430 //[3117] 거리 최대값//[3104] 측정 거리 최대치
#define PIN_LED 9
// Distance sensor
#define _DIST_ALPHA 0.5  //[3099] EMA 필터링을 위한 alpha 값
               // [3108] 0~1 사이의 값

// Servo range
#define _DUTY_MIN 1050     //[3100] 최저 서보 위치
#define _DUTY_NEU 1450     //[3100] 중립 서보 위치
#define _DUTY_MAX 1850     //[3100] 최대 서보 위치


// Servo speed control
#define _SERVO_ANGLE 30.0 // [3114] 서보각도
#define _SERVO_SPEED 120.0 //[3104] 서보속도

// Event periods
#define _INTERVAL_DIST 20 //[3099] 각 event 사이에 지정한 시간 간격
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100 

// PID parameters
#define _KP 2 // [3103] KP값은 개인이 설정

// [3108] dist 100, 400mm 일때 값, 각자 a,b로 수정
#define a 80
#define b 340
//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;

// Distance sensor

float dist_target;
float dist_raw, dist_ema; 

unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 

bool event_dist, event_servo, event_serial; 

int duty_chg_per_interval; // [3116] 주기 당 서보 duty값 변화량
int duty_target, duty_curr; //[1928] 목표 위치와 현재 위치
// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;


void setup() {
// initialize GPIO pins for LED and attach servo 
myservo.attach(PIN_SERVO); // attach servo
pinMode(PIN_LED,OUTPUT); // initialize GPIO pins

// initialize global variables

// move servo to neutral position
myservo.writeMicroseconds(_DUTY_NEU);
duty_curr = _DUTY_NEU;

// initialize serial port
Serial.begin(57600);

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / 180) * ((float)_INTERVAL_SERVO / 1000);
//    duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED/180) * ((float)_INTERVAL_SERVO / 1000.0);
}

float ir_distance(void){ // return value unit: mm
  float value;
  float volt = float(analogRead(PIN_IR));
  value = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return 300.0 / (b - a) * (value - a) + 100;
}

void loop() {
 if (millis() >= last_sampling_time_dist + _INTERVAL_DIST) event_dist = true;
  if (millis() >= last_sampling_time_servo + _INTERVAL_SERVO) event_servo = true;
  if (millis() >= last_sampling_time_serial + _INTERVAL_SERIAL) event_serial = true;


unsigned long time_curr = millis();
if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST){
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
}

if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO ){
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
}

if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL ){
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
}

if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO ){
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
}

if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL ){
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
      // [3099] dist_ema?

  // PID control logic
    error_curr = _DIST_TARGET - dist_ema;
    pterm = error_curr;
  // [3099]
    iterm = 0;
    dterm = 0;
    control = _KP * pterm + iterm + dterm;


  //duty_target = f(duty_neutral, control);
   //duty_target = ((control>0)?(_DUTY_MAX - _DUTY_NEU)*_SERVO_ANGLE / 180.0:(_DUTY_NEU - _DUTY_MIN) * _SERVO_ANGLE / 180.0) * control;
   //[3099] 비례이득의 비대칭 해결가능
    duty_target = _DUTY_NEU + control; // * ((control>0)?(_DUTY_MAX - _DUTY_NEU):(_DUTY_NEU - _DUTY_MIN));

  // duty_neutral?


  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if(duty_target > _DUTY_MAX){
      duty_target = _DUTY_MAX;
    }
    else if(duty_target < _DUTY_MIN){
      duty_target = _DUTY_MIN;
    }
    // [3099]
    /* duty_target = min(duty_target, _DUTY_MAX);
    duty_target = max(duty_target, _DUTY_MIN);
    // [3111] 범위 제한의 또다른 방법 */
    last_sampling_time_dist = millis();
  }
  
if(event_servo) {
    event_servo=false;
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target>duty_curr) {
  duty_curr += duty_chg_per_interval;
  if(duty_curr > duty_target) duty_curr = duty_target;
     }
    else {
  duty_curr -= duty_chg_per_interval;
  if(duty_curr < duty_target) duty_curr = duty_target;
    }
    // update servo position
     myservo.writeMicroseconds(duty_curr);
     last_sampling_time_servo = millis();
  }   


  if(event_serial) {
    event_serial = false;
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
    last_sampling_time_serial = millis();
  }
  
}

float ir_distance_filtered(void){ // return value unit: mm
  dist_raw = ir_distance();    

  return _DIST_ALPHA * dist_raw + (1 - _DIST_ALPHA) * dist_ema;
} 
