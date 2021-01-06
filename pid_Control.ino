#include <Servo.h>
Servo myservo;
// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

unsigned long last_sampling_time_dist, last_sampling_time_serial; // unit: ms
bool event_dist, event_serial;
float raw_dist;
//===================================================
// ⚠⚠❗❗⚠⚠ 코드를 작동시키기 전에 _DUTY_NEU의 값을 각자의 중립위치각도로 수정 후 사용!!!
#define _DUTY_NEU 900 // neutral position
//===================================================
#define _INTERVAL_DIST 30   // USS interval (unit: ms)
#define _INTERVAL_SERIAL 100 // serial interval (unit: ms)


#define DELAY_MICROS  1500
#define EMA_ALPHA 0.35
float filtered_dist;
float ema_dist = 0;
float samples_num = 3;

float servoAngle = _DUTY_NEU;
float setPoint = 210.0;
float dist_target = setPoint;
float duty_curr = 0;
float error;

float duty_target = 900;

  float pTerm = 0;
  float iTerm = 0;
  float dTerm = 0;
  float total_err = 0;

      float Kp = 0.9;
    float Ki = 0.01;
    float Kd = 45.0;

   float  p_error = 0.0; 
   float i_error = 0.0;  
   float d_error = 0.0; 
   int step_num;
   float p[3];
   int numSteps;
   int p_index;
   int p_add;
   int p_sub;
 float best_err;
   
float ir_distance(void) { // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0 / (volt - 9.0)) - 4.0) * 10.0;
  return val;
}

// ================
float under_noise_filter(void) {
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) {
      largestReading = currReading;
    }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}
float filtered_ir_distance(void) {
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) {
      lowestReading = currReading;
    }
  }
  // eam 필터 추가
  ema_dist = EMA_ALPHA * lowestReading + (1 - EMA_ALPHA) * ema_dist;
  return ema_dist;
}
//===================================================

void UpdateError(float cte) {
   if(step_num == 1) {
     p_error = cte;
   }

   d_error = cte - p_error;
   p_error = cte;
   i_error += cte;

  total_err += cte*cte;

  if(step_num % numSteps == 0) {
    if(total_err < best_err) {
      best_err = total_err;
      p[p_index] *= 1.1;

      p_add = p_sub = 0;
    }

    if(!p_add && !p_sub) {
      Twiddler(p_index, p[p_index]);
      p_add = 1;
    } else if(p_add && !p_sub) {
      Twiddler(p_index, -2*p[p_index]);
      p_sub = 1;
    } else {
      Twiddler(p_index, p[p_index]);
      p[p_index] *= 0.9;
      p_add = p_sub = 0;

      p_index = (p_index + 1) % 3;
    }
    total_err = 0;
  }

   step_num++;
}

void Twiddler(int index, float value) {
  switch(index) {
    case 0:
      Kp += value;
      break;
    case 1:
      Ki += value;
      break;
    case 2:
      Kd += value;
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(57600);
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);
  delay(1000);
  // initialize last sampling time
  last_sampling_time_dist = last_sampling_time_serial = 0;
  event_dist = event_serial = false;
 pTerm = 0;
 iTerm = 0;
  dTerm = 0;

   p_error = 0.0;   
   i_error = 0.0;   
   d_error = 0.0;   

   step_num = 1;
   best_err = 10000000000;

   numSteps = 200;

   p[0] = Kp*0.1;
   p[1] = Ki*0.1;
   p[2] = Kd*0.1;

   p_index = 0;
}

void loop() {
  unsigned long time_curr = millis();

  if (time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }
  if (time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }
  if (event_dist) {
    event_dist = false;
    raw_dist = ir_distance();
    //==============================================
    filtered_dist = filtered_ir_distance();
    //===============================================

    //before_err = error; 
    error = setPoint - filtered_dist;
    

    /*
    pTerm = Kp * error;
    totalError += error;
    iTerm += Ki * error * 0.05;
    dTerm = Kd * (error - beforeError);
    */
    
    UpdateError(error);
    pTerm = Kp * p_error;
    iTerm = Kd * d_error;
    dTerm = Ki * i_error;
    servoAngle = pTerm + iTerm + dTerm;
    
    duty_curr = servoAngle + 900;
    myservo.writeMicroseconds(servoAngle + 900);
  }
  if (event_serial) {
    event_serial = false;
Serial.print("IR:");
 Serial.print(filtered_dist);
 Serial.print(",T:");
 Serial.print(dist_target);
 Serial.print(",P:");
 Serial.print(map(pTerm,-1000,1000,510,610));
 Serial.print(",D:");
 Serial.print(map(dTerm,-1000,1000,510,610));
 Serial.print(",I:");
 Serial.print(map(iTerm,-1000,1000,510,610));
  Serial.print(",IOrig:");
 Serial.print(iTerm);
 Serial.print(",DTT:");
 Serial.print(map(duty_target,1000,2000,410,510));
 Serial.print(",DTC:");
 Serial.print(map(duty_curr,1000,2000,410,510));
 Serial.println(",-G:245,+G:265,m:0,M:800");

  }
}
