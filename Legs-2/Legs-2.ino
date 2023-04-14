#include <WiFiNINA.h>
#include <utility/wifi_drv.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define SERVOMIN  175 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  350 // This is the 'maximum' pulse length count (out of 4096)
#define LegMin 200
#define LegMax 300
#define HipFrontReach 100
#define HipMin 175
#define HipMax 350
#define HipMid 295
//#define INI

// our servo # counter
uint8_t servonum = 0;

void setup() {
  WiFiDrv::pinMode(25, OUTPUT); //define green pin
  WiFiDrv::pinMode(26, OUTPUT); //define red pin
  WiFiDrv::pinMode(27, OUTPUT); //define blue pin


  Serial.begin(9600);

  pwm.begin();

  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates


  //initLegs();

  #ifdef INI
  
  initLegs();
  
  

  
  WiFiDrv::analogWrite(26, 255);
  WiFiDrv::analogWrite(25, 255);
  WiFiDrv::analogWrite(27, 0);
  
  delay(2000);
  WiFiDrv::analogWrite(26, 0);
  WiFiDrv::analogWrite(25, 255);
  WiFiDrv::analogWrite(27, 0);

  delay(1000);
  WiFiDrv::analogWrite(25, 0);
  #endif
}

void stand(){
  WiFiDrv::analogWrite(26, 0);
  WiFiDrv::analogWrite(25, 255);
  WiFiDrv::analogWrite(27, 255);
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    pwm.setPWM(0, 0, pulselen);
    pwm.setPWM(1, 0, pulselen);
    pwm.setPWM(2, 0, pulselen);
  }
}
void sit(){
  WiFiDrv::analogWrite(26, 255);
  WiFiDrv::analogWrite(25, 0);
  WiFiDrv::analogWrite(27, 255);
  for (uint16_t pulselen = SERVOMAX; pulselen < SERVOMIN; pulselen++) {
    pwm.setPWM(0, 0, pulselen);
    pwm.setPWM(1, 0, pulselen);
    pwm.setPWM(2, 0, pulselen);
  }
}
void initLegs(){
  /*
  //front left
  pwm.setPWM(0, 0, LegMin);
  pwm.setPWM(8, 0, HipMid);
  //back left
  pwm.setPWM(1, 0, LegMin);
  pwm.setPWM(11, 0, HipMid);
  
  //pwm.setPWM(2, 0, LegMin);
  pwm.setPWM(9, 0, HipMid);
  pwm.setPWM(10, 0, HipMid);
  //*/
}
void loop() {
  
  Serial.println("loop");
  Serial.println(servonum);
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    pwm.setPWM(servonum, 0, pulselen);
  }
  delay(2000);
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(servonum, 0, pulselen);
  }

  delay(500);
  //*/
}

