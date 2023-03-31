#include <WiFiNINA.h>
#include <utility/wifi_drv.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  200 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  400 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

//#define INI

// our servo # counter
uint8_t servonum = 0;

void setup() {
  WiFiDrv::pinMode(25, OUTPUT); //define green pin
  WiFiDrv::pinMode(26, OUTPUT); //define red pin
  WiFiDrv::pinMode(27, OUTPUT); //define blue pin


  Serial.begin(9600);

  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

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
void loop() {
  //*
  pwm.setPWM(servonum, 0, SERVOMIN);
  delay(1000);
  pwm.setPWM(servonum, 0, SERVOMAX);
  delay(1000);
  servonum++;
  if(servonum>2) servonum=0;
  //*/
  /*
  sit();
  delay(3000);
  stand();
  delay(3000);
  /*
  Serial.println(servonum);
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    pwm.setPWM(servonum, 0, pulselen);
  }
  delay(500);
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(servonum, 0, pulselen);
  }

  delay(500);
  //*/
}

void initLegs(){
  pwm.setPWM(0, 0, SERVOMAX);
  pwm.setPWM(1, 0, SERVOMAX);
  pwm.setPWM(2, 0, SERVOMAX);
}