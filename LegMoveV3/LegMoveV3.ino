#include <WiFiNINA.h>
#include <utility/wifi_drv.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define SERVOMIN  175 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  350 // This is the 'maximum' pulse length count (out of 4096)
#define LegMin 200
#define LegMid 250
#define LegMax 300
#define HipFrontReach 100
#define HipMin 175 //forwards
#define HipMax 350 //backwards
#define HipMid 295

#define FrontLeftLeg 0
#define FrontLeftHip 8
#define RearLeftLeg 1
#define RearLeftHip 11
#define FrontRightLeg 2
#define FrontRightHip 9
#define RearRightLeg 3
#define RearRightHip 10


int lastPWML=0;
int lastPWMH=0;

#define INI

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
  ledYellow();
  
  initLegs();
  
  
  delay(2000);
  ledGreen();

  delay(1000);
  ledOff();
  #endif
}
void centerHips(){
  if(lastPWMH<HipMid){
    for (uint16_t pulselen = lastPWMH; pulselen < HipMid; pulselen++) {
        pwm.setPWM(FrontLeftHip, 0, pulselen); delay(5);
        pwm.setPWM(FrontRightHip, 0, pulselen); delay(5);
        pwm.setPWM(RearLeftHip, 0, pulselen); delay(5);
        pwm.setPWM(RearRightHip, 0, pulselen); delay(5);
      }
  }
  else if(lastPWMH>HipMid){
    for (uint16_t pulselen = lastPWMH; pulselen > HipMid; pulselen--) {
        pwm.setPWM(FrontLeftHip, 0, pulselen); delay(5);
        pwm.setPWM(FrontRightHip, 0, pulselen); delay(5);
        pwm.setPWM(RearLeftHip, 0, pulselen); delay(5);
        pwm.setPWM(RearRightHip, 0, pulselen); delay(5);
      }
  }
  lastPWMH=HipMid;
}
void midLegs(){
  if(lastPWML<LegMid){
    for (uint16_t pulselen = lastPWML; pulselen < LegMid; pulselen++) {
        pwm.setPWM(FrontLeftLeg, 0, pulselen); delay(5);
        pwm.setPWM(FrontRightLeg, 0, pulselen); delay(5);
        pwm.setPWM(RearLeftLeg, 0, pulselen); delay(5);
        pwm.setPWM(RearRightLeg, 0, pulselen); delay(5);
      }
  }
  else if(lastPWML>LegMid){
    for (uint16_t pulselen = lastPWML; pulselen > LegMid; pulselen--) {
        pwm.setPWM(FrontLeftLeg, 0, pulselen); delay(5);
        pwm.setPWM(FrontRightLeg, 0, pulselen); delay(5);
        pwm.setPWM(RearLeftLeg, 0, pulselen); delay(5);
        pwm.setPWM(RearRightLeg, 0, pulselen); delay(5);
      }
  }
  lastPWML=LegMid;
}
void stand(){
  //center hips to stand
  if(lastPWMH!=HipMid){
    centerHips();
    delay(100); 
  }

  for (uint16_t pulselen = lastPWML; pulselen < LegMax; pulselen++) {
    pwm.setPWM(FrontRightLeg, 0, pulselen); delay(5);
    pwm.setPWM(FrontLeftLeg, 0, pulselen); delay(5);
    pwm.setPWM(RearRightLeg, 0, pulselen); delay(5);
    pwm.setPWM(RearLeftLeg, 0, pulselen); delay(5);
  }
  lastPWML=LegMax;

}
void sit(){
  //center hips to sit
  if(lastPWMH!=HipMid){
    centerHips();
    delay(100); 
  }
   
  for (uint16_t pulselen = lastPWML; pulselen < LegMin; pulselen++) {
    pwm.setPWM(FrontRightLeg, 0, pulselen); delay(5);
    pwm.setPWM(FrontLeftLeg, 0, pulselen); delay(5);
    pwm.setPWM(RearRightLeg, 0, pulselen); delay(5);
    pwm.setPWM(RearLeftLeg, 0, pulselen); delay(5);
  }
  lastPWML=LegMin;
}
void initLegs(){
  //*
  lastPWML=LegMid;
  lastPWMH=HipMid;

  pwm.setPWM(FrontLeftLeg, 0, LegMax); delay(5);
  pwm.setPWM(FrontLeftHip, 0, HipMid); delay(5);

  pwm.setPWM(FrontLeftLeg, 0, LegMid); delay(5);
  pwm.setPWM(FrontLeftHip, 0, HipMid); delay(5);

  pwm.setPWM(RearRightLeg, 0, LegMid); delay(5);
  pwm.setPWM(RearRightHip, 0, HipMid); delay(5);

  pwm.setPWM(RearLeftLeg, 0, LegMid); delay(5);
  pwm.setPWM(RearLeftHip, 0, HipMid); delay(5);
  //*/
}

void walkCycleSide(int FrontLeg, int FrontHip, int RearLeg, int RearHip){
  //To walk you must first...
  
  int temp=lastPWML;

  //retract FL and RR legs for movement
  for (uint16_t pulselen = LegMid; pulselen < LegMin; pulselen+=4) {
    pwm.setPWM(FrontLeg, 0, pulselen); delay(5);
    pwm.setPWM(RearLeg, 0, pulselen); delay(5);
  }
  //should now be "standing" on Front Right and Rear Left legs 
  
  //bring the front left and rear right hip joints forwards to prepare for movement
  for (uint16_t pulselen = HipMid; pulselen < HipMin; pulselen+=4) {
    pwm.setPWM(FrontHip, 0, pulselen); delay(5);
    pwm.setPWM(RearHip, 0, pulselen); delay(5);
  }

  //turn the front left and rear right hip joints backwards
  //while also extending the legs ot make ground contact
  uint16_t pulseLeg=LegMin;
  for (uint16_t pulselen = HipMin; pulselen < HipMax; pulselen+=4) {
    pwm.setPWM(FrontLeg, 0, pulseLeg); delay(5);
    pwm.setPWM(RearLeg, 0, pulseLeg); delay(5);
    pwm.setPWM(FrontHip, 0, pulselen); delay(5);
    pwm.setPWM(RearHip, 0, pulselen); delay(5);
    pulseLeg+=2;
    temp=pulseLeg;
  }
  //initially pulseLeg+=.57 to equate for the difference of
  //moving simultaneously 175 units and 100 units, but changed to
  //0.44 to prevent FULL extension of the leg and only approx 75% extension

  //retract legs
  for (uint16_t pulselen = temp; pulselen > LegMin; pulselen-=4) {
    pwm.setPWM(FrontLeg, 0, pulselen); delay(5);
    pwm.setPWM(RearLeg, 0, pulselen); delay(5);
  }

  //reset hips to Middle
  for (uint16_t pulselen = HipMax; pulselen > HipMid; pulselen-=4) {
    pwm.setPWM(FrontHip, 0, pulselen); delay(5);
    pwm.setPWM(RearHip, 0, pulselen); delay(5);
  }

  //extend legs
  for (uint16_t pulselen=LegMin; pulselen < LegMid; pulselen++) {
    pwm.setPWM(FrontLeg, 0, pulselen); delay(5);
    pwm.setPWM(RearLeg, 0, pulselen); delay(5);
  }
  
}
void walkForwards(){
  //move one side then the other side
  walkCycleSide(FrontLeftLeg, FrontLeftHip, RearRightLeg, RearRightHip);
  walkCycleSide(FrontRightLeg, FrontRightHip, RearLeftLeg, RearLeftHip);
  //thus completing a "walk cycle"
}
void beginMoving(int direction){
  //when you want to start moving...
  //make sure hips are centered
  if(lastPWMH!=HipMid){
    centerHips();
    delay(100); 
  }
  //and legs are at Mid point
  if(lastPWML!=LegMid){
    midLegs();
    delay(100);
  }

  //determine direction
  if(direction==0){
    walkForwards();    
  }

  else{
    //otherwise display unexpected input (should never happen)
    Serial.println("unexpected input in -- BeginMoving");
    delay(1000);    
  }
}

void loop() {
  

  //initLegs();

  //Serial.println("loop");

  //First take input from the controller
  
  //if input of right controller is "forwards"
  //perform one forwards walk cycle

  //if right controller is "backwards"
  //perform one backwards walk cycle

  //if left controller is "forwards"
  //sit down

  //if left controller if "backwards"
  //stand up
  

  
  
  /*
  //Testing Shit
  Serial.println(servonum);
  for (uint16_t pulselen = LegMin; pulselen < LegMax; pulselen++) {
    pwm.setPWM(servonum, 0, pulselen);
  }
  delay(2000);
  for (uint16_t pulselen = LegMax; pulselen > LegMin; pulselen--) {
    pwm.setPWM(servonum, 0, pulselen);
  }

  delay(1000);
  servonum++;
  if(servonum>1){servonum=0;}
  
  //*/
}
