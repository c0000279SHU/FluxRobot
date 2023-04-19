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
  //leg pins
#define FrontLeftLeg 0
#define FrontLeftHip 8
#define RearLeftLeg 1
#define RearLeftHip 11
#define FrontRightLeg 3
#define FrontRightHip 9
#define RearRightLeg 2
#define RearRightHip 10
int lastPWML=0;
int lastPWMH=0;
//Servo Controller
//------------------------------------
//GPS
#include <Adafruit_GPS.h>
// Connect to the GPS on the hardware I2C port
Adafruit_GPS GPS(&Wire);
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
uint32_t timer = millis();
//GPS
//------------------------------------
//MLX
#include <Adafruit_MLX90640.h>
Adafruit_MLX90640 mlx;
float frame[32*24]; // buffer for full frame of temperatures
//low range of the sensor (this will be blue on the screen)
#define MINTEMP 20
//high range of the sensor (this will be red on the screen)
#define MAXTEMP 35
const uint16_t camColors[] = {0x480F,
0x400F,0x400F,0x400F,0x4010,0x3810,0x3810,0x3810,0x3810,0x3010,0x3010,
0x3010,0x2810,0x2810,0x2810,0x2810,0x2010,0x2010,0x2010,0x1810,0x1810,
0x1811,0x1811,0x1011,0x1011,0x1011,0x0811,0x0811,0x0811,0x0011,0x0011,
0x0011,0x0011,0x0011,0x0031,0x0031,0x0051,0x0072,0x0072,0x0092,0x00B2,
0x00B2,0x00D2,0x00F2,0x00F2,0x0112,0x0132,0x0152,0x0152,0x0172,0x0192,
0x0192,0x01B2,0x01D2,0x01F3,0x01F3,0x0213,0x0233,0x0253,0x0253,0x0273,
0x0293,0x02B3,0x02D3,0x02D3,0x02F3,0x0313,0x0333,0x0333,0x0353,0x0373,
0x0394,0x03B4,0x03D4,0x03D4,0x03F4,0x0414,0x0434,0x0454,0x0474,0x0474,
0x0494,0x04B4,0x04D4,0x04F4,0x0514,0x0534,0x0534,0x0554,0x0554,0x0574,
0x0574,0x0573,0x0573,0x0573,0x0572,0x0572,0x0572,0x0571,0x0591,0x0591,
0x0590,0x0590,0x058F,0x058F,0x058F,0x058E,0x05AE,0x05AE,0x05AD,0x05AD,
0x05AD,0x05AC,0x05AC,0x05AB,0x05CB,0x05CB,0x05CA,0x05CA,0x05CA,0x05C9,
0x05C9,0x05C8,0x05E8,0x05E8,0x05E7,0x05E7,0x05E6,0x05E6,0x05E6,0x05E5,
0x05E5,0x0604,0x0604,0x0604,0x0603,0x0603,0x0602,0x0602,0x0601,0x0621,
0x0621,0x0620,0x0620,0x0620,0x0620,0x0E20,0x0E20,0x0E40,0x1640,0x1640,
0x1E40,0x1E40,0x2640,0x2640,0x2E40,0x2E60,0x3660,0x3660,0x3E60,0x3E60,
0x3E60,0x4660,0x4660,0x4E60,0x4E80,0x5680,0x5680,0x5E80,0x5E80,0x6680,
0x6680,0x6E80,0x6EA0,0x76A0,0x76A0,0x7EA0,0x7EA0,0x86A0,0x86A0,0x8EA0,
0x8EC0,0x96C0,0x96C0,0x9EC0,0x9EC0,0xA6C0,0xAEC0,0xAEC0,0xB6E0,0xB6E0,
0xBEE0,0xBEE0,0xC6E0,0xC6E0,0xCEE0,0xCEE0,0xD6E0,0xD700,0xDF00,0xDEE0,
0xDEC0,0xDEA0,0xDE80,0xDE80,0xE660,0xE640,0xE620,0xE600,0xE5E0,0xE5C0,
0xE5A0,0xE580,0xE560,0xE540,0xE520,0xE500,0xE4E0,0xE4C0,0xE4A0,0xE480,
0xE460,0xEC40,0xEC20,0xEC00,0xEBE0,0xEBC0,0xEBA0,0xEB80,0xEB60,0xEB40,
0xEB20,0xEB00,0xEAE0,0xEAC0,0xEAA0,0xEA80,0xEA60,0xEA40,0xF220,0xF200,
0xF1E0,0xF1C0,0xF1A0,0xF180,0xF160,0xF140,0xF100,0xF0E0,0xF0C0,0xF0A0,
0xF080,0xF060,0xF040,0xF020,0xF800,};
uint16_t displayPixelWidth, displayPixelHeight;
//MLX
//--------------------------------
//DISPLAY
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#define TFT_CS         3
#define TFT_RST        2 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC         1
//Connect MOSI and SCLK respectively on MK1010
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
float p = 3.1415926;
bool displayRotated=false;
#define Black ST77XX_BLACK
int outputCycles=0;
//Display
//-----------------------------
//Controller Variables
#define RJoyXpin A2
#define RJoyYpin A1
#define LJoyXpin A5
#define LJoyYpin A6
int RJoyX,RJoyY,LJoyX,LJoyY;
#define BtnPin 0

//sets the positive and negative deadzones of the controller
#define JoystickHigh 400
#define JoystickLow 400
#define Deadzone 200
bool inDeadRX, inDeadRY, inDeadLX, inDeadLY;
//short for: In Deadzone Right Joystick X (etc..)

//SR04 vars
#define trigPin 9
#define echoPin 10
float duration, distance;

//-----------
//Other or Helper Variables
bool ThermalOutput=false;
int screenRotation=0;
int mode=0;

void setup() {
  //simple setup stuff
  Serial.begin(9600);
  pinMode(BtnPin, INPUT);
  WiFiDrv::pinMode(25, OUTPUT); //define green pin
  WiFiDrv::pinMode(26, OUTPUT); //define red pin
  WiFiDrv::pinMode(27, OUTPUT); //define blue pin

  ledYellow();
  //Yellow LED signles INSIDE setup

//-------GPS SETUP---------
  GPS.begin(0x10);  // The I2C address to use is 0x10
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  // Ask for firmware version
  GPS.println(PMTK_Q_RELEASE);

//--------DISPLAY SETUP------
  Serial.print(F("Hello! ST77xx TFT Test"));

  // OR use this initializer (uncomment) if using a 1.9" 170x320 TFT:
  tft.init(170, 320);           // Init ST7789 170x320

  // SPI speed defaults to SPI_DEFAULT_FREQ defined in the library, you can override it here
  // Note that speed allowable depends on chip and quality of wiring, if you go too fast, you
  // may end up with a black screen some times, or all the time.
  //tft.setSPISpeed(40000000);
  Serial.println(F("Initialized"));

  uint16_t time = millis();
  tft.fillScreen(ST77XX_BLACK);
  time = millis() - time;

  Serial.println(time, DEC);
  if(displayRotated){tft.setRotation(3);}
  screenLogo();
  displayPixelWidth = tft.width() / 32;
  displayPixelHeight = tft.width() / 32; //Keep pixels square 
  //---------DISPLAY SETUP END------------------

  //-----Servo Controller SETUP -----
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  initLegs();

  tft.print(" .");
  //-----Servo Controller SETUP END------

  //------MLX CAM SETUP-----------
  //*/
  //while (!Serial) delay(10);
  //Serial.begin(115200);
  //delay(100);

  Serial.println("Adafruit MLX90640 Simple Test");
  if (! mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("MLX90640 not found!");
    ledRed();
    while (1) delay(10);
  }
  Serial.println("Found Adafruit MLX90640");

  Serial.print("Serial number: ");
  Serial.print(mlx.serialNumber[0], HEX);
  Serial.print(mlx.serialNumber[1], HEX);
  Serial.println(mlx.serialNumber[2], HEX);
  
  //mlx.setMode(MLX90640_INTERLEAVED);
  mlx.setMode(MLX90640_CHESS);
  Serial.print("Current mode: ");
  if (mlx.getMode() == MLX90640_CHESS) {
    Serial.println("Chess");
  } else {
    Serial.println("Interleave");    
  }

  mlx.setResolution(MLX90640_ADC_18BIT);
  Serial.print("Current resolution: ");
  mlx90640_resolution_t res = mlx.getResolution();
  switch (res) {
    case MLX90640_ADC_16BIT: Serial.println("16 bit"); break;
    case MLX90640_ADC_17BIT: Serial.println("17 bit"); break;
    case MLX90640_ADC_18BIT: Serial.println("18 bit"); break;
    case MLX90640_ADC_19BIT: Serial.println("19 bit"); break;
  }

  mlx.setRefreshRate(MLX90640_2_HZ);
  Serial.print("Current frame rate: ");
  mlx90640_refreshrate_t rate = mlx.getRefreshRate();
  switch (rate) {
    case MLX90640_0_5_HZ: Serial.println("0.5 Hz"); break;
    case MLX90640_1_HZ: Serial.println("1 Hz"); break; 
    case MLX90640_2_HZ: Serial.println("2 Hz"); break;
    case MLX90640_4_HZ: Serial.println("4 Hz"); break;
    case MLX90640_8_HZ: Serial.println("8 Hz"); break;
    case MLX90640_16_HZ: Serial.println("16 Hz"); break;
    case MLX90640_32_HZ: Serial.println("32 Hz"); break;
    case MLX90640_64_HZ: Serial.println("64 Hz"); break;
  }
  tft.print(" .");
  //*/
  //------MLX CAM SETUP END----------------

  ledGreen();
  delay(500);
  ledOff();
  delay(500);
  ledGreen();
  delay(500);
  ledOff();
  delay(500);
  ledGreen();
  delay(1000);
  tft.fillScreen(ST77XX_BLACK);
  ledOff();
}
//-----DISPLAY FUNCTIONS-------
void screenLogo(){
  uint16_t color1 = ST77XX_BLUE;
  uint16_t color2 = ST77XX_RED;
  for (int16_t x=0; x < tft.width(); x+=6) {
    tft.drawLine(0, 0, x, tft.height()-1, color1);
    delay(0);
    tft.drawLine(tft.width()-1, 0, x, tft.height()-1, color2);
    delay(0);
  }
  for (int16_t y=0; y < tft.height(); y+=6) {
    tft.drawLine(0, 0, tft.width()-1, y, color1);
    delay(0);
    tft.drawLine(tft.width()-1, 0, 0, y, color2);
    delay(0);
  }
  
  tft.setTextWrap(false);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(3);
  if(displayRotated){
    tft.setCursor((tft.height()/2)-10, (tft.width()/2)-75);
    tft.println("AEM Tech");
  }
  else if(!displayRotated){
    tft.setCursor((tft.width()/2)-20, (tft.height()/2)-75);
    tft.println("AEM");
    tft.setCursor((tft.width()/2)-30, (tft.height()/2)-45);
    tft.println("Tech");
  }
}

void dataOutTFT(){
  tft.fillScreen(Black);
  tft.setCursor(0, 0);
  tft.setTextSize(2);
  tft.print("RJoyX: ");
  tft.println(RJoyX);
  
  tft.print("RJoyY: ");
  tft.println(RJoyY);

  tft.print("LJoyX: ");
  tft.println(LJoyX);

  tft.print("KJoyY: ");
  tft.println(LJoyY);

  tft.println("--GPS--");
  tft.println("Location: ");
  tft.print(GPS.latitude, 4); tft.print(GPS.lat);
  tft.println(", ");
  tft.print(GPS.longitude, 4); tft.println(GPS.lon);
}

//-----DISPLAY FUNCTIONS END-------

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------START LOOP-------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


void loop() {
  //Change Mode if the button is pressed
  if(digitalRead(BtnPin)==1){
    if(!displayRotated){
      if(mode!=0){
        if(screenRotation==3){screenRotation=0;}
        else if(screenRotation==0){screenRotation=3;}
      }else {screenRotation=0;} //ALWAYS make sure the screen is Top-Down when returning to Data Output mode
      tft.setRotation(screenRotation);
      displayPixelWidth = tft.width() / 32;
      displayPixelHeight = tft.width() / 32;
    }
    //mode 0 and 2 change back and forth from thermal output
    if(mode!=1){
      ThermalOutput=!ThermalOutput;
    }
    tft.fillScreen(ST77XX_BLACK);
    mode++;
    if(mode>2){mode=0;}
    delay(500);
    //add a delay to allow for depress time
  }


  //For controlling Flux...

  //First take input from the controller
  RJoyX=map(analogRead(RJoyXpin), 0, 1025, JoystickLow, -JoystickHigh);
  RJoyY=map(analogRead(RJoyYpin), 0, 1025, JoystickLow, -JoystickHigh);
  LJoyX=map(analogRead(LJoyXpin), 0, 1025, JoystickLow, -JoystickHigh);
  LJoyY=map(analogRead(LJoyYpin), 0, 1025, JoystickLow, -JoystickHigh);
  //This maps the Joystick values to easy to work with values
  if(RJoyX > -Deadzone && RJoyX < Deadzone){inDeadRX=true;} else {inDeadRX=false;}
  if(RJoyY > -Deadzone && RJoyY < Deadzone){inDeadRY=true;} else {inDeadRY=false;}
  if(LJoyX > -Deadzone && LJoyX < Deadzone){inDeadLX=true;} else {inDeadLX=false;}
  if(LJoyY > -Deadzone && LJoyY < Deadzone){inDeadLY=true;} else {inDeadLY=false;}
  //This determines if a joystick is in the "deadzone" AKA center
  

  //if input of right controller is "forwards"
  //perform one forwards walk cycle
  if(!inDeadRX && RJoyX > Deadzone){beginMoving(0);}

  //if right controller is "backwards"
  //perform one backwards walk cycle
  if(!inDeadRX && RJoyX < -Deadzone){beginMoving(1);}

  //if left controller is "forwards"
  //sit down
  else if(!inDeadLX && LJoyX > Deadzone){sit();}

  //if left controller if "backwards"
  //stand up
  else if(!inDeadLX && LJoyX < -Deadzone){stand();}

  //if no controllers are moving
  //go back to neutral posotion
  else{
    centerHips();
    midLegs();
  }

  if(!ThermalOutput){
    doGPS();
  }

  //*
  Serial.print("RJoyX: ");
  Serial.print(RJoyX);
  Serial.print("\t");
  
  Serial.print("RJoyY: ");
  Serial.print(RJoyY);
  Serial.print("\t");

  Serial.print("LJoyX: ");
  Serial.print(LJoyX);
  Serial.print("\t");

  Serial.print("KJoyY: ");
  Serial.print(LJoyY);
  Serial.print("\t");

  Serial.print("Mode: ");
  Serial.println(mode);
  //*/


  //determines how often to refresh the screen
  if(!ThermalOutput && outputCycles%500==0){
    sr04();
    dataOutTFT();
    if(outputCycles>100000){outputCycles=0;} //resets to prevent stackoverflow
  }
  outputCycles++;
  //about every 500 "cycles" it refreshes
  //to prevent stuttery screen viewing



  //handles the output of the thermal camera
  if(ThermalOutput){
    thermalImaging();
  }
  
}//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------END LOOP--------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//SR04 sensor function
void sr04(){
  
  digitalWrite(trigPin, HIGH);
  delay(5);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;
}

//GPS Function
void doGPS(){
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  }
}

//MLX Function
void thermalImaging(){
  //if the MLX is not detected do nothing
  if (mlx.getFrame(frame) != 0) {
    Serial.println("Failed");
    return;
  }
  int colorTemp;
  //need a double for to traverse the MLX cam data array
  for (uint8_t h=0; h<24; h++) {
    for (uint8_t w=0; w<32; w++) {
      float t = frame[h*32 + w];
      
      t = min(t, MAXTEMP);
      t = max(t, MINTEMP); 

      uint8_t colorIndex = map(t, MINTEMP, MAXTEMP, 0, 255);
        
      colorIndex = constrain(colorIndex, 0, 255);
      //draw the pixels!
      tft.fillRect(displayPixelWidth * w, displayPixelHeight * h,
                    displayPixelHeight, displayPixelWidth, 
                    camColors[colorIndex]);
    }
  }
}

//----SERVO FUNCTIONS--------
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
  for (uint16_t pulselen = lastPWML; pulselen > LegMin; pulselen--) {
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

void walkCycleSideForward(int FrontLeg, int FrontHip, int RearLeg, int RearHip, bool isFrontRightSide){
  //isFrontRightSide? will determine how if we flip the Front or Rear's Hip movements (they're inversed)
  uint16_t hip2;
  int temp=lastPWML;
  //To walk you must first...

  //retract FL and RR legs for movement
  for (uint16_t pulselen = LegMid; pulselen < LegMin; pulselen+=4) {
    pwm.setPWM(FrontLeg, 0, pulselen); delay(5);
    pwm.setPWM(RearLeg, 0, pulselen); delay(5);
  }
  //should now be "standing" on Front Right and Rear Left legs 
  
  //bring the front left and rear right hip joints forwards to prepare for movement
  
  for (uint16_t pulselen = HipMid; pulselen < HipMin; pulselen+=4) {
    hip2=map(pulselen, HipMid, HipMin, HipMid, HipMax);
    if(!isFrontRightSide){
      pwm.setPWM(FrontHip, 0, pulselen); delay(5);
      pwm.setPWM(RearHip, 0, hip2); delay(5);
    }
    else{
      pwm.setPWM(FrontHip, 0, hip2); delay(5);
      pwm.setPWM(RearHip, 0, pulselen); delay(5);
    }
  }

  //turn the front left and rear right hip joints backwards
  //while also extending the legs ot make ground contact
  uint16_t pulseLeg=LegMin;
  for (uint16_t pulselen = HipMin; pulselen < HipMax; pulselen+=4) {
    hip2=map(pulselen, HipMin, HipMax, HipMax, HipMin);
    pwm.setPWM(FrontLeg, 0, pulseLeg); delay(5);
    pwm.setPWM(RearLeg, 0, pulseLeg); delay(5);

    if(!isFrontRightSide){
      pwm.setPWM(FrontHip, 0, pulselen); delay(5);
      pwm.setPWM(RearHip, 0, hip2); delay(5);
    }
    else{
      pwm.setPWM(FrontHip, 0, hip2); delay(5);
      pwm.setPWM(RearHip, 0, pulselen); delay(5);
    }
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
    hip2=map(pulselen, HipMid, HipMin, HipMid, HipMax);
    if(!isFrontRightSide){
      pwm.setPWM(FrontHip, 0, pulselen); delay(5);
      pwm.setPWM(RearHip, 0, hip2); delay(5);
    }
    else{
      pwm.setPWM(FrontHip, 0, hip2); delay(5);
      pwm.setPWM(RearHip, 0, pulselen); delay(5);
    }
  }

  //extend legs
  for (uint16_t pulselen=LegMin; pulselen < LegMid; pulselen++) {
    pwm.setPWM(FrontLeg, 0, pulselen); delay(5);
    pwm.setPWM(RearLeg, 0, pulselen); delay(5);
  }
  
}
void walkForwards(){
  //move one side then the other side
  walkCycleSideForward(FrontLeftLeg, FrontLeftHip, RearRightLeg, RearRightHip, false);
  walkCycleSideForward(FrontRightLeg, FrontRightHip, RearLeftLeg, RearLeftHip, true);
  //thus completing a "walk cycle"
}
void walkCycleSideBackwards(int FrontLeg, int FrontHip, int RearLeg, int RearHip, bool isFrontRightSide){
  //isFrontRightSide? will determine how if we flip the Front or Rear's Hip movements (they're inversed)
  uint16_t hip2;
  int temp=lastPWML;
  //To walk you must first...

  //retract FL and RR legs for movement
  for (uint16_t pulselen = LegMid; pulselen < LegMin; pulselen+=4) {
    pwm.setPWM(FrontLeg, 0, pulselen); delay(5);
    pwm.setPWM(RearLeg, 0, pulselen); delay(5);
  }
  //should now be "standing" on Front Right and Rear Left legs 
  
  //bring the front left and rear right hip joints forwards to prepare for movement
  
  for (uint16_t pulselen = HipMid; pulselen < HipMax; pulselen+=4) {
    hip2=map(pulselen, HipMid, HipMin, HipMid, HipMax);
    if(!isFrontRightSide){
      pwm.setPWM(FrontHip, 0, pulselen); delay(5);
      pwm.setPWM(RearHip, 0, hip2); delay(5);
    }
    else{
      pwm.setPWM(FrontHip, 0, hip2); delay(5);
      pwm.setPWM(RearHip, 0, pulselen); delay(5);
    }
  }

  //turn the front left and rear right hip joints backwards
  //while also extending the legs ot make ground contact
  uint16_t pulseLeg=LegMin;
  for (uint16_t pulselen = HipMax; pulselen > HipMin; pulselen-=4) {
    hip2=map(pulselen, HipMin, HipMax, HipMax, HipMin);
    pwm.setPWM(FrontLeg, 0, pulseLeg); delay(5);
    pwm.setPWM(RearLeg, 0, pulseLeg); delay(5);

    if(!isFrontRightSide){
      pwm.setPWM(FrontHip, 0, pulselen); delay(5);
      pwm.setPWM(RearHip, 0, hip2); delay(5);
    }
    else{
      pwm.setPWM(FrontHip, 0, hip2); delay(5);
      pwm.setPWM(RearHip, 0, pulselen); delay(5);
    }
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
  for (uint16_t pulselen = HipMin; pulselen < HipMid; pulselen+=4) {
    hip2=map(pulselen, HipMid, HipMin, HipMid, HipMax);
    if(!isFrontRightSide){
      pwm.setPWM(FrontHip, 0, pulselen); delay(5);
      pwm.setPWM(RearHip, 0, hip2); delay(5);
    }
    else{
      pwm.setPWM(FrontHip, 0, hip2); delay(5);
      pwm.setPWM(RearHip, 0, pulselen); delay(5);
    }
  }

  //extend legs
  for (uint16_t pulselen=LegMin; pulselen < LegMid; pulselen++) {
    pwm.setPWM(FrontLeg, 0, pulselen); delay(5);
    pwm.setPWM(RearLeg, 0, pulselen); delay(5);
  }
  
}

void walkBackwards(){
  //move one side then the other side
  walkCycleSideBackwards(FrontLeftLeg, FrontLeftHip, RearRightLeg, RearRightHip, false);
  walkCycleSideBackwards(FrontRightLeg, FrontRightHip, RearLeftLeg, RearLeftHip, true);
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
  else if(direction==1){
    walkBackwards();
  }
  else{
    //otherwise display unexpected input (should never happen)
    Serial.println("unexpected input in -- BeginMoving");
    tft.fillScreen(Black);
    tft.setCursor(10, 10);
    tft.println("unexpected");
    tft.println(" input in:");
    tft.println("BeginMoving");
    delay(1000); 
  }
}
//-----SERVO FUNCTIONS END--------


//-------Simple Helper functions---------

void ledGreen(){
  WiFiDrv::analogWrite(26, 0);
  WiFiDrv::analogWrite(25, 255);
  WiFiDrv::analogWrite(27, 0);
}
void ledRed(){
  WiFiDrv::analogWrite(26, 255);
  WiFiDrv::analogWrite(25, 0);
  WiFiDrv::analogWrite(27, 0);
}
void ledBlue(){
  WiFiDrv::analogWrite(26, 0);
  WiFiDrv::analogWrite(25, 0);
  WiFiDrv::analogWrite(27, 255);
}
void ledYellow(){
  WiFiDrv::analogWrite(26, 255);
  WiFiDrv::analogWrite(25, 255);
  WiFiDrv::analogWrite(27, 0);
}
void ledPurple(){
  WiFiDrv::analogWrite(26, 255);
  WiFiDrv::analogWrite(25, 0);
  WiFiDrv::analogWrite(27, 255);
}
void ledAqua(){
  WiFiDrv::analogWrite(26, 0);
  WiFiDrv::analogWrite(25, 255);
  WiFiDrv::analogWrite(27, 255);
}
void ledOff(){
  WiFiDrv::analogWrite(26, 0);
  WiFiDrv::analogWrite(25, 0);
  WiFiDrv::analogWrite(27, 0);
}


