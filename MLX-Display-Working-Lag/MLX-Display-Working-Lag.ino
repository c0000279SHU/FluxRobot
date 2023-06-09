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
//-------------
//DISPLAY
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

#define TFT_CS         3
#define TFT_RST        2 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC         1

//Connect MOSI and SCLK respectively on MK1010
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

//OR uncomment and assign
//#define TFT_MOSI 11  // Data out
//#define TFT_SCLK 13  // Clock out
//Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);


float p = 3.1415926;


void setup() {
  //------MLX CAM SETUP-----------
  while (!Serial) delay(10);
  Serial.begin(115200);
  delay(100);

  Serial.println("Adafruit MLX90640 Simple Test");
  if (! mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("MLX90640 not found!");
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
  //------MLX CAM SETUP END----------------


  displayPixelWidth = tft.width() / 32;
  displayPixelHeight = tft.width() / 32; //Keep pixels square 

  
  //--------DISPLAY SETUP------
  Serial.begin(9600);
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
  tft.setRotation(3);
  screenLogo();
  //---------DISPLAY SETUP END------------------
}

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
  tft.setCursor((tft.height()/2)-10, (tft.width()/2)-75);
  tft.println("AEM Tech");
  delay(1000);
  tft.print("     ");
  tft.print(". ");
  delay(250);
  tft.print(". ");
  delay(250);
  tft.print(". ");
  delay(250);
  tft.print(". ");
  delay(750);
  tft.fillScreen(ST77XX_BLACK);
}
/*
uint16_t tempColor (float temp){
  uint16_t c = 0x019F;
  if (t < 20) c = ' ';
  else if (t < 23) c = '.';
  else if (t < 25) c = '-';
  else if (t < 27) c = '*';
  else if (t < 29) c = '+';
  else if (t < 31) c = 'x';
  else if (t < 33) c = '%';
  else if (t < 35) c = '#';
  else if (t < 37) c = 'X';
}//*/
void loop() {
  delay(500);
  if (mlx.getFrame(frame) != 0) {
    Serial.println("Failed");
    return;
  }
  //Serial.println();
  //Serial.println();

  int colorTemp;
  for (uint8_t h=0; h<24; h++) {
    for (uint8_t w=0; w<32; w++) {
      float t = frame[h*32 + w];
      
      //Serial.print(t, 1);
      //Serial.print(", ");
      
      t = min(t, MAXTEMP);
      t = max(t, MINTEMP); 

      //tft.drawPixel(tft.width()/2, tft.height()/2, ST77XX_GREEN);
      uint8_t colorIndex = map(t, MINTEMP, MAXTEMP, 0, 255);
      
      colorIndex = constrain(colorIndex, 0, 255);
      //draw the pixels!
      tft.fillRect(displayPixelWidth * w, displayPixelHeight * h,
                               displayPixelHeight, displayPixelWidth, 
                               camColors[colorIndex]);
    }
    //Serial.println();
  }
  
}//end loop

//Transform 32*24 to 320 * 170 pixel
void interpolation(float *data, uint16_t *out)
{

    for (uint8_t h = 0; h < 24; h++)
    {
        for (uint8_t w = 0; w < 32; w++)
        {
            out[h * 10 * 320 + w * 10] = map_f(data[h * 32 + w], MINTEMP, MAXTEMP);
        }
    }
    for (int h = 0; h < 170; h += 10)
    {
        for (int w = 1; w < 310; w += 10)
        {
            for (int i = 0; i < 9; i++)
            {
                out[h * 320 + w + i] = (out[h * 320 + w - 1] * (9 - i) + out[h * 320 + w + 9] * (i + 1)) / 10;
            }
        }
        for (int i = 0; i < 9; i++)
        {
            out[h * 320 + 311 + i] = out[h * 320 + 310];
        }
    }
    for (int w = 0; w < 320; w++)
    {
        for (int h = 1; h < 230; h += 10)
        {
            for (int i = 0; i < 9; i++)
            {
                out[(h + i) * 320 + w] = (out[(h - 1) * 320 + w] * (9 - i) + out[(h + 9) * 320 + w] * (i + 1)) / 10;
            }
        }
        for (int i = 0; i < 9; i++)
        {
            out[(231 + i) * 320 + w] = out[230 * 320 + w];
        }
    }
    for (int h = 0; h < 170; h++)
    {
        for (int w = 0; w < 320; w++)
        {
            out[h * 320 + w] = camColors[out[h * 320 + w]];
        }
    }
}
