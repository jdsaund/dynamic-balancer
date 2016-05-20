#include <MPU6050.h>
#include <I2Cdev.h>
#include <stdio.h>
#include <math.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include <Wire.h>
#endif

#define NUMSAMPLES      25

#define TFT_CS          10
#define TFT_RST         9
#define TFT_DC          8

#define Black           0x0000      /*   0,   0,   0 */
#define Navy            0x000F      /*   0,   0, 128 */
#define DarkGreen       0x03E0      /*   0, 128,   0 */
#define DarkCyan        0x03EF      /*   0, 128, 128 */
#define Maroon          0x7800      /* 128,   0,   0 */
#define Purple          0x780F      /* 128,   0, 128 */
#define Olive           0x7BE0      /* 128, 128,   0 */
#define LightGrey       0xC618      /* 192, 192, 192 */
#define DarkGrey        0x7BEF      /* 128, 128, 128 */
#define Blue            0x001F      /*   0,   0, 255 */
#define Green           0x07E0      /*   0, 255,   0 */
#define Cyan            0x07FF      /*   0, 255, 255 */
#define Red             0xF800      /* 255,   0,   0 */
#define Magenta         0xF81F      /* 255,   0, 255 */
#define Yellow          0xFFE0      /* 255, 255,   0 */
#define White           0xFFFF      /* 255, 255, 255 */
#define Orange          0xFD20      /* 255, 165,   0 */
#define GreenYellow     0xAFE5      /* 173, 255,  47 */
#define Pink            0xF81F

#define bgColour        White
#define textColour      Black
#define textColourLight DarkGrey
#define lightLineColour LightGrey
#define darkLineColour  DarkGrey
#define featureColour1  Green
#define featureColour2  Magenta
#define featureColour3  DarkGreen

#define PLOT_RADIUS     52
#define PLOT_CENTER_X   64
#define PLOT_CENTER_Y   80

MPU6050 accelgyro;
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

int rpm                         = 0;
float x2                        = PLOT_CENTER_X;
float y2                        = PLOT_CENTER_Y;
float angle                     = 0;
float magnitude                 = 0;
float coeff                     = 0;
float Q1                        = 0;
float Q2                        = 0;
float sine                      = 0;
float cosine                    = 0;
int N                           = 0;

int16_t sample[NUMSAMPLES + 1]  = {0}; 
uint32_t time[NUMSAMPLES + 1]   = {0}; 
float cycletime[NUMSAMPLES + 1] = {0};

float real                      = 0;
float imag                      = 0;
float realRaw                   = 0; 
float imagRaw                   = 0;
volatile unsigned long start    = 0; 
volatile unsigned long end      = 0;
volatile uint16_t loopTime      = 0;
volatile int ticker             = 0;
volatile int n                  = 0; 
volatile int i                  = 0;

int scaledAngle                 = 0;

int x3[NUMSAMPLES + 1]          = {0};
int y3[NUMSAMPLES + 1]          = {0};
int x4[NUMSAMPLES + 1]          = {0};
int y4[NUMSAMPLES + 1]          = {0};

String sysStatus                = "    ok";
bool clip                       = false;

int button1State                = 0;
int button2State                = 0;
int button1Pin                  = 4;
int button2Pin                  = 5;

static int range                = 0; // 0 = +/-2g, 1 = +/-4g, 2 = +/-8g, 3 = +/-16g
static int mode                 = 0;  // 0 = interactive, 1 = polar, 2 = waveform
const float rangeScale[4]       = {0.00059816, 0.00119633, 0.00239265, 0.00478530}; // 1 / (32767 / g range / 9.8)
const float polarScale[4]       = {19.6, 39.2, 78.4, 156.8};

void setup(void){
  pinMode(button1Pin, INPUT);
  pinMode(button2Pin, INPUT);
  
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
  tft.setTextColor(textColour, bgColour);
  tft.fillScreen(bgColour);
  tft.setRotation(0);

  // splash screen
  splashScreen();

    // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  accelgyro.initialize();
  accelgyro.setRate(0x00); // 1khz/ (1+0) = 1khz
  
  // verify i2c connection
  sysStatus = accelgyro.testConnection() ? "    ok" : "i2cErr";

  // setup tacho
  attachInterrupt(3, ISRspeed, RISING);//number 3 (on digital pin 1) 

  // setup menu - sets the range and mode
  setupMenu();

  accelgyro.setFullScaleAccelRange(range);

  tftStaticGraphics();
  tftDynamicText();

  if(mode == 0){
    tftPlotPolarInteractive();
  } else if(mode == 1){
    tftPlotPolar();
  } else if(mode == 2){
    tftPlotWaveform();
  }
  
}

void loop(void){
  if(ticker == 1){
    sample[i] = accelgyro.getAccelerationX();
    time[i] = micros() - start;

    if (i < NUMSAMPLES){
      i++;
    }

    delayMicroseconds(100);
  }

  if(ticker == 2){
    sample[i] = accelgyro.getAccelerationX();
    time[i] = micros() - start;

    if (i < NUMSAMPLES){
      i++;
    }

    delayMicroseconds(100);
  }
  
  if (ticker == 3){// || !loop2Complete) { // if its this one's turn, or if this one is incomplete
  if(end > start){
    loopTime = end - start;
  }

  for (int k=0; k < NUMSAMPLES; k++){ // only show the first 49 samples
    cycletime[k] = (float)time[k] / (float)loopTime - 0.049;
    if (cycletime[k] < 1.0){
      ProcessSample(sample[k]);
      N++;
    }
  }
    
  i=0;
  realRaw = getReal() * rangeScale[range];
  imagRaw = getImag() * rangeScale[range];
  InitGoertzel();
  N=0;
  }
  
  if(ticker == 4){
    rpm = 120000000 / loopTime;
    polarMath();
    tftDynamicText();

    if(mode == 0){
      tftPlotPolarInteractive();
    } else if(mode == 1){
      tftPlotPolar();
    } else if(mode == 2){
      tftPlotWaveform();
    }

    if (clip) {
      sysStatus = "CLIP!!";
    } else {
      sysStatus = "    ok";
    }
    
    // verify i2c connection
    sysStatus = accelgyro.testConnection() ? "    ok" : "i2cErr";
    
    ticker = 0; // reset scheduler
  }
}

void polarMath(void){
    // smooth the real and imaginary parts
    real = (realRaw*(1-0.9))+(real*0.9);
    imag = (imagRaw*(1-0.9))+(imag*0.9);
    magnitude = sqrt(real*real + imag*imag);
    angle = degrees(atan2(real,imag)) + 80; // +80 dgerees aligns the goertzel estimate with the actual measurements // this may not be correct
}

void tftDynamicText(void){
  tft.setTextColor(textColour, bgColour);
  tft.setCursor(25, 1);
  if (magnitude < 100 && magnitude >= 10){
    tft.print(" ");
    tft.println(magnitude);
  } else  if (magnitude < 10 && magnitude >= 0){
    tft.print("  ");
    tft.println(magnitude);
  } else {
    tft.println(magnitude);
  }

  // angle readout
  tft.setTextColor(textColour, bgColour);
  tft.setCursor(37, 9);

  if (angle >= 100){
    tft.println((int)angle);
  } else if (angle < 100 && angle >= 10){
    tft.print(" ");
    tft.println((int)angle);
  } else if (angle < 10 && angle >= 0){
    tft.print("  ");
    tft.println((int)angle);
  } else if (angle < 0 && angle > -10){
    tft.println(360 + (int)angle);
  } else if (angle <= -10){
    tft.println(360 + (int)angle);
  }

  // rpm readout
  tft.setCursor(95, 1);
  if (rpm < 10000 && rpm >= 1000){
    tft.print(" ");
    tft.println((int)rpm);
  } else if (rpm < 1000 && rpm >= 100){
    tft.print("  ");
    tft.println((int)rpm);
  } else  if (rpm < 100 && rpm >= 10){
    tft.print("   ");
    tft.println((int)rpm);
  } else  if (rpm < 10 && rpm >= 0){
    tft.print("    ");
    tft.println((int)rpm);
  } else {
    tft.println((int)rpm);
  }

  tft.setCursor(89, 9);
  if (sysStatus == "    ok"){
    tft.setTextColor(Green, bgColour);
  } else {
    tft.setTextColor(Red, bgColour);
  }
  tft.print(sysStatus);
}

void tftStaticGraphics(void){
  tft.setTextWrap(false);
  tft.setTextColor(textColourLight);
  tft.setTextSize(0);

  // magnitude readout
  tft.setCursor(1, 1);
  tft.print("mag:");  


  // angle readout
  tft.setCursor(1, 9);
  tft.print("ang:");
  tft.drawCircle(56, 10, 1, textColour);

  // rpm readout
  tft.setCursor(64, 1);
  tft.print("rpm:");

  // status readout
  tft.setCursor(64, 9);
  tft.print("sys:");

  // range readout
  tft.setCursor(1, 151);
  tft.print("range: ");
  tft.setTextColor(textColour);
  if(range == 0){
    tft.print("[+/- 2g]");
  } else if(range == 1){
    tft.print("[+/- 4g]");
  } else if(range == 2){
    tft.print("[+/- 8g]");
  } else if(range == 3){
    tft.print("[+/- 16g]");
  }

  // mode readout
  tft.setTextColor(textColourLight);
  tft.setCursor(1, 143);
  tft.print("mode: ");
  tft.setTextColor(textColour);
  if(mode == 0){
    tft.print(" [interactive]");
  } else if(mode == 1){
    tft.print(" [polar]");
  } else if(mode == 2){
    tft.print(" [waveform]");
  } 

  tft.drawFastHLine(0, 18, 128, lightLineColour);
  tft.drawFastHLine(0, 141, 128, lightLineColour);

  if(mode == 0){
  } else if(mode == 1){
    tft.drawFastVLine(64, 29, 104, lightLineColour);
    tft.drawFastHLine(13, 80, 104, lightLineColour);
    tft.drawCircle(PLOT_CENTER_X, PLOT_CENTER_Y, PLOT_RADIUS * 0.5, lightLineColour);
    tft.drawCircle(PLOT_CENTER_X, PLOT_CENTER_Y, PLOT_RADIUS, lightLineColour);
  
    tft.setTextColor(textColourLight);
    tft.setCursor(120, 66);
    tft.print("+");
    tft.setCursor(120, 77);
    tft.print("0");
    tft.setCursor(120, 87);
    tft.print("-");
  } else if(mode == 2){
    tft.drawFastHLine(0, 100, 128, lightLineColour);

    tft.drawFastVLine(32, 100, 3, lightLineColour);
    tft.setCursor(20, 105);
    tft.print("-180");

    tft.drawFastVLine(64, 100, 3, lightLineColour);
    tft.setCursor(62, 105);
    tft.print("0");

    tft.drawFastVLine(96, 100, 3, lightLineColour);
    tft.setCursor(84, 105);
    tft.print("+180");
  }
}

void ISRspeed(void){
  ticker++;
  if(ticker == 1){
    start = micros();
  }
  if(ticker == 3){
    end = micros();
  }
}

void ResetGoertzel(void){
  Q2 = 0;
  Q1 = 0;
}

void InitGoertzel(void){
  float  floatN;
  float  omega;

  floatN = (float) N;
  omega = (2.0 * PI * 2) / floatN;
  sine = sin(omega);
  cosine = cos(omega);
  coeff = 2.0 * cosine;

  ResetGoertzel();
}

void ProcessSample(int16_t sample){
  float Q0;
  Q0 = coeff * Q1 - Q2 + (float) sample;
  Q2 = Q1;
  Q1 = Q0;

  if (sample >= 32768 || sample <= -32768)
  {
    clip = true;
  } else {
    clip = false;
  }
}

void GetRealImag(float *realPart, float *imagPart){
  *realPart = (Q1 - Q2 * cosine);
  *imagPart = (Q2 * sine);
}

float GetMagnitudeSquared(void){
  float result;

  result = Q1 * Q1 + Q2 * Q2 - Q1 * Q2 * coeff;
  return result;
}

float GetPhase(void){
  float result;
  result = atan2(Q2 * sine, Q1 - Q2 * cosine); // atan2(imag, real);
  return result;
}

float getReal(void){
  float result;
  result = Q1 - Q2 * cosine;
  return result;
}

float getImag(void){
  float result;
  result = Q2 * sine;
  return result;
}

void tftPlotWaveform(void){
  for (int k=0; k < NUMSAMPLES; k++){ // only show the first 49 samples
    if (cycletime[k] <= 1.0 && cycletime[k+1] > 0){
      tft.drawLine(x3[k],y3[k],x4[k],y4[k], bgColour);
      x3[k] = cycletime[k] * 128;
      y3[k] = sample[k] / 800 + 59;
      x4[k] = cycletime[k+1] * 128;
      y4[k] = sample[k+1] / 800 + 59;
      
      tft.drawLine(x3[k],y3[k],x4[k],y4[k], featureColour1);
    }
  }

  tft.drawFastVLine(scaledAngle, 20, 79, bgColour);
  tft.drawFastVLine(scaledAngle + 64, 20, 79, bgColour);
  tft.drawFastVLine(scaledAngle + 128, 20, 79, bgColour);

  scaledAngle = angle * 0.1778; // =  angle * (screen_width / 360degrees / 2 cycles on screen)
  if (scaledAngle >= 10){
    tft.drawFastVLine(scaledAngle, 20, 79, featureColour2);
  }
  if (scaledAngle > -64){
  tft.drawFastVLine(scaledAngle + 64, 20, 79, featureColour2);
  }
  if (scaledAngle < 10){
    tft.drawFastVLine(scaledAngle + 128, 20, 79, featureColour2);
  }
  tft.drawFastHLine(0, 59, 128, lightLineColour);
}

void tftPlotClear(void){
  tft.fillRect(0, 18, 128, 100, bgColour);
}

void tftPlotPolar(void){
  float magMultiplier;
  if(polarScale[range] < magnitude){
    magMultiplier = 1;
  } else {
    magMultiplier = magnitude / polarScale[range];
  }
  
  tft.drawLine(PLOT_CENTER_X, PLOT_CENTER_Y, x2, y2, bgColour);
  
  x2 = PLOT_CENTER_X + magMultiplier*PLOT_RADIUS * cos(radians(-angle));
  y2 = PLOT_CENTER_Y + magMultiplier*PLOT_RADIUS * sin(radians(-angle));

  tft.drawFastVLine(64, 29, 104, lightLineColour);
  tft.drawFastHLine(13, 80, 104, lightLineColour);
  tft.drawCircle(PLOT_CENTER_X, PLOT_CENTER_Y, PLOT_RADIUS * 0.5, lightLineColour);
  tft.drawCircle(PLOT_CENTER_X, PLOT_CENTER_Y, PLOT_RADIUS, lightLineColour);

  tft.drawLine(PLOT_CENTER_X, PLOT_CENTER_Y, x2, y2, featureColour2);
}

void tftPlotPolarInteractive(void){
  static float vector1[2]; // 0 = x2, 1 = y2
  static float vector2[2]; // 0 = x2, 1 = y2
  float vector1magnitude;

  static bool storeVector1, storeVector2 = false;
  
  float magMultiplier;
  if(polarScale[range] < magnitude){
    magMultiplier = 1;
  } else {
    magMultiplier = magnitude / polarScale[range];
  }
  
  tft.drawLine(PLOT_CENTER_X, PLOT_CENTER_Y, x2, y2, bgColour);
  
  x2 = PLOT_CENTER_X + magMultiplier*PLOT_RADIUS * cos(radians(-angle));
  y2 = PLOT_CENTER_Y + magMultiplier*PLOT_RADIUS * sin(radians(-angle));

  tft.drawFastVLine(64, 29, 104, lightLineColour);
  tft.drawFastHLine(13, 80, 104, lightLineColour);
  tft.drawCircle(PLOT_CENTER_X, PLOT_CENTER_Y, PLOT_RADIUS * 0.5, lightLineColour);
  tft.drawCircle(PLOT_CENTER_X, PLOT_CENTER_Y, PLOT_RADIUS, lightLineColour);

  tft.drawLine(PLOT_CENTER_X, PLOT_CENTER_Y, x2, y2, featureColour3);

  button2State = digitalRead(button2Pin);

  if (storeVector1 == true && button2State == HIGH){
    storeVector2 = true; 
    vector2[0] = x2;
    vector2[1] = y2;
  }
  
  if(storeVector1 == false && button2State == HIGH){
    storeVector1 = true; 
    vector1[0] = x2;
    vector1[1] = y2;
    vector1magnitude = magnitude;
  }

  if (storeVector1 == true){
    tft.drawLine(PLOT_CENTER_X, PLOT_CENTER_Y, vector1[0],  vector1[1], lightLineColour);
  }

  if (storeVector2 == true){
    showResults(vector1, vector2, vector1magnitude);
  }
}

void showResults(float vector1[2], float vector2[2], float unbalanceMagnitude){
  tft.fillScreen(bgColour);

  tft.setTextColor(textColour);
  tft.setCursor(23, 1);
  tft.setTextSize(2);
  tft.print("Results");
  tft.setTextSize(0);
  tft.setCursor(7, 17);
  tft.print("Press 'OK' for more");

  int vector3[2]; // corrected index mass vector
  int vector4[2]; // corrected unbalance mass vector
  float vector4angle;
  float vector4magnitude;
  float vector3magnitude;
  
  vector3[0] = (vector2[0] - PLOT_CENTER_X) - (vector1[0] - PLOT_CENTER_X);
  vector3[1] = (vector2[1] - PLOT_CENTER_Y) - (vector1[1] - PLOT_CENTER_Y);

  vector3magnitude = sqrt(vector3[0] * vector3[0] + vector3[1] * vector3[1]);

  vector4angle = atan2(vector1[1] - PLOT_CENTER_Y, vector1[0] - PLOT_CENTER_X) - atan2(vector3[1], vector3[0]);
  vector4magnitude = sqrt((vector1[0] - PLOT_CENTER_X) * (vector1[0] - PLOT_CENTER_X) + (vector1[1] - PLOT_CENTER_Y) * (vector1[1] - PLOT_CENTER_Y));
  vector4[0] = vector4magnitude * cos(vector4angle);
  vector4[1] = vector4magnitude * sin(vector4angle);

  while(1){
    delay(100);
    static bool page2 = false;
    
    button2State = digitalRead(button2Pin);
    
    if (button2State == HIGH){
      page2 = !page2;
      tft.fillRect(1, 28, 128, 140, bgColour);
    }

    if(!page2){
      tft.drawFastVLine(64, 29, 104, lightLineColour);
      tft.drawFastHLine(13, 80, 104, lightLineColour);
      tft.drawCircle(PLOT_CENTER_X, PLOT_CENTER_Y, PLOT_RADIUS * 0.5, lightLineColour);
      tft.drawCircle(PLOT_CENTER_X, PLOT_CENTER_Y, PLOT_RADIUS, lightLineColour);
      tft.setTextColor(textColourLight);
      tft.setCursor(120, 66);
      tft.print("+");
      tft.setCursor(120, 77);
      tft.print("0");
      tft.setCursor(120, 87);
      tft.print("-");
    
      tft.fillRect(1, 139, 7, 7, lightLineColour); // key
      tft.fillRect(1, 150, 7, 7, featureColour3);  // key
      tft.fillRect(64, 139, 7, 7, featureColour1); // key
      tft.fillRect(64, 150, 7, 7, featureColour2); // key
    
      tft.setCursor(9, 139);
      tft.print("initial");
      tft.setCursor(9, 150);
      tft.print("tst mass");
      tft.setCursor(73, 139);
      tft.print("tst corr");
      tft.setCursor(73, 150);
      tft.print("final");
    
      tft.drawLine( PLOT_CENTER_X,  PLOT_CENTER_Y,  vector1[0],  vector1[1], lightLineColour);
      tft.drawLine( PLOT_CENTER_X,  PLOT_CENTER_Y,  vector2[0],  vector2[1], darkLineColour);
      tft.drawLine( PLOT_CENTER_X,  PLOT_CENTER_Y,  PLOT_CENTER_X + vector3[0],  PLOT_CENTER_Y + vector3[1], featureColour1);
      tft.drawLine( PLOT_CENTER_X,  PLOT_CENTER_Y,  PLOT_CENTER_X + vector4[0],  PLOT_CENTER_Y + vector4[1], featureColour2);
    } else {
      tft.setTextColor(textColourLight);
      tft.setCursor(1, 30);
      tft.print("Test mass multiplier:");
      tft.setTextSize(2);
      tft.setTextColor(textColour);
      tft.setCursor(1, 41);
      tft.print(vector4magnitude / vector3magnitude);
      tft.setTextSize(0);
      tft.setTextColor(textColourLight);
      tft.setCursor(1, 61);
      tft.print("At an angle of:");
      tft.setTextSize(2);
      tft.setTextColor(textColour);
      tft.setCursor(1, 72);
      tft.print(degrees(-vector4angle)); tft.setTextSize(0); tft.print("o");
      tft.setTextSize(0);
    }
  }
}

void setupMenu(void){
  tft.fillScreen(bgColour);
  tft.setTextColor(textColour);
  tft.setCursor(33, 1);
  tft.setTextSize(2);
  tft.print("Setup");
  tft.setCursor(7, 17);

  // select range
  tft.setTextSize(0);
  tft.setTextColor(textColour);
  tft.setCursor(6, 17);
  tft.print("Accelerometer range:");

  tft.setTextColor(textColourLight);
  tft.setCursor(30, 40);
  tft.print("+/-  2g");
  tft.setCursor(30, 51);
  tft.print("+/-  4g");
  tft.setCursor(30, 62);
  tft.print("+/-  8g");
  tft.setCursor(30, 73);
  tft.print("+/- 16g");

  while(1){
    button1State = digitalRead(button1Pin);
    button2State = digitalRead(button2Pin);

    if(button1State == HIGH)
      {
        if(range < 3){
          range++;
        } else {
          range = 0;
        }
        tft.drawRect(28, 38, 72, 11, bgColour);
        tft.drawRect(28, 49, 72, 11, bgColour);
        tft.drawRect(28, 60, 72, 11, bgColour);
        tft.drawRect(28, 71, 72, 11, bgColour);
      }

    if(range == 0){
      tft.drawRect(28, 38, 72, 11, featureColour2);
    }
    if(range == 1){
      tft.drawRect(28, 49, 72, 11, featureColour2);
    }
    if(range == 2){
      tft.drawRect(28, 60, 72, 11, featureColour2);
    }
    if(range == 3){
      tft.drawRect(28, 71, 72, 11, featureColour2);
    }

  if(button2State == HIGH)
    {
      break;
    }

    delay(200);
  }

  tft.fillRect(1, 17, 128, 130, bgColour);

  // select plot mode
  tft.setTextColor(textColour);
  tft.setCursor(35, 17);
  tft.print("Test mode:");

 tft.setTextColor(textColourLight);
  tft.setCursor(30, 40);
  tft.print("interactive");
  tft.setCursor(30, 51);
  tft.print("polar");
  tft.setCursor(30, 62);
  tft.print("waveform");

  while(1){
    button1State = digitalRead(button1Pin);
    button2State = digitalRead(button2Pin);

    if(button1State == HIGH)
      {
        if(mode < 2){
          mode++;
        } else {
          mode = 0;
        }
        tft.drawRect(28, 38, 72, 11, bgColour);
        tft.drawRect(28, 49, 72, 11, bgColour);
        tft.drawRect(28, 60, 72, 11, bgColour);
      }

    if(mode == 0){
      tft.drawRect(28, 38, 72, 11, featureColour2);
    }
    if(mode == 1){
      tft.drawRect(28, 49, 72, 11, featureColour2);
    }
    if(mode == 2){
      tft.drawRect(28, 60, 72, 11, featureColour2);
    }


  if(button2State == HIGH)
    {
      break;
    }

    delay(200);
  }

  tft.fillScreen(bgColour);
}

void splashScreen(void){
  tft.fillScreen(bgColour);
  tft.setTextColor(textColour);
  tft.setTextSize(0);
  
  tft.setCursor(36, 29);
  tft.print("Very Small");
  delay(400);
  tft.setCursor(30, 40);
  tft.print("Single Plane");
  delay(400);
  tft.setCursor(42, 51);
  tft.print("Balancer");
  
  delay(1000);
  tft.setCursor(1, 152);
  tft.print("v.01a");
  delay(2000);
}

