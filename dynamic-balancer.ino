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

MPU6050 accelgyro;

#define TFT_CS     10
#define TFT_RST    9
#define TFT_DC     8

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

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

const float p = 3.1415926;
int rpm;
int radiusOutline = 42;
int radiusPolarMax = 40;
int x1 = 64;
int y1 = 64;
float x2;
float y2;
float angle = 0;
float magnitude = 0;
float magmax = 1000; // 1000 milli-g's
float coeff;
float Q1;
float Q2;
float sine;
float cosine;
int N, M;
volatile uint16_t loopTime;
int16_t sample[52]; 
uint32_t time[52]; 
float cycletime[52];
volatile unsigned long start, end;
uint32_t start1, end1;
uint32_t tick2time, tick3time;
//float ang, mag, angRaw, magRaw;
float real, imag, realRaw, imagRaw;
volatile int ticker, n, i = 0;

int scaledMag;
int scaledAngle;

int x3[52];
int y3[52];
int x4[52];
int y4[52];
int x5;
int y5;

String sysStatus;
bool clip = false;

volatile bool loopEnabled = false;

void setup(void) {
  Serial.begin(115200);
  
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
  tft.setTextColor(textColour, bgColour);
  tft.fillScreen(bgColour);
  tft.setRotation(0);

    // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  
  pinMode(13, OUTPUT);//LEDPIN
  delay(10);
  digitalWrite(13, HIGH);
  delay(10);

  accelgyro.initialize();
  accelgyro.setRate(0x00); // 1khz/ (1+0) = 1khz
  
  // verify connection
  sysStatus = accelgyro.testConnection() ? "    ok" : "i2cErr";

  tftFixedGraphics();
  tftDynamicGraphicsDraw();
  tftWaveform();

  // setup tacho
  attachInterrupt(3, ISRspeed, RISING);//number 3 (on digital pin 1) 
  digitalWrite(13, LOW);
}

void loop() {
  if(ticker == 1){
    sample[i] = accelgyro.getAccelerationX();
    time[i] = micros() - start;

    if (i <= 50){
      i++;
    }
    
    delayMicroseconds(100);
  }

  if(ticker == 2){
    sample[i] = accelgyro.getAccelerationX();
    time[i] = micros() - start;
    
    if (i <= 50){
      i++;
    }
    
    delayMicroseconds(100);
  }
    
  if (ticker == 3){// || !loop2Complete) { // if its this one's turn, or if this one is incomplete
    loopTime = end - start;
      for (int k=0; k <= 49; k++){ // only show the first 49 samples
        cycletime[k] = (float)time[k] / (float)loopTime - 0.049;
        if (cycletime[k] < 1.0){
          ProcessSample(sample[k]);
          N++;
        }
      }
      
    i=0;
    realRaw = getReal() / 1671.8; // dividing by 1671.8 converts the reading into ms^-2
    imagRaw = getImag() / 1671.8; // dividing by 1671.8 converts the reading into ms^-2
    InitGoertzel();
    N=0;
  }
  
  if(ticker == 4){
    rpm = 120000000 / loopTime;
    polarMath();
    tftDynamicGraphicsDraw();
    tftWaveform();

    if (clip) {
      sysStatus = "CLIP!!";
      clip = false;
    } else {
      sysStatus = "    ok";
    }
    
    ticker = 0; // reset scheduler
  }
}

void testdrawtext(char *text, uint16_t color) {
  tft.setCursor(0, 0);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.print(text);
}

void polarMath(){
    // smooth the real and imaginary parts
    real = (realRaw*(1-0.9))+(real*0.9);
    Serial.println(real);
    imag = (imagRaw*(1-0.9))+(imag*0.9);
    Serial.println(imag);
    magnitude = sqrt(real*real + imag*imag);
    angle = degrees(atan2(real,imag)) + 80; // +80 dgerees aligns the goertzel estimate with the actual measurements
  
//  float magMultiplier;
//  if(magmax < magnitude){
//    magMultiplier = 1;
//  } else {
//    magMultiplier = magnitude / magmax;
//  }
//  
//  x2 = x1 + magMultiplier*radiusPolarMax * cos(radians(-angle));
//  y2 = y1 + magMultiplier*radiusPolarMax * sin(radians(-angle));
}

void tftDynamicGraphicsDraw(){
  // magnitude readout
//   if (magnitude < 500) {
//    tft.drawLine(x1, y1, x2, y2, Green);
//    tft.setTextColor(Green, bgColour);
//  } else if (magnitude > 1000){
//    tft.drawLine(x1, y1, x2, y2, Red);
//    tft.setTextColor(Red, bgColour);
//  } else {
//    tft.drawLine(x1, y1, x2, y2, Orange);
//    tft.setTextColor(Orange, bgColour);
//  }
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

void tftDynamicGraphicsErase(){
  // polar arrow
  tft.drawLine(x1, y1, x2, y2, bgColour);
}

void tftFixedGraphics() {
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

  tft.drawFastHLine(0, 18, 128, lightLineColour);
  tft.drawFastHLine(0, 100, 128, lightLineColour);
}

void ISRspeed()
{
  ticker++;
  if(ticker == 1){
    start = micros();
  }
  if(ticker == 3){
    end = micros();
  }
}

void ResetGoertzel(void)
{
  Q2 = 0;
  Q1 = 0;
}

void InitGoertzel()
{
  float  floatN;
  float  omega;

  floatN = (float) N;
  omega = (2.0 * PI * 2) / floatN;
  sine = sin(omega);
  cosine = cos(omega);
  coeff = 2.0 * cosine;

  ResetGoertzel();
}

void ProcessSample(int16_t sample)
{
  float Q0;
  Q0 = coeff * Q1 - Q2 + (float) sample;
  Q2 = Q1;
  Q1 = Q0;

  if (sample >= 32768 || sample <= -32768)
  {
    clip = true;
  }
}

void GetRealImag(float *realPart, float *imagPart)
{
  *realPart = (Q1 - Q2 * cosine);
  *imagPart = (Q2 * sine);
}

float GetMagnitudeSquared(void)
{
  float result;

  result = Q1 * Q1 + Q2 * Q2 - Q1 * Q2 * coeff;
  return result;
}

float GetPhase(void)
{
  float result;
  result = atan2(Q2 * sine, Q1 - Q2 * cosine); // atan2(imag, real);
  return result;
}

float getReal(void)
{
  float result;
  result = Q1 - Q2 * cosine;
  return result;
}

float getImag(void)
{
  float result;
  result = Q2 * sine;
  return result;
}

void tftWaveform(void)
{
  for (int k=0; k <= 49; k++){ // only show the first 49 samples
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
  tft.drawFastVLine(64, 55, 8, darkLineColour);
  tft.drawFastVLine(32, 57, 4, darkLineColour);
  tft.drawFastVLine(96, 57, 4, darkLineColour);
}

