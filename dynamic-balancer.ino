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

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

float p = 3.1415926;
int rpm = 1000;
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

void setup(void) {
  Serial.begin(115200);
  
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
  tft.setTextColor(White, Black);
  tft.fillScreen(ST7735_BLACK);
  
  // tft print function!
  //tftFixedGraphics();

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

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  accelgyro.setRate(0x00); // 1khz/ (1+0) = 1khz
  
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // setup tacho
  attachInterrupt(4, ISRspeed, RISING);//number 4 (on digital pin 7) 
  digitalWrite(13, LOW);
}

void loop() {
  while(ticker == 1){
    sample[i] = accelgyro.getAccelerationX();
    time[i] = micros() - start;
    if (i <= 50){
      i++;
    }
  }

  while(ticker == 2){
    sample[i] = accelgyro.getAccelerationX();
    time[i] = micros() - start;
    if (i <= 50){
      i++;
    }
  }
    
  if (ticker == 3){// || !loop2Complete) { // if its this one's turn, or if this one is incomplete
    loopTime = end - start;
      for (int k=0; k <= 49; k++){ // only show the first 49 samples
        cycletime[k] = (float)time[k] / (2*(float)loopTime) - 0.049;
        if (cycletime[k] < 1.0){
//          Serial.print(cycletime, 4);
//          Serial.print(", "); 
//          Serial.println(sample[k]);
          ProcessSample(sample[k]);
          N++;
        }
      }
      
    realRaw = getReal();
    imagRaw = getImag();
    i=0;
    InitGoertzel();
    N=0;
    while(ticker == 2){}
    }
  
  if(ticker == 4){
    tftPlot();
    
//    rpm = 60000000 / loopTime;
//    tftDynamicGraphicsErase();
    polarMath();
//    tftDynamicGraphicsDraw();
    
    ticker = 0; // reset scheduler
while(ticker == 0){}
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
    imag = (imagRaw*(1-0.9))+(imag*0.9);
    magnitude = sqrt(real*real + imag*imag);
    angle = degrees(atan2(real,imag)) + 80;
//    if (angle < 0) {
//      angle = 360 - angle;
//    }
  
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
   if (magnitude < 500) {
    tft.drawLine(x1, y1, x2, y2, Green);
    tft.setTextColor(Green, Black);
  } else if (magnitude > 1000){
    tft.drawLine(x1, y1, x2, y2, Red);
    tft.setTextColor(Red, Black);
  } else {
    tft.drawLine(x1, y1, x2, y2, Orange);
    tft.setTextColor(Orange, Black);
  }

  tft.setCursor(25, 1);
  if (magnitude < 10000 && magnitude >= 1000){
    tft.print(" ");
    tft.println((int)magnitude);
  } else if (magnitude < 1000 && magnitude >= 100){
    tft.print("  ");
    tft.println((int)magnitude);
  } else  if (magnitude < 100 && magnitude >= 10){
    tft.print("   ");
    tft.println((int)magnitude);
  } else  if (magnitude < 10 && magnitude >= 0){
    tft.print("    ");
    tft.println((int)magnitude);
  } else {
    tft.println((int)magnitude);
  }

  // angle readout
  tft.setTextColor(White, Black);
  tft.setCursor(110, 1);

  if (angle >= 100){
    tft.println((int)angle);
  } else  if (angle < 100 && angle >= 10){
    tft.print(" ");
    tft.println((int)angle);
  } else  if (angle < 10 && angle >= 0){
    tft.print("  ");
    tft.println((int)angle);
  }

  // rpm readout
  if (rpm < 2000) {
    tft.setTextColor(Cyan, Black);
  } else if (rpm > 7000){
    tft.setTextColor(Red, Black);
  } else {
    tft.setTextColor(Green, Black);
  }
    
  tft.setCursor(25, 10);
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
}

void tftDynamicGraphicsErase(){
  // polar arrow
  tft.drawLine(x1, y1, x2, y2, Black);
}

void tftFixedGraphics() {
  tft.drawCircle(64, 64, radiusOutline, ST7735_WHITE);
  tft.setTextWrap(false);
  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(0);

  // fixed angle references
  tft.setCursor(59, 11);
  tft.println("90");
  tft.setCursor(56, 111);
  tft.println("270");
  tft.setCursor(111, 61);
  tft.println("0");
  tft.setCursor(1, 61);
  tft.println("180");

  // press to cont.
  tft.setCursor(6, 140);
  tft.print("[press to continue]");  

  // magnitude readout
  tft.setCursor(1, 1);
  tft.print("mag:");  


  // angle readout
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(86, 1);
  tft.print("ang:"); 

  // rpm readout
  tft.setCursor(1, 10);
  tft.print("rpm:");  
}

void ISRspeed()
{
  
  if(ticker == 0){
    start = micros();
  }
    if(ticker == 1){
    end = micros();
  }
  ticker++;
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

void tftPlot(void)
{
  for (int k=0; k <= 49; k++){ // only show the first 49 samples
    if (cycletime[k] <= 1.0 && cycletime[k+1] <= 1.0){
      tft.drawLine(x3[k],y3[k],x4[k],y4[k], Black);
      x3[k] = cycletime[k] * 108 + 10;
      y3[k] = sample[k] / 1000 + 40;
      x4[k] = cycletime[k+1] * 108 + 10;
      y4[k] = sample[k+1] / 1000 + 40;
      
      tft.drawLine(x3[k],y3[k],x4[k],y4[k], Green);
      //tft.drawPixel(x3[k], y3[k], Red); 
    }
  }

  tft.drawFastVLine(scaledAngle, 20, 20, Black);
  tft.drawFastVLine(scaledAngle + 54, 20, 20, Black);
  tft.drawFastVLine(scaledAngle + 108, 20, 20, Black);

  scaledAngle = angle *0.15+10;
  if (scaledAngle >= 10){
    tft.drawFastVLine(scaledAngle, 20, 20, Magenta);
  }
  if (scaledAngle > -64){
  tft.drawFastVLine(scaledAngle + 54, 20, 20, Magenta);
  }
  if (scaledAngle < 10){
    tft.drawFastVLine(scaledAngle + 108, 20, 20, Magenta);
  }
  tft.drawFastHLine(10, 40, 108, White);
}

