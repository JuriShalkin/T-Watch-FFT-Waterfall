#include "arduinoFFT.h"          // Standard Arduino FFT library 
arduinoFFT FFT = arduinoFFT();
#include <TTGO.h>
#define SAMPLES 512              // Must be a power of 2
#define SAMPLING_FREQUENCY 40000 
// Hz, must be 40000 or less due to ADC conversion time.
// Determines maximum frequency that can be analysed by the FFT Fmax=sampleF/2.

TTGOClass *ttgo;

int micpin = 33; //change this to 36 if you are using the fc-04

unsigned int sampling_period_us;
unsigned long microseconds;
double vReal[SAMPLES];
double vImag[SAMPLES];
unsigned long newTime, oldTime;
uint16_t tft_width  = 240; // ST7789_TFTWIDTH;
uint16_t tft_height = 240; // ST7789_TFTHEIGHT;
int hist_height = 160; 
int watfall[14400];

// variables for interpolated colors
byte red, green, blue;

float MinAmp = 0;
float MaxAmp = 160; //4096;
float a, b, c, d;
int base_amp = 200;

void setup() {
  Serial.begin(115200);
  ttgo = TTGOClass::getWatch();
  ttgo->begin();
  ttgo->openBL();
  ttgo->lvgl_begin();
  ttgo->eTFT->fillScreen(TFT_BLACK);
  ttgo->eTFT->setTextColor(TFT_YELLOW, TFT_BLACK);
  ttgo->eTFT->setTextSize(1);
  ttgo->eTFT->setRotation(0);
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
  delay(2000);

}

void loop() {
  for (int i = 0; i < SAMPLES; i++) {
  newTime = micros()-oldTime; //newTime = 30 - 5 = 25
  oldTime = newTime; //oldTime=25
  vReal[i] = analogRead(micpin); // A conversion takes about 1uS on an ESP32
  vImag[i] = 0;
  while (micros() < (newTime + sampling_period_us)) { 
      // do nothing to wait
    }
  }
  //FFT.DCRemoval();  
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
  
  double x = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
  Serial.println(x);
  ttgo->eTFT->setTextFont(0);        // Select font 0 which is the Adafruit font
  //ttgo->eTFT->setCursor(0, 10);
  //ttgo->eTFT->print(String(x, 0));
  ttgo->eTFT->drawString(String(x, 0), 0, 0, 2);

  drawHisto();

  //drawGraph();
  
  // get the cutoff points for the color interpolation routines
  // note this function called when the temp scale is changed
  Getabcd();
  
  drawWaterfall();
}
  
void drawHisto() {
  for (int k = 0; k < tft_width; k++){ 
    int amp = vReal[k+2] / base_amp; //используем отсчеты со 2-го
    //amp = map(vReal[k], 0 , MaxAmp, 0, hist_height);
    if (amp > hist_height){
      amp = hist_height;
      }
    ttgo->eTFT->drawFastVLine(k, hist_height-amp+20, amp, TFT_YELLOW); 
    ttgo->eTFT->drawFastVLine(k, 20, hist_height-amp, TFT_BLACK);
    }
}
  
void drawWaterfall(){
    for (int n = (tft_height/4)*tft_width; n > tft_width; n--) {                                //сдвигаем массив на tft_width
      watfall[n]=watfall[n-tft_width];
    }
    for (int n = 0; n <= tft_width; n++) {
      watfall[n]=vReal[n+2];                                                                    //используя отсчеты со 2-го заполняем первые tft_width элементов массива значениями амплитуд
    }
    for (int r = 1; r <= tft_height/4; r++) {
      for (int c = 1; c <= tft_width; c++) {
      ttgo->eTFT->drawPixel(c, tft_height/4*3+r, GetColor(watfall[tft_width*(r-1)+c]/base_amp));
      } 
    }
}

void drawGraph(){
    for (int m = 2; m <= tft_width; m++){
      ttgo->eTFT->drawLine(m, hist_height-vReal[m]/base_amp+20, m+1, hist_height-vReal[m+1]/base_amp+20, TFT_YELLOW);
    }
    ttgo->eTFT->fillRect(0, 20, tft_width, hist_height, TFT_BLACK);
    //for (int m = 2; m <= tft_width; m++){
      //ttgo->eTFT->drawLine(m, hist_height-vReal[m]/base_amp+20, m+1, hist_height-vReal[m+1]/base_amp+20, TFT_BLACK);
    //}
}

uint16_t GetColor(float val) {
  /*
    pass in value and figure out R G B
    several published ways to do this I basically graphed R G B and developed simple linear equations
    again a 5-6-5 color display will not need accurate temp to R G B color calculation
    equations based on
    http://web-tech.ga-usa.com/2012/05/creating-a-custom-hot-to-cold-temperature-color-gradient-for-use-with-rrdtool/index.html
  */
  red = constrain(255.0 / (c - b) * val - ((b * 255.0) / (c - b)), 0, 255);
  if ((val > MinAmp) & (val < a)) {
    green = constrain(255.0 / (a - MinAmp) * val - (255.0 * MinAmp) / (a - MinAmp), 0, 255);
  }
  else if ((val >= a) & (val <= c)) {
    green = 255;
  }
  else if (val > c) {
    green = constrain(255.0 / (c - d) * val - (d * 255.0) / (c - d), 0, 255);
  }
  else if ((val > d) | (val < a)) {
    green = 0;
  }
  if (val <= b) {
    blue = constrain(255.0 / (a - b) * val - (255.0 * b) / (a - b), 0, 255);
  }
  else if ((val > b) & (val <= d)) {
    blue = 0;
  }
  else if (val > d) {
    blue = constrain(240.0 / (MaxAmp - d) * val - (d * 240.0) / (MaxAmp - d), 0, 240);
  }
  // use the displays color mapping function to get 5-6-5 color palet (R=5 bits, G=6 bits, B-5 bits)
  return ttgo->eTFT->color565(red, green, blue);
}

// function to get the cutoff points in the temp vs RGB graph
void Getabcd() {
  a = MinAmp + (MaxAmp - MinAmp) * 0.2121;
  b = MinAmp + (MaxAmp - MinAmp) * 0.3182;
  c = MinAmp + (MaxAmp - MinAmp) * 0.4242;
  d = MinAmp + (MaxAmp - MinAmp) * 0.8182;
  }
