/* This example shows how to use continuous mode to take
range measurements with the VL53L0X. It is based on
vl53l0x_ContinuousRanging_Example.c from the VL53L0X API.
The range readings are in units of mm. */
#define FASTLED_ESP8266_NODEMCU_PIN_ORDER
#include <Wire.h>
#include <VL53L0X.h>
#include <FastLED.h>
#include <AnalogSmooth.h>

//Lidar initialization
VL53L0X sensor;

//LED options
#define LED_PIN     6
#define NUM_LEDS    5
#define BRIGHTNESS  255
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];
#define UPDATES_PER_SECOND 100

//Global scripting variables
int distance = 0;
int LEDsToOn = 0;

//Global input variables
int Button1 = 0;

//Global timing variables
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
int workingFadeINCycle[] = {0,0,0,0}; //declare array for working timing
int workingFadeOUTCycle[] = {255,255,255,255};

//Smoothing setup
int SmoothDistance = 0;
AnalogSmooth as15 = AnalogSmooth(15);

//
void setup()
{
  Serial.begin(9600);

  //Lidar initialize
  Wire.begin();
  sensor.init();
  sensor.setTimeout(1500);
  
  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous();

  //Initialized LED array
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.clear(); //clear all LEDs before we start too much
  FastLED.show();
  delay(100);
}

void loop()
{

  //Global timing functions
  previousMillis = millis();
  
  //read the analog values
  
  //Lidar Serial Reporting
  Serial.print(SmoothDistance);
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.println();

  //add variable dealing with distance 
  distance = sensor.readRangeSingleMillimeters();
  delay(20);

  //smooth the distance readings
  SmoothDistance = as15.smooth(distance);
  
  if(Button1 == HIGH) {
    LEDFadeIN(0,0,255,255,8);
  }else{
    LEDFadeOUT(0,0,255,255,8); //specify the color we want to fade to, in 0-255 format
  }
}

void LEDFadeIN(int workingLEDNumber, int workingH, int workingS, int workingV, int workingFadeSpeed){
  if(workingFadeINCycle[workingLEDNumber] < 255)
  {
    workingFadeINCycle[workingLEDNumber] = workingFadeINCycle[workingLEDNumber] + workingFadeSpeed; //adjust this number for speed of gain
    leds[workingLEDNumber] = CHSV(workingH,workingS,workingFadeINCycle[workingLEDNumber]);
    Serial.println((String)"in loop 2 " + workingFadeINCycle[workingLEDNumber]);
  }
  else
  {
    leds[workingLEDNumber] = CHSV (workingH, workingS, 255);
  }
  FastLED.show();
}

void LEDFadeOUT(int workingLEDNumber, int workingH, int workingS, int workingV, int workingFadeSpeed){
  if(workingFadeOUTCycle[workingLEDNumber] >= workingFadeSpeed)
  {
    workingFadeOUTCycle[workingLEDNumber] = workingFadeOUTCycle[workingLEDNumber] - workingFadeSpeed; //adjust this number for speed of gain
    leds[workingLEDNumber] = CHSV(workingH,workingS,workingFadeOUTCycle[workingLEDNumber]);
    Serial.println((String)"in loop 3 " + workingFadeOUTCycle[workingLEDNumber]);
  }
  else
  {
    leds[workingLEDNumber] = CHSV (workingH, workingS, 0);
  }
  FastLED.show();
}
