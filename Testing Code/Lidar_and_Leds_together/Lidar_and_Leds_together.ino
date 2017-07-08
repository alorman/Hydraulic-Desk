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
int Button1 = 1;

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
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.clear(); //clear all LEDs before we start too much
  FastLED.show();
  delay(100);
}

void loop()
{

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
    leds[0] = CRGB::Green;
    FastLED.show();
    LEDFadein(0);
  }else{
    LEDButtonOff(0);
  }
}

//led Button response subrouting
void LEDButtonResponse(int workingLEDNumber){
  leds[workingLEDNumber] = CRGB::Green;
  FastLED.show();
}

void LEDButtonOff(int workingLEDNumber){
  leds[workingLEDNumber] = CRGB::Black;
  FastLED.show();
}

void LEDFadein(int workingLEDNumber){
  int WorkingBrightness = 0;
  int FadeAmount = 1;
  leds[workingLEDNumber] = CRGB::Green;
  for(int i=0; i <= 256; i++)
  {
    leds[workingLEDNumber].fadeLightBy(WorkingBrightness);
    Serial.println("in loop");
    WorkingBrightness = WorkingBrightness + FadeAmount;
    FastLED.show();
  }
}

