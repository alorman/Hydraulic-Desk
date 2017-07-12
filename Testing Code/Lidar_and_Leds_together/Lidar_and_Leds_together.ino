/* This example shows how to use continuous mode to take
range measurements with the VL53L0X. It is based on
vl53l0x_ContinuousRanging_Example.c from the VL53L0X API.
The range readings are in units of mm. */
#define FASTLED_ESP8266_NODEMCU_PIN_ORDER
#include <Wire.h>
#include <VL53L0X.h>
#include <FastLED.h>
#include <AnalogSmooth.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>

//wifi Initiation
const char* ssid = "nestlink";
const char* wifipassword = "nestlink";
const char* mqtt_server = "192.168.0.127";
int WifiAttempts = 0;
int WifiMaxAttempts = 5;
int WifiOnline = 0;

//setup pubsub
WiFiClient espClient;
PubSubClient client (espClient);
long lastMsg = 0;
char msg[50];
int value = 0;
char* connectionStatus = "0";
String username = "homeassistant";
String password = "";
const char* ConnectedTopic = "/desk/connected";
const char* HeightTopic = "/desk/actualheight";
const char* CommandedHeightTopic = "/desk/commandedheight";
const char* ErrorTopic = "/desk/error";
const char* ExecuteTopic = "/desk/execute";

//Lidar initialization
VL53L0X sensor;

//LED options
#define LED_PIN     6
#define NUM_LEDS    4
#define BRIGHTNESS  255
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];
#define UPDATES_PER_SECOND 100

//Pin setup
#define SW1Pin D5

//Global scripting variables
int distance = 0;
int LEDsToOn = 0;

//Global input variables
int Button1 = 0;
int Test1 = 0;

//Global timing variables
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
int workingFadeCycle[] = {0,0,0,0}; //declare array for working timing

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

  //pin mode setup
  pinMode(SW1Pin, INPUT);

  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous();

  //Initialized LED array
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.clear(); //clear all LEDs before we start too much
  FastLED.show();
  delay(100);

  //setup wifi and connect to mqtt
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop()
{

  //Global timing functions
  previousMillis = millis();
  
  //read the analog and digital values
  Button1 = digitalRead(SW1Pin);
  //Serial.println(Test1);
  
  //Lidar Serial Reporting
  //Serial.print(SmoothDistance);
  if (sensor.timeoutOccurred()) { Serial.println(" TIMEOUT"); }
  //Serial.println();

  //add variable dealing with distance 
  distance = sensor.readRangeSingleMillimeters();
  delay(20);

  //smooth the distance readings
  SmoothDistance = as15.smooth(distance);
  
  if(Button1 == HIGH) {
    LEDFadeIN(0,219,77,100,75); //LEDnumber, Hue, Sat, Value, (use normal color picker, range is 0-360, 0-100, 0-100) FadeSpeed(higher is faster)
    //workingFadeOUTCycle[0] = 255; //re renable the fade out cycle
  }else{
    LEDFadeOUT(0,219,77,100,75); //specify the color we want to fade to, in 0-255 format
    //workingFadeINCycle[0] = 0; //re enable the fade in cycle
  }


  //MQTT get established on the connection first
  if (!client.connected()) {
      reconnect();
      }
    client.loop();

  //MQTT publish message
  long now = millis();
   if (now - lastMsg > 2000) {
    lastMsg = now;
    ++value;
    snprintf (msg, 75, "hello world #%ld", value);
    Serial.print("Publish message: ");
    Serial.println(msg);
    client.publish("outTopic", msg);
}
}

void LEDFadeIN(int workingLEDNumber, int workingH, int workingS, int workingV, int workingFadeSpeed){
  workingH = map(workingH, 0, 360, 0, 255); //map H value to 0-360 so that expressions going in can be from a normal color picker
  workingS = map(workingS, 0, 100, 0, 255);
  workingV = map(workingV, 0, 100, 0, 255);
  int workingEndSpeed = 255- workingFadeSpeed;
  if(workingFadeCycle[workingLEDNumber] < workingEndSpeed)
  {
    workingFadeCycle[workingLEDNumber] = workingFadeCycle[workingLEDNumber] + workingFadeSpeed; //adjust this number for speed of gain
    leds[workingLEDNumber] = CHSV(workingH,workingS,workingFadeCycle[workingLEDNumber]);
    Serial.println((String)"in loop 2 " + workingFadeCycle[workingLEDNumber]);
  }
  else
  {
    leds[workingLEDNumber] = CHSV (workingH, workingS, 255);
  }
  FastLED.show();
}

void LEDFadeOUT(int workingLEDNumber, int workingH, int workingS, int workingV, int workingFadeSpeed){
  workingH = map(workingH, 0, 360, 0, 255); //map H value to 0-360 so that expressions going in can be from a normal color picker
  workingS = map(workingS, 0, 100, 0, 255);
  workingV = map(workingV, 0, 100, 0, 255);  
  int workingEndSpeed = 0 + workingFadeSpeed;
  if(workingFadeCycle[workingLEDNumber] >= workingEndSpeed)
  {
    workingFadeCycle[workingLEDNumber] = workingFadeCycle[workingLEDNumber] - workingFadeSpeed; //adjust this number for speed of gain
    leds[workingLEDNumber] = CHSV(workingH,workingS,workingFadeCycle[workingLEDNumber]);
    Serial.println((String)"in loop 3 " + workingFadeCycle[workingLEDNumber]);
  }
  else
  {
    leds[workingLEDNumber] = CHSV (workingH, workingS, 0);
  }
  FastLED.show();
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, wifipassword);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  WifiOnline = 1;
}

void reconnect() {
  // Loop until we're reconnected
  if(!client.connected()&& WifiAttempts <= WifiMaxAttempts) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected"); //once connected set connected status to 1
      connectionStatus = "1";
      client.publish(ConnectedTopic, connectionStatus); //reconnect to all the topics we need to
      client.subscribe(ConnectedTopic);
      client.subscribe(HeightTopic);
      client.subscribe(CommandedHeightTopic);
      client.subscribe(ErrorTopic);
      client.subscribe(ExecuteTopic);
    } else {
      WifiAttempts ++;
      connectionStatus = 0;
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(1000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}
