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
int value = 0;
String username = "homeassistant";
String password = "";

//MQTT topics and payloads
char* ConnectedTopic = "/desk/connected";
char* HeightTopic = "/desk/actualheight";
char* CommandedHeightTopic = "/desk/commandedheight";
char* ErrorTopic = "/desk/error";
char* ExecuteTopic = "/desk/execute";
int ConnectedStatus = 0;
int Height = 0;
int HeightCommanded = 0;
int ErrorCode = 0;
int ExecuteFlag = 0;


//Lidar initialization
VL53L0X sensor;
VL53L0X sensor2;
int Lidar1ShutdownPin = D3;
int Lidar1Shutdown = 1; //must be 1 to read sensors. In conjuction with timeout being at 0
int LidarTimeOut = 0; //global lidar timeout used in function call to setup

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
int distance1 = 0;
int distance2 = 0; //second lidar distance reading
int mmOutOfLevel = 0;
int LEDsToOn = 0;

//Global input variables
int Button1 = 0;
int Test1 = 0;

//Global timing variables
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
int workingFadeCycle[] = {0,0,0,0}; //declare array for working timing

//Smoothing setup
int SmoothDistance1 = 0;
int SmoothDistance2 = 0;
int AverageDistance = 0;
AnalogSmooth as15 = AnalogSmooth(15);

//
void setup()
{
  Serial.begin(9600);

  //Lidar initialize
  


  //pin mode setup
  pinMode(SW1Pin, INPUT);
  pinMode(Lidar1ShutdownPin, OUTPUT);
  digitalWrite(Lidar1ShutdownPin, Lidar1Shutdown); 
  // ms (e.g. sensor.startContinuous(100)).
  //sensor.startContinuous();

  //Initialized LED array
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.clear(); //clear all LEDs before we start too much
  FastLED.show();
  delay(5);

  //setup wifi and connect to mqtt
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  SetupTwinLidars();
  //Serial.println("Setup Done");
}

void loop()
{

  //Global timing functions
  currentMillis = millis();
  
  //read the analog and digital values
  Button1 = digitalRead(SW1Pin);
  
  //Lidar Serial Reporting
  ReadDistance();
  Serial.println((String)"Smooth Average Distance : " + AverageDistance);
  
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
  if(currentMillis - previousMillis >= 5000) {
    sendCommandedHeightMessage(AverageDistance);
    previousMillis = currentMillis;
   }else {
    //sendConnectMessage(ConnectedStatus);
   }
   
   //sendCommandedHeightMessage(100);
   //Serial.println(millis());
   //sendHeightMessage(AverageDistance);
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
      ConnectedStatus = 1;
      //client.publish(ConnectedTopic, connectionStatus); //reconnect to all the topics we need to
      client.subscribe(ConnectedTopic);
      client.subscribe(HeightTopic);
      client.subscribe(CommandedHeightTopic);
      client.subscribe(ErrorTopic);
      client.subscribe(ExecuteTopic);
    } else {
      WifiAttempts ++;
      ConnectedStatus = 0;
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(1000);
    }
  }
}

void SetupTwinLidars () {
  Lidar1Shutdown = 0;
  digitalWrite(Lidar1ShutdownPin, Lidar1Shutdown);
  delay(100);
  Wire.begin();
  sensor2.init(true);
  sensor2.setAddress((uint8_t)26);
  sensor2.setTimeout(LidarTimeOut); //0 seems to work best here although units are not fully understood
  delay(100);
  Lidar1Shutdown = 1;
  digitalWrite(Lidar1ShutdownPin, Lidar1Shutdown);
  sensor.init(true);
  sensor.setTimeout(LidarTimeOut); //0 seems to work best here although units are not fully understood
  Serial.println("Sensor changeover complete");
}

void ReadDistance() {
  if(Lidar1Shutdown == 1  || sensor.timeoutOccurred() || sensor2.timeoutOccurred()){    
    //Serial.println(" TIMEOUT");
    ErrorCode = 1;
    sendErrorMessage(ErrorCode);
    }    
    if(ErrorCode == 0){ //ensure we don't reboot the whole shebang due to error codes on the I2C bus
    distance1 = sensor.readRangeSingleMillimeters(); //must be done in conjunction with the pin goign high or low, will  cause bizzare boot error if sensor is off and trying to read
    distance2 = sensor2.readRangeSingleMillimeters();
    }
    //Serial.println((String)"Sensor1: " + distance1);
    //Serial.println((String)"Sensor2: " + distance2);
  //smooth the distance readings
  SmoothDistance1 = as15.smooth(distance1);
  SmoothDistance2 = as15.smooth(distance2);
  AverageDistance = (SmoothDistance1 + SmoothDistance2)/2;
  //calculate the offset between sensors
  if(SmoothDistance1 >= SmoothDistance2){
    mmOutOfLevel = SmoothDistance1 - SmoothDistance2;
    }else{
    mmOutOfLevel = SmoothDistance2 - SmoothDistance1; 
    }
  Serial.println((String) "Out of Level: " + mmOutOfLevel);
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

void sendConnectMessage(int workingConnectedPayload){ 
   char workingPayload[50];
   snprintf (workingPayload, 100, "%d", workingConnectedPayload);
   Serial.print("Sending Message: ");
   Serial.println((String)"Connected: " + workingPayload);
   client.publish("/desk/connected", workingPayload);
  }

void sendHeightMessage(int workingHeightPayload){ 
   char workingPayload[50];
   snprintf (workingPayload, 100, "%d", workingHeightPayload);
   Serial.print("Sending Message: ");
   Serial.println((String)"Height: " + workingPayload);
   client.publish("/desk/height", workingPayload);
  }

void sendCommandedHeightMessage(int workingCommandedHeightPayload){ 
   char workingPayload[50];
   snprintf (workingPayload, 100, "%d", workingCommandedHeightPayload);
   Serial.print("Sending Message: ");
   Serial.println((String)"Commanded Height: " + workingPayload);
   client.publish("/desk/commandedheight", workingPayload);
  }

void sendErrorMessage(int workingErrorPayload){ 
   char workingPayload[50];
   snprintf (workingPayload, 100, "%d", workingErrorPayload);
   Serial.print("Sending Message: ");
   Serial.println((String)"Error: " + workingPayload);
   client.publish("/desk/error", workingPayload);
  }

void sendExecuteMessage(int workingExecutePayload){ 
   char workingPayload[50];
   snprintf (workingPayload, 100, "%d", workingExecutePayload);
   Serial.print("Sending Message: ");
   Serial.println((String)"Execute: " + workingPayload);
   client.publish("/desk/execute", workingPayload);
  }
