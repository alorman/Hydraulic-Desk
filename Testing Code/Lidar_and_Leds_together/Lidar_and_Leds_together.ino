/* This example shows how to use continuous mode to take
range measurements with the VL53L0X. It is based on
vl53l0x_ContinuousRanging_Example.c from the VL53L0X API.
The range readings are in units of mm. */
//#define FASTLED_ESP8266_NODEMCU_PIN_ORDER
#include <Wire.h>
#include <VL53L0X.h>
#include <NeoPixelBus.h>
#include <AnalogSmooth.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Preferences.h>

//setup preferences for esp32
Preferences preferences;

//wifi Initiation
const char* ssid = "nestlink";
const char* wifipassword = "nestlink";
const char* mqtt_server = "192.168.0.127";
int WifiTimeout = 5; //how many times to try to reconnect. roughly .5x in seconds
int WifiOnline = 0;

//setup pubsub
WiFiClient espClient;
PubSubClient client (espClient);
String clientId = "MagicalDesk-"; //in connect is appended with random follower
String MQTTusername = "pi";
String MQTTpassword = "raspberry";

//MQTT topics and payloads
char* ConnectedTopic = "/desk/connected";
char* HeightTopic = "/desk/height";
char* CommandedHeightTopic = "/desk/commandedheight";
char* ErrorTopic = "/desk/error";
char* ExecuteTopic = "/desk/execute";
int ConnectedStatus = 0;
int Height = 0;
float HeightCommanded = 0.0;
int TargetHeightTolerance = 1;
int ErrorCode = 0;
int ExecuteFlag = 0;
int ConnectionTries = 0;
int ConnectionRetries = 5;
int RetryInterval = 10000; 
int MQTTStatusFlag = 0; //variable to track if we should send mqtt message, preferably only once. 

//Lidar initialization
VL53L0X sensor;
VL53L0X sensor2;
int Lidar1ShutdownPin = 16;
int Lidar1Shutdown = 1; //must be 1 to read sensors. In conjuction with timeout being at 0
int LidarTimeOut = 0; //global lidar timeout used in function call to setup

//LED options
#define LED_PIN     21
#define NUM_LEDS    4
#define BRIGHTNESS  255
#define LED_FEATURE NeoGrbFeature
#define LED_METHOD Neo800KbpsMethod
NeoPixelBus<LED_FEATURE, LED_METHOD> strip(NUM_LEDS, LED_PIN);

//Pin setup
#define SW1Pin 15
#define SW2Pin 2
#define SW3Pin 0

//Global scripting variables
int distance1 = 0;
int distance2 = 0; //second lidar distance reading
int mmOutOfLevel = 0;
int MotorEnable = 0;
int LEDsToOn = 0;
int AllowableTilt = 55; //maximum allowable tilt in mm
int SensorSleep = 0; //flag to stop firing laser pings all the time
int SuspendInterval = 5000; //mS
unsigned long MotorSecondsOnCount = 0;
unsigned long MotorMSecondsOnCount = 0;
unsigned long MotorTempOnCount = 0;
int newCommandReady = 0;
float PreviousDistance = 0;

//Global input variables
int Button1 = 0;
int Button2 = 0;
int Button3 = 0;
int Button4 = 0;
int HydPressure = 0;
int HydPressureLimit = 250; //PSI

//Global Output variables
#define MotorUpPin 7
#define MotorDownPin 8
int MotorRunning = 0;
int NewMotorData = 0;

//Global timing variables
unsigned long Timer1 = 0;
unsigned long Timer2 = 0;
unsigned long Timer3 = 0;
unsigned long Timer4 = 0;
unsigned long Timer5 = 0;
unsigned long currentMillis = 0;
float workingFadeCycle[] = {0.0,0.0,0.0,0.0}; //declare array for working timing

//Smoothing setup
int SmoothDistance1 = 0;
int SmoothDistance2 = 0;
int AverageDistance = 0;
int LastDistanceShot = 0;
float DistanceDownStop = 24.0; //inches, float. Include padded zero
float DistanceUpStop = 58.0; //inches, float. Include padded zero
AnalogSmooth SmoothSensor1 = AnalogSmooth(15);
AnalogSmooth SmoothSensor2 = AnalogSmooth(15);

/////////////////////////////////// Main Setup
void setup(){
  
  Serial.begin(115200);

  //pin mode setup
  pinMode(SW1Pin, INPUT);
  pinMode(Lidar1ShutdownPin, OUTPUT);
  digitalWrite(Lidar1ShutdownPin, Lidar1Shutdown); 

  //setup wifi
  setup_wifi();

  //connect to mqtt
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  //Read the seconds motor has run in eeprom
  eepromReadSeconds();
  //eepromClear();
  
  //Setup Lidars
  SetupTwinLidars();
  
  //Clear the LED array (moved down here to go after any of the blocking setup commands)
  strip.Begin();
  strip.Show();
  Serial.println("Setup Done");
}

////////////////////////////////////// Main Loop
void loop() {

  strip.Begin();
  strip.Show();
  
  //Global timing functions
  currentMillis = millis();
  
  //read the analog and digital values
  Button1 = digitalRead(SW1Pin);
  Button2 = digitalRead(SW2Pin);
  Button3 = digitalRead(SW3Pin);
  
  //enable distance sensing when buttons are pressed and keep on for 5 seconds for good measure
  if(Button1 == 1 || Button2 == 1){
    Timer2 = currentMillis;
    }
  if(currentMillis - Timer2 >= SuspendInterval){
    SensorSleep = 1;
    //Serial.println((String)"in the loop: State: " + SensorSleep);
    }else{
    SensorSleep = 0;
    Serial.println((String)"out of loop: State: " + SensorSleep);
    Serial.println((String)"current time: " + currentMillis + " Timer 2: " + Timer2);
    }

  //reconnect the mqtt if wifi comes back 
  if(currentMillis - Timer5 >= RetryInterval){
    Timer5 = currentMillis;
    if(WiFi.status() == WL_CONNECTED && !client.connected()){
      ConnectionRetries = 1;
      ConnectionTries = 0;
      Serial.println("reconnecting to MQTT after wifi loss");
      client.setServer(mqtt_server, 1883);
     }
   }

  //Send motor on time packet over mqtt only when there's not a ton else to do)
  
  if(SensorSleep == 1 && MQTTStatusFlag == 1) {
    sendTimeOnCount();
    LastDistanceShot = AverageDistance; //stores the last value for comparison
    sendHeightMessage(LastDistanceShot);
    MQTTStatusFlag = 0;
  }
  
  //Lidar Read Distance
  if(SensorSleep == 0){
    ReadDistance();
    Serial.println((String)"reading distances: " + AverageDistance);
    MQTTStatusFlag = 1;
    }else{
    MQTTStatusFlag = 0;
    }
  
  //MQTT Check for connection, else 
  if (!client.connected()) {
      MQTTreconnect();
      }
      client.loop();
      
  //Run the motor UP on button press and check for tilt angles otherwise thrown and error and flash the lights
  if(Button1 == HIGH && newCommandReady == 0) {
    MotorUp();
    LEDFadeIN(0,219,77,100,0.05); //LEDnumber, Hue, Sat, Value, (use normal color picker, range is 0-360, 0-100, 0-100) FadeSpeed(higher is faster)
    Serial.println("motor running up on button");
    }
    if(Button1 == LOW && newCommandReady == 0) {
    LEDFadeOUT(0,219,77,100,0.1); //specify the color we want to fade to, in 0-255 format
    MotorAllStop();
    //Serial.println("motor stopping on key not pressed");
    }

    /* //This is having issues due to resetting button 1 and motor controllers stuff rewrtie logic
  //Run the motor Down on button press and check the above in function
  if(Button2 == HIGH) {
    MotorDown();
    LEDFadeIN(1,219,77,100,75); //LEDnumber, Hue, Sat, Value, (use normal color picker, range is 0-360, 0-100, 0-100) FadeSpeed(higher is faster)
    }
    if(Button2 == LOW) {
    LEDFadeOUT(1,219,77,100,75); //specify the color we want to fade to, in 0-255 format
    MotorAllStop();
    }
*/
  if(newCommandReady == 1){
    MotorToCommandedHeight(HeightCommanded);
    }
  
  if(currentMillis - Timer1 >= 5000) {
    //sendCommandedHeightMessage(AverageDistance);
    Timer1 = currentMillis;
   }else {
    //sendConnectMessage(ConnectedStatus);
   }
   
   //sendCommandedHeightMessage(100);
   //Serial.println(millis());
   //sendHeightMessage(AverageDistance);
}

/////////////////////////////////////////Functions

void LEDFadeIN(int workingLEDNumber, int workingH, int workingS, int workingV, float workingFadeSpeed){
  strip.Begin();
  strip.Show();
  float workingHFloat = 0.0;
  float workingSFloat = 0.0;
  float workingVFloat = 0.0;
  workingHFloat = (float)workingH/(float)360.0; //remap the color picker to the stupid -0.0 to 1.0 values this requires
  workingSFloat = (float)workingS/(float)100.0;
  workingVFloat = (float)workingV/(float)100.0;
  float workingEndSpeed = 1.0 - workingFadeSpeed;
  if(workingFadeCycle[workingLEDNumber] < workingEndSpeed)
  {
    workingFadeCycle[workingLEDNumber] = workingFadeCycle[workingLEDNumber] + workingFadeSpeed; //adjust this number for speed of gain
    HslColor workingHSL(workingHFloat, workingSFloat, workingFadeCycle[workingLEDNumber]);
    strip.SetPixelColor(workingLEDNumber, workingHSL);
    Serial.println((String)"in fade in loop :" + workingHFloat + "," + workingSFloat + "," + workingFadeCycle[workingLEDNumber]);
  }
  else
  {
    HslColor workingHSL2(workingHFloat, workingSFloat, 1.0);
    strip.SetPixelColor(workingLEDNumber, workingHSL2);  
  }
  strip.Show();
  delay(10);
}

void LEDFadeOUT(int workingLEDNumber, int workingH, int workingS, int workingV, float workingFadeSpeed){
  float workingHFloat = 0.0;
  float workingSFloat = 0.0;
  float workingVFloat = 0.0;
  workingHFloat = (float)workingH/(float)360.0; //remap the color picker to the stupid -0.0 to 1.0 values this requires
  workingSFloat = (float)workingS/(float)100.0;
  workingVFloat = (float)workingV/(float)100.0;
  float workingEndSpeed = 0.0 + workingFadeSpeed;
  if(workingFadeCycle[workingLEDNumber] >= workingEndSpeed){
    workingFadeCycle[workingLEDNumber] = workingFadeCycle[workingLEDNumber] - workingFadeSpeed;//adjust this number for speed of gain
    HslColor workingHSL(workingHFloat, workingSFloat, workingFadeCycle[workingLEDNumber]);
    strip.SetPixelColor(workingLEDNumber, workingHSL);
    Serial.println((String)"in fade out loop " + workingFadeCycle[workingLEDNumber]);
  }else{
    HslColor workingHSL2(workingHFloat, workingSFloat, 0.0);
    strip.SetPixelColor(workingLEDNumber, workingHSL2);  
  }
  //Serial.println("Executing fade out loop");
  strip.Show();
  delay(10);
}

void setup_wifi() {
  int workingWifiTries = 0;
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, wifipassword);

  while (WiFi.status() != WL_CONNECTED && workingWifiTries < WifiTimeout ) {
    delay(500);
    Serial.print(".");
    workingWifiTries++;
    WifiOnline = 0;
  }
  if(WiFi.status() == WL_CONNECTED){
  randomSeed(micros());
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  WifiOnline = 1;
  }
}

void MQTTreconnect() {
  // Loop until we're reconnected
  if(!client.connected()&& ConnectionTries < ConnectionRetries) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID 
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), MQTTusername.c_str(), MQTTpassword.c_str())) {
      Serial.println("connected"); //once connected set connected status to 1
      ConnectedStatus = 1;
      //client.publish(ConnectedTopic, connectionStatus); //reconnect to all the topics we need to
      //client.subscribe(ConnectedTopic);
      //client.subscribe(HeightTopic);
      client.subscribe(CommandedHeightTopic);
      //client.subscribe(ErrorTopic);
      client.subscribe(ExecuteTopic);
    } else {
      ConnectionTries ++;
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
  Wire.begin(23,22);
  sensor2.init(true);
  sensor2.setAddress((uint8_t)26);
  sensor2.setTimeout(LidarTimeOut); //0 seems to work best here although units are not fully understood
  delay(100);
  Lidar1Shutdown = 1;
  digitalWrite(Lidar1ShutdownPin, Lidar1Shutdown);
  sensor.init(true);
  sensor.setTimeout(LidarTimeOut); //0 seems to work best here although units are not fully understood
  Serial.println("Sensor changeover complete");
  delay(100);
}

void ReadDistance() {
  if(Lidar1Shutdown == 0  || sensor.timeoutOccurred() || sensor2.timeoutOccurred()){    
    //Serial.println(" TIMEOUT");
    ErrorCode = 1;
    sendErrorMessage(ErrorCode);
    Serial.println(Lidar1Shutdown);
    }else{
    ErrorCode = 0;    
    }
  if(ErrorCode == 0){ //ensure we don't reboot the whole shebang due to error codes on the I2C bus
  distance1 = sensor.readRangeSingleMillimeters(); //must be done in conjunction with the pin goign high or low, will  cause bizzare boot error if sensor is off and trying to read
  distance2 = sensor2.readRangeSingleMillimeters();
  }
    //Serial.println((String)"Sensor1: " + distance1);
    //Serial.println((String)"Sensor2: " + distance2);
  //smooth the distance readings
  SmoothDistance1 = SmoothSensor1.smooth(distance1);
  SmoothDistance2 = SmoothSensor2.smooth(distance2);
  AverageDistance = (SmoothDistance1 + SmoothDistance2)/2;
  //calculate the offset between sensors
  if(SmoothDistance1 >= SmoothDistance2){
    mmOutOfLevel = SmoothDistance1 - SmoothDistance2;
    }else{
    mmOutOfLevel = SmoothDistance2 - SmoothDistance1; 
    }
  Serial.println((String) "Out of Level: " + mmOutOfLevel);
}

void MotorUp(){
  //if(mmOutOfLevel <= AllowableTilt && HydPressure < HydPressureLimit && AverageDistance > DistanceDownStop && AverageDistance < DistanceUpStop){
    Timer2 = currentMillis; //reset sensing suspend clock
      if(MotorRunning == 0){ //stop syncing the motor on timer 
         MotorTempOnCount = currentMillis;
         }
    MotorRunning = 1;
    NewMotorData = 1;
    Serial.println(MotorTempOnCount);
    digitalWrite(MotorUpPin, HIGH);
    Serial.println("motor running up");
    Serial.println((String)"Motor State Variable: " + MotorRunning);
  /*}else{
    MotorRunning = 0;
    ErrorCode = 3;
    sendErrorMessage(ErrorCode);
    MotorSecondsOnCount = (currentMillis - MotorTempOnCount)/ 1000;
    MotorTempOnCount = 0;
    digitalWrite(MotorUpPin, LOW); 
    }
    */
}

void MotorDown(){
  //if(mmOutOfLevel <= AllowableTilt && HydPressure < HydPressureLimit && AverageDistance > DistanceDownStop && AverageDistance < DistanceUpStop){
    Timer2 = currentMillis; //reset sensing suspend clock
      if(MotorRunning == 0) { // stop syncing the motor on timer
         MotorTempOnCount = currentMillis;
         }
    MotorRunning = 1;
    NewMotorData = 1;
    digitalWrite(MotorDownPin, HIGH);
    Serial.println("motor running down");
 /* }else{
    MotorRunning = 0;
    ErrorCode = 4;
    sendErrorMessage(ErrorCode);
    MotorSecondsOnCount = (currentMillis - MotorTempOnCount)/ 1000;
    MotorTempOnCount = 0;
    digitalWrite(MotorDownPin, LOW); 
    }
    */
}

void MotorAllStop(){
  digitalWrite(MotorDownPin, LOW);
  digitalWrite(MotorUpPin, LOW);
  if(MotorRunning == 1) {
    MotorMSecondsOnCount = (currentMillis - MotorTempOnCount);
    MotorSecondsOnCount = MotorMSecondsOnCount / 1000;
    eepromWriteSeconds();
    }
  //MotorTempOnCount = 0;
  MotorRunning = 0;
  NewMotorData = 0;
}

void MotorToCommandedHeight(float workingCommandedHeight){
  float workingHeightChangeLimit = 20.0; //must be with padded zero due to floating
  int workingMotorCheckInterval = 2000; //milliseconds
  int workingCurrentMillis = millis();
  float workingHeightUpper = AverageDistance + TargetHeightTolerance;
  float workingHeightLower = AverageDistance - TargetHeightTolerance;
  float workingPreviousDistance = AverageDistance;
  if(workingCommandedHeight <= DistanceDownStop || workingCommandedHeight >= DistanceUpStop){
    sendErrorMessage(5);
    newCommandReady = 0;
    Serial.println("Commanded Out of Bounds");
  }else{
  if(workingCommandedHeight >= AverageDistance){
  Timer2 = currentMillis; //reset sensing suspend clock 
  Serial.println("ordered up, going up");
  MotorUp();
  }else if(workingCommandedHeight <= AverageDistance){
  Timer2 = currentMillis; //reset sensing suspend clock 
  Serial.println("ordered down, going down");
  MotorDown(); 
  }
 if(PreviousDistance == 0){
  PreviousDistance = AverageDistance;
  Timer3 = workingCurrentMillis;
  }
 if(workingCurrentMillis - Timer3 >= workingMotorCheckInterval) {
  Serial.println("in command height timing loop");
  Serial.println((String)"Timing bits// workingCurrent: " + workingCurrentMillis + " Timer3: " + Timer3);
  Timer3 = workingCurrentMillis;
   if(AverageDistance <= (AverageDistance + workingHeightChangeLimit)||AverageDistance >= (AverageDistance - workingHeightChangeLimit)){
     Serial.println("Motor didn't move enough, stopping");
     sendErrorMessage(6);
     MotorAllStop();
     newCommandReady = 0;
    }
   PreviousDistance = 0;
  }
 if(workingHeightUpper >= workingCommandedHeight && workingHeightLower <= workingCommandedHeight){
  Serial.println("good enough, stoppping here");
  MotorAllStop();
  newCommandReady = 0;
  }
 }
} 
 
void callback(char* topic, byte* payload, unsigned int length) {
  String workingCommandedHeightString = "0";
  int i = 0;
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (i; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  if(topic == "/desk/commandedheight"){
    workingCommandedHeightString = String((char*)payload);
    HeightCommanded = workingCommandedHeightString.toFloat();
    Serial.print("height commanded: ");
    Serial.println(HeightCommanded, 1);
    newCommandReady = 1;
   }
  if(topic == "/desk/execute"){
    
   }
}

void sendConnectMessage(int workingConnectedPayload){ 
   char workingPayload[50];
   snprintf (workingPayload, 100, "%d", workingConnectedPayload);
   Serial.print("Sending Message: ");
   Serial.println((String)"Connected: " + workingPayload);
   client.publish("/desk/connected", workingPayload);
  }

void sendHeightMessage(int workingHeightPayload){ 
   char workingPayload[100];
   float workingHeightPayloadFloat;
   workingHeightPayloadFloat = workingHeightPayload/25.4;
   dtostrf(workingHeightPayloadFloat, 3, 1, workingPayload);
   Serial.println((String)"float payload " + workingHeightPayloadFloat);
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

void sendTimeOnCount(){
  long tempMinutesOnCount = 0;
  char workingPayload[100];
  tempMinutesOnCount = MotorSecondsOnCount / 60;
  snprintf (workingPayload, 100, "%ld", tempMinutesOnCount);
  NewMotorData = 0;
  Serial.print("Sending Message: ");
  Serial.println((String)"Minutes On Count: " + tempMinutesOnCount);
  Serial.println((String)"Seconds on count: " + MotorSecondsOnCount);
  client.publish("/desk/MinutesOn", workingPayload);
}

void eepromWriteSeconds(){
  //setup the preferences namespace for esp-32
  preferences.begin("MotorLife", false);
  unsigned long tempEEPROMread = 0;
  unsigned long tempEEPROMtoWrite = 0;
  tempEEPROMread = preferences.getUInt("SecondsOnCount", 0);
  Serial.println((String)"Previous EEPROM: "+ tempEEPROMread);
  MotorSecondsOnCount = tempEEPROMread + MotorSecondsOnCount;
  preferences.putUInt("SecondsOnCount", MotorSecondsOnCount);
  preferences.end();
  Serial.println((String)"EEPROM write :" + MotorSecondsOnCount);
}

void eepromClear(){
  //setup the preferences namespace for esp-32
  preferences.begin("MotorLife", false);
  preferences.remove("SecondsOnCount");
  Serial.println("EEPROM Cleared ...........");
}
void eepromReadSeconds(){
  //setup the preferences namespace for esp-32
  preferences.begin("MotorLife", false);
  MotorSecondsOnCount = preferences.getUInt("SecondsOnCount", 0);
  Serial.println((String)"EEPROM read :" + MotorSecondsOnCount);
}
