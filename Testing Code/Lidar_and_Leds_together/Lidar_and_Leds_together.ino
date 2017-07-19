#include "FastLED.h"

// How many leds in your strip?
#define NUM_LEDS 44 

// For led chips like Neopixels, which have a data line, ground, and power, you just
// need to define DATA_PIN.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the LPD8806, define both DATA_PIN and CLOCK_PIN
#define DATA_PIN 6
//#define CLOCK_PIN 13

// Define the array of leds
CRGB leds[NUM_LEDS];
<<<<<<< HEAD
=======
#define UPDATES_PER_SECOND 100
#define FASTLED_ALLOW_INTERRUPTS 0

//Pin setup
#define SW1Pin D5

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
#define MotorUpPin D7
#define MotorDownPin D8
int MotorRunning = 0;
int NewMotorData = 0;

//Global timing variables
unsigned long Timer1 = 0;
unsigned long Timer2 = 0;
unsigned long Timer3 = 0;
unsigned long Timer4 = 0;
unsigned long currentMillis = 0;
int workingFadeCycle[] = {0,0,0,0}; //declare array for working timing

//Smoothing setup
int SmoothDistance1 = 0;
int SmoothDistance2 = 0;
int AverageDistance = 0;
int LastDistanceShot = 0;
int DistanceDownPosition = 25;
int DistanceUpPosition = 1000;
AnalogSmooth SmoothSensor1 = AnalogSmooth(15);
AnalogSmooth SmoothSensor2 = AnalogSmooth(15);

/////////////////////////////////// Main Setup
void setup(){
  
  Serial.begin(250000);

  //pin mode setup
  pinMode(SW1Pin, INPUT);
  pinMode(Lidar1ShutdownPin, OUTPUT);
  digitalWrite(Lidar1ShutdownPin, Lidar1Shutdown); 
  
  //Initialized LED array
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.clear(); //clear all LEDs before we start too much
  FastLED.show();
  delay(5);

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
  Serial.println("Setup Done");
}

////////////////////////////////////// Main Loop
void loop() {
  
  //Global timing functions
  currentMillis = millis();
  
  //read the analog and digital values
  Button1 = digitalRead(SW1Pin);
  
  //enable distance sensing when buttons are pressed and keep on for 5 seconds for good measure
  if(Button1 == 1 || Button2 == 1 || Button3 == 1 || Button4 == 1){
    Timer2 = currentMillis;
    }
  if(currentMillis - Timer2 >= SuspendInterval){
    SensorSleep = 1;
    //Serial.println((String)"in the loop: State: " + SensorSleep);
    }else{
    SensorSleep = 0;
    Serial.println((String)"out of loop: State: " + SensorSleep);
    }
    
  //Send motor on time packet over mqtt only when there's not a ton else to do)
  if(SensorSleep == 1 && NewMotorData == 1) {
    sendTimeOnCount();
    LastDistanceShot = AverageDistance; //stores the last value for comparison
    sendHeightMessage(LastDistanceShot);
  }
  
  //Lidar Read Distance
  if(SensorSleep == 0){
    ReadDistance();
    Serial.println((String)"reading distances: " + AverageDistance);
    }
  
  //MQTT Check for connection, else 
  if (!client.connected()) {
      reconnect();
      }
      client.loop();
      
  //Run the motor UP on button press and check for tilt angles otherwise thrown and error and flash the lights
  if(Button1 == HIGH && newCommandReady == 0) {
    MotorUp();
    LEDFadeIN(0,219,77,100,75); //LEDnumber, Hue, Sat, Value, (use normal color picker, range is 0-360, 0-100, 0-100) FadeSpeed(higher is faster)
    Serial.println("motor running up on button");
    }
    if(Button1 == LOW && newCommandReady == 0) {
    LEDFadeOUT(0,219,77,100,75); //specify the color we want to fade to, in 0-255 format
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
  if(!client.connected()&& ConnectionTries <= ConnectionRetries) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-"; 
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
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
  //if(mmOutOfLevel <= AllowableTilt && HydPressure < HydPressureLimit && AverageDistance > DistanceDownPosition && AverageDistance < DistanceUpPosition){
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
  //if(mmOutOfLevel <= AllowableTilt && HydPressure < HydPressureLimit && AverageDistance > DistanceDownPosition && AverageDistance < DistanceUpPosition){
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
    MotorSecondsOnCount = (currentMillis - MotorTempOnCount);
    MotorSecondsOnCount = MotorSecondsOnCount / 1000;
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
 
void callback(String topic, byte* payload, unsigned int length) {
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
>>>>>>> parent of a4d8f17... implement out of bounds stops for commanded

void setup() { 
  Serial.begin(57600);
  Serial.println("resetting");
  LEDS.addLeds<WS2812,DATA_PIN,RGB>(leds,NUM_LEDS);
  LEDS.setBrightness(84);
}

void fadeall() { for(int i = 0; i < NUM_LEDS; i++) { leds[i].nscale8(250); } }

void loop() { 
  static uint8_t hue = 0;
  Serial.print("x");
  // First slide the led in one direction
  for(int i = 0; i < NUM_LEDS; i++) {
    // Set the i'th led to red 
    leds[i] = CHSV(hue++, 255, 255);
    // Show the leds
    FastLED.show(); 
    // now that we've shown the leds, reset the i'th led to black
    // leds[i] = CRGB::Black;
    fadeall();
    // Wait a little bit before we loop around and do it again
    delay(10);
  }
  Serial.print("x");

  // Now go in the other direction.  
  for(int i = (NUM_LEDS)-1; i >= 0; i--) {
    // Set the i'th led to red 
    leds[i] = CHSV(hue++, 255, 255);
    // Show the leds
    FastLED.show();
    // now that we've shown the leds, reset the i'th led to black
    // leds[i] = CRGB::Black;
    fadeall();
    // Wait a little bit before we loop around and do it again
    delay(10);
  }
}
