// Simple test script to run hydraulic pump up and down
// Bang Bang-only control implemented

// Global variables
  //Output pin definitions
  int MotorUpPin = 0;
  int MotorDownPin = 5;
  int MotorEnable = 4;
  //Input pins
  int PushbuttonPin = 13;
  int SwitchDirUp = 14;
  int SwitchDirDown = 12;

  //Control variables
  int MotorUpSwitch = 0;
  int MotorDownSwitch = 0;
  int PushbuttonPushed = 0;

  //output variables
  int MotorOutUp = 0;
  int MotorOutDown = 0;
  int EnableOut = 0;

  //Cycle counter
  int CycleCount = 0;

//Setup loop
void setup() {
  //Pin Modes
  pinMode(MotorUpPin, OUTPUT);
  pinMode(MotorDownPin, OUTPUT);
  pinMode(MotorEnable, OUTPUT);
  pinMode(PushbuttonPin, INPUT);
  pinMode(SwitchDirUp, INPUT);
  pinMode(SwitchDirDown, INPUT);

  //Setup serial 
  Serial.begin(9600);
}
//End Setup loop

//Main Loop
void loop() {
  
  //read inputs first
  ReadInputs();
  //now run control
  ControlLogic();
  //now output control
  OutputControl();
  //Serial.println("Main loop");
}
//End main loop

void ReadInputs() {
 MotorUpSwitch = digitalRead(SwitchDirUp);
 MotorDownSwitch = digitalRead(SwitchDirDown);
 PushbuttonPushed = digitalRead(PushbuttonPin); 
}

void ControlLogic() {
  if(MotorDownSwitch == LOW && PushbuttonPushed == LOW)
  {
    MotorOutDown = 0;
    MotorOutUp = 1;
    EnableOut = 1;
    //Serial.println("Break 1");
  }
  else if(MotorUpSwitch == LOW &&PushbuttonPushed == LOW)
  {
    MotorOutDown = 1;
    MotorOutUp = 0;
    EnableOut = 1;
    //Serial.println("Break 2");
  }
  if(PushbuttonPushed == HIGH)
  {
    MotorOutUp = 0;
    MotorOutDown = 0;
    EnableOut = 0;
    //Serial.println("Break 3");
  }
}

void OutputControl() {
  if (MotorOutDown == 1)
  {
  digitalWrite(MotorUpPin, HIGH);
  digitalWrite(MotorDownPin, LOW);
  digitalWrite(EnableOut, HIGH);
  Serial.println("Break 5");
  }
  if (MotorOutUp == 1)
  {
  digitalWrite(MotorUpPin, LOW);
  digitalWrite(MotorDownPin, HIGH);
  digitalWrite(EnableOut, HIGH);
  Serial.println("Break 7");
  }
  if (MotorOutUp == 0 && MotorOutDown == 0)
  {
  digitalWrite(MotorUpPin, LOW);
  digitalWrite(MotorDownPin, LOW);
  digitalWrite(EnableOut, LOW);
  Serial.println("Break 8");
  }
}


