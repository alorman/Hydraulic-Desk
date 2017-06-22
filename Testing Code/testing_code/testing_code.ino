// Simple test script to run hydraulic pump up and down
// Bang Bang-only control implemented

// Global variables
  //Output pin definitions
  int MotorUpPin = 1;
  int MotorDownPin = 2;
  int MotorEnable = 4;
  //Input pins
  int PushbuttonPin = 3;
  int SwitchDirUp = 4;
  int SwitchDirDown = 5;

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
}
//End main loop

void ReadInputs() {
 MotorUpSwitch = digitalRead(SwitchDirUp);
 MotorDownSwitch = digitalRead(SwitchDirDown);
 PushbuttonPushed = digitalRead(PushbuttonPin); 
}

void ControlLogic() {
  if(MotorUpSwitch == HIGH && PushbuttonPushed == HIGH)
  {
    MotorOutDown = 0;
    MotorOutUp = 1;
    EnableOut = 1;
  }
  else if(MotorDownSwitch == HIGH && PushbuttonPushed == HIGH)
  {
    MotorOutUp = 0;
    MotorOutDown = 1;
    EnableOut = 1;
  }
  else if(MotorDownSwitch == HIGH || MotorUpSwitch == HIGH && PushbuttonPushed == LOW)
  {
    MotorOutUp = 0;
    MotorOutDown = 0;
    EnableOut = 0;
  }
}

void OutputControl() {
  if (MotorOutDown == 1)
  {
  digitalWrite(MotorDownPin, HIGH);
  digitalWrite(MotorUpPin, LOW);
  digitalWrite(EnableOut, HIGH);
  } else {
  digitalWrite(MotorDownPin, LOW);
  digitalWrite(MotorUpPin, LOW); 
  digitalWrite(EnableOut, LOW); 
  }

  if (MotorOutUp == 1)
 {
  digitalWrite(MotorUpPin, HIGH);
  digitalWrite(MotorDownPin, LOW);
  digitalWrite(EnableOut, HIGH);
 } else {
  digitalWrite(MotorUpPin, LOW);
  digitalWrite(MotorDownPin, LOW);
  digitalWrite(EnableOut, LOW);
 }
}


