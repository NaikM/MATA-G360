#include <Servo.h>
#include <PS2X_lib.h>  //for v1.6

/*
 * Initializes four Servo objects
 */
Servo motorR;
Servo motorL;
Servo motorBH;
Servo motorFH;

/*
 * Initialize a struct of motor states
 */
struct motorState{
  Servo motorObject;
  String motorName; 
  int motorZeroRpmSetting;
  int motorCurrentRpmSetting;
};

/*
 * Initialize four motorStates
 */
motorState motorRState = {motorR, "Right Motor", 100, motorRState.motorZeroRpmSetting};
motorState motorLState = {motorL, "Left Motor", 90, motorLState.motorZeroRpmSetting};
motorState motorBHState = {motorBH, "Back-Heave Motor", 90, motorBHState.motorZeroRpmSetting};
motorState motorFHState = {motorFH, "Front-Heave Motor", 60, motorFHState.motorZeroRpmSetting};

 
/*
 * Initialize array of four motorStates
 */
motorState motorStateInfo[] = {motorRState, motorLState, motorBHState, motorFHState};

/*
 * Initialize incoming string
 */
String motorInput = "";

/*
 * Initialize string to detect user motor input
 */
String motorInputDetected;
String detectMotorInput;

/*
 * Initialize array to obtain user motor input
 */
int motorInputArray[4]; 

/*******************************************************
 ****************** HELPER DECLARATIONS ****************
 ******************************************************/
int checkMotorInput(int valueMotorInput, int whichMotor);
void writeMotorR(int rpmPercentage);
void writeMotorL(int rpmPercentage);
void writeMotorBH(int rpmPercentage);
void writeMotorFH(int rpmPercentage);
 
void setup() {
  
  motorR.attach(2);
  motorL.attach(3);
  motorBH.attach(4);
  motorFH.attach(5);

  Serial.begin(9600);   // Required for I/O from Serial monitor
  Serial.println("initializing");   // Print a startup message
  initialize();
}

void loop() {
  //String motorInputDetected;
  //int motorInputArray[4]; 
  
  motorInputDetected = detectSerialMonitor();
  if (motorInputDetected != "")
  {
    //Serial.println("User input detected!");
    Serial.println("Read " + motorInputDetected);
    Serial.println("\nCurrent State");
    Serial.println(motorStateInfo[0].motorCurrentRpmSetting);
    Serial.println(motorStateInfo[1].motorCurrentRpmSetting);
    Serial.println(motorStateInfo[2].motorCurrentRpmSetting);
    Serial.println(motorStateInfo[3].motorCurrentRpmSetting);
    
    parseMotorInput(motorInputDetected, motorInputArray);

    Serial.println("\nDesired State");
    Serial.println(motorInputArray[0]);
    Serial.println(motorInputArray[1]);
    Serial.println(motorInputArray[2]);
    Serial.println(motorInputArray[3]);
    Serial.println("\n");
    
    for(int whichMotor = 0; whichMotor < 4; whichMotor++)
    {
      int valueMotorInteger;
      valueMotorInteger = checkMotorInput(motorInputArray[whichMotor], whichMotor);
      if (valueMotorInteger != -1){
        writeMotor(valueMotorInteger, whichMotor);
        motorStateInfo[whichMotor].motorCurrentRpmSetting = valueMotorInteger;
      }
    }
    motorInputDetected = "";
    detectMotorInput = "";
  }

}

/***************************************************************
 ********************* MAIN MOTOR LIBRARY **********************
 **************************************************************/
/*
 * Initialize all ESC's by writing the neutral value
 */
void initialize() {
  for( int i = 0; i < 4; i++)
  {
    motorStateInfo[i].motorObject.write(motorStateInfo[i].motorZeroRpmSetting);
  }
}

/*
 * Detect serial monitor input
 */
String detectSerialMonitor() {
  
  if(Serial.available() > 0)
  {
    // read the value
    char ch = Serial.read();
  
    /*
     *  If ch isn't a newline(linefeed) character, we will add the characterto the incomingString
     */
    if (ch != 10)
    {    
      detectMotorInput += ch;   // Add the character to the incomingString
    }
    else
    {
      //detectMotorInput = motorInput;
      //detectMotorInput = detectMotorInput;
      return detectMotorInput; 
    }
  }

    //return detectMotorInput;  
    return ""; 
 }

/*
 * Parse serial monitor into motor inputs
 */
void parseMotorInput(String detectMotorInput, int motorInputArray[4]) {
  
  for(int i = 0; i < 3; i++)
  {
    int spaceIndex;
    String motorInputString; 
    int motorInputInteger;
    
    spaceIndex = detectMotorInput.indexOf(" ");
    motorInputString = detectMotorInput.substring(0,spaceIndex);
    detectMotorInput = detectMotorInput.substring(spaceIndex+1,-1);

    //Serial.println("First Substring:" + motorInputString);
    //Serial.println("Remaining Substring:" + detectMotorInput);
    
    motorInputInteger = motorInputString.toInt();
    motorInputArray[i] = motorInputInteger;
  }

  detectMotorInput = detectMotorInput.substring(0,-1);
  //Serial.println("Final Substring:" + detectMotorInput);
  motorInputArray[3] = detectMotorInput.toInt();
}

/*
 * Write total RPM percentage to Any Motor
 */
void writeMotor(int rpmPercentage, int whichMotor) {
  switch (whichMotor)
  {
    case 0:
      Serial.println("Case Right Motor");
      writeMotorR(rpmPercentage);
      break;
    case 1:
      Serial.println("Case Left Motor");
      writeMotorL(rpmPercentage);
      break;
    case 2:
      Serial.println("Case Back-Heave Motor");
      writeMotorBH(rpmPercentage);
      break;
    case 3:
      Serial.println("Case Front-Heave Motor");
      writeMotorFH(rpmPercentage);
      break;
    default:
      Serial.println("No motor written to");
      break;
  }
}


/*
 * Write total RPM percentage to Right-Front Motor
 */
void writeMotorR(int rpmPercentage) {
  Serial.println("Writing to Motor R");
  int valueMotorInteger = 0;
  int rampValue = motorStateInfo[0].motorCurrentRpmSetting;
  
  //valueMotorInteger = checkMotorInput(rpmPercentage, 0);
  if (valueMotorInteger != -1){

    /*
     * If: ramping faster correct direction
     * Else: ramping slower
     */
    if (motorStateInfo[0].motorCurrentRpmSetting < rpmPercentage){
       Serial.println("Ramping up: ");
       Serial.println(motorStateInfo[0].motorCurrentRpmSetting);
       Serial.println(rpmPercentage);
       Serial.println("\n");
       while (rampValue < rpmPercentage){
        rampValue += 2;
        motorR.write(rampValue); 
       } 
    }
    else if (motorStateInfo[0].motorCurrentRpmSetting > rpmPercentage){
      Serial.println("Ramping down");
      Serial.println(motorStateInfo[0].motorCurrentRpmSetting);
      Serial.println(rpmPercentage);
      Serial.println("\n");
      while (rampValue > rpmPercentage){
        rampValue -= 2;
        motorR.write(rampValue); 
       }  
    }
    else{
      Serial.println("No change\n");
    }
  }
}

/*
 * Write total RPM percentage to Left-Front Motor
 */
void writeMotorL(int rpmPercentage) {
  Serial.println("Writing to Motor L");
  int valueMotorInteger = 0;
  int rampValue = motorStateInfo[1].motorCurrentRpmSetting;
  
  //valueMotorInteger = checkMotorInput(rpmPercentage, 0);
  if (valueMotorInteger != -1){

    /*
     * If: ramping faster correct direction
     * Else: ramping slower
     */
    if (motorStateInfo[1].motorCurrentRpmSetting < rpmPercentage){
       Serial.println("Ramping up: ");
       Serial.println(motorStateInfo[1].motorCurrentRpmSetting);
       Serial.println(rpmPercentage);
       Serial.println("\n");
       while (rampValue < rpmPercentage){
        rampValue += 2;
        motorL.write(rampValue); 
       } 
    }
    else if (motorStateInfo[1].motorCurrentRpmSetting > rpmPercentage){
      Serial.println("Ramping down");
      Serial.println(motorStateInfo[1].motorCurrentRpmSetting);
      Serial.println(rpmPercentage);
      Serial.println("\n");
      while (rampValue > rpmPercentage){
        rampValue -= 2;
        motorL.write(rampValue); 
       }  
    }
    else{
      Serial.println("No change\n");
    }
  }
}

/*
 * Write total RPM percentage to Right-Back Motor
 */
void writeMotorBH(int rpmPercentage) {
  Serial.println("Writing to Motor BH");
  int valueMotorInteger = 0;
  int rampValue = motorStateInfo[2].motorCurrentRpmSetting;
  
  //valueMotorInteger = checkMotorInput(rpmPercentage, 0);
  if (valueMotorInteger != -1){

    /*
     * If: ramping faster correct direction
     * Else: ramping slower
     */
    if (motorStateInfo[2].motorCurrentRpmSetting < rpmPercentage){
       Serial.println("Ramping up: ");
       Serial.println(motorStateInfo[2].motorCurrentRpmSetting);
       Serial.println(rpmPercentage);
       Serial.println("\n");
       while (rampValue < rpmPercentage){
        rampValue += 2;
        motorBH.write(rampValue); 
        delay(10);
       } 
    }
    else if (motorStateInfo[2].motorCurrentRpmSetting > rpmPercentage){
      Serial.println("Ramping down");
      Serial.println(motorStateInfo[2].motorCurrentRpmSetting);
      Serial.println(rpmPercentage);
      Serial.println("\n");
      while (rampValue > rpmPercentage){
        rampValue -= 2;
        motorBH.write(rampValue); 
       }  
    }
    else{
      Serial.println("No change\n");
    }
  }
}

/*
 * Write total RPM percentage to Left-Back Motor
 */
void writeMotorFH(int rpmPercentage) {
  Serial.println("Writing to Motor FH");
  int valueMotorInteger = 0;
  int rampValue = motorStateInfo[3].motorCurrentRpmSetting;
  
  //valueMotorInteger = checkMotorInput(rpmPercentage, 0);
  if (valueMotorInteger != -1){

    /*
     * If: ramping faster correct direction
     * Else: ramping slower
     */
    if (motorStateInfo[3].motorCurrentRpmSetting < rpmPercentage){
       Serial.println("Ramping up: ");
       Serial.println(motorStateInfo[3].motorCurrentRpmSetting);
       Serial.println(rpmPercentage);
       Serial.println("\n");
       while (rampValue < rpmPercentage){
        rampValue += 2;
        motorFH.write(rampValue); 
       } 
    }
    else if (motorStateInfo[3].motorCurrentRpmSetting > rpmPercentage){
      Serial.println("Ramping down");
      Serial.println(motorStateInfo[3].motorCurrentRpmSetting);
      Serial.println(rpmPercentage);
      Serial.println("\n");
      while (rampValue > rpmPercentage){
        rampValue -= 2;
        motorFH.write(rampValue); 
       }  
    }
    else{
      Serial.println("No change\n");
    }
  }
}

void softStartUp() {
  
}

void softShutDown() {
  
}

void hardStartUp() {
  
}

void hardShutDown() {
  
}

/***************************************************************
 *********************** HELPER FUNCTIONS **********************
 **************************************************************/

/*
 * Check if input to motors are valid
 */
int checkMotorInput(int valueMotorInput, int whichMotor) {
     
  /*
   *  We only want to write an integer between 10 and 170 to the motor. 
   */
  if (valueMotorInput >= 20 && valueMotorInput <= 150)
  {
    //Serial.println("Value is valid");
  }
  else
  {
    valueMotorInput = -1;
  }
    
  return valueMotorInput;
}
