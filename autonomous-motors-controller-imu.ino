#include <FilterDerivative.h>
#include <FilterOnePole.h>
#include <Filters.h>
#include <FilterTwoPole.h>
#include <FloatDefine.h>
#include <RunningStatistics.h>

#include <Servo.h>
#include <PS2X_lib.h>  //for v1.6

// Includes for IMU 
#include "quaternionFilters.h"
#include "MPU9250.h"
// Includes for Pressure sensor
#include <Wire.h>
#include <SparkFun_MS5803_I2C.h>

/******************************************************************
 * set pins connected to PS2 controller:
 *   - 1e column: original 
 *   - 2e colmun: Stef?
 * replace pin numbers by the ones you use
 ******************************************************************/
#define PS2_DAT        8  //14    
#define PS2_CMD        11  //15
#define PS2_SEL        10  //16
#define PS2_CLK        12  //17

/******************************************************************
 * select modes of PS2 controller:
 *   - pressures = analog reading of push-butttons 
 *   - rumble    = motor rumbling
 * uncomment 1 of the lines for each mode selection
 ******************************************************************/
#define pressures   true
//#define pressures   false
//#define rumble      true
#define rumble      false

PS2X ps2x; // create PS2 Controller Class

//right now, the library does NOT support hot pluggable controllers, meaning 
//you must always either restart your Arduino after you connect the controller, 
//or call config_gamepad(pins) again after connecting the controller.

int error = 0;
byte type = 0;
byte vibrate = 0;

/*
 * Initializes four Servo objects
 */
Servo motorR;
Servo motorL;
Servo motorBH;
Servo motorFH;

////////////////////.......................................................////////////////////
// Definitions for IMU
#define AHRS true         // Set to false for basic data read
#define SerialDebug false  // Set to true to get Serial output for debugging
#define LM_SIZE 8
// Definitions for Pressure Sensor
#define AVGNUM 1          // Number that controls the number fo values that are averaged out
#define PRESSURERESET 4000    // Number that determines the number of milliseonds between resetting and starting communication with the pressure sensor
#define DEPTHSIGNAL 20
#define YAWSIGNAL 5

// Pin definitions
int intPin = 12;  // Interrupt pin
int myLed  = 13;  // Set up pin 13 led for toggling
int yawLed = 26;
int depthLed = 30;

//IMU initialization
MPU9250 myIMU;
// Pressure Sensor init
MS5803 sensor(ADDRESS_HIGH);
//Filter 
FilterOnePole lowpassFilter( LOWPASS, 2 );

//IMU variables
float rollavg = 0;
float pitchavg = 0;
float yawavg = 0;
int RPYcounter = 0;

// Pressure sensors variables
float temperature_c;
double pressure_abs,pressure_baseline, depth_water;
unsigned long last_press_reset;
///////////////////////////////////////////////////////////////////////////////////////////

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


String detectMotorInput;

/*
 * Initialize array to obtain user motor input from the PS2 Controller
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
 
void setup(){
  Wire.begin(); // Required for Pressure sensor
  Serial.begin(57600);   // Required for I/O from Serial monitor

  Serial.println("initializing");   // Print a startup message
  initialize();
  
  ////////////////............................................../////////////////
  // Pressure Init //
  pressureInit();
  for (int i = 0; i < AVGNUM; i++)
    pressure_baseline += runningPressureAverage(sensor.getPressure(ADC_4096));
  pressure_baseline /= AVGNUM;

  // Initialize Pins
  pinMode(intPin, INPUT);   // Set up the interrupt pin
  digitalWrite(intPin, LOW); // Its set as active high, push-pull
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);
  // Yaw Led init
  pinMode(yawLed, OUTPUT);   // Set up the interrupt pin
  digitalWrite(yawLed, LOW);
  // Depth Led init
  pinMode(depthLed, OUTPUT);   // Set up the interrupt pin
  digitalWrite(depthLed, LOW);

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  if (SerialDebug)
  {
    Serial.print(F("MPU9250 I AM 0x"));
    Serial.print(c, HEX);
    Serial.print(F(" I should be 0x"));
    Serial.println(0x71, HEX);
  }
  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    if (SerialDebug)
    
    {
      Serial.println(F("MPU9250 is online..."));

      // Start by performing self test and reporting values
      myIMU.MPU9250SelfTest(myIMU.selfTest);
      Serial.print(F("x-axis self test: acceleration trim within : "));
      Serial.print(myIMU.selfTest[0],1); Serial.println("% of factory value");
      Serial.print(F("y-axis self test: acceleration trim within : "));
      Serial.print(myIMU.selfTest[1],1); Serial.println("% of factory value");
      Serial.print(F("z-axis self test: acceleration trim within : "));
      Serial.print(myIMU.selfTest[2],1); Serial.println("% of factory value");
      Serial.print(F("x-axis self test: gyration trim within : "));
      Serial.print(myIMU.selfTest[3],1); Serial.println("% of factory value");
      Serial.print(F("y-axis self test: gyration trim within : "));
      Serial.print(myIMU.selfTest[4],1); Serial.println("% of factory value");
      Serial.print(F("z-axis self test: gyration trim within : "));
      Serial.print(myIMU.selfTest[5],1); Serial.println("% of factory value");
    }

    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);  // Calibrate gyro and accelerometers, load biases in bias registers

    myIMU.initMPU9250();  // Initialize acceleration + gyro
    
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963); // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    if (SerialDebug)
    {
      Serial.print("AK8963 ");
      Serial.print("I AM 0x");
      Serial.print(d, HEX);
      Serial.print(" I should be 0x");
      Serial.println(0xFF, HEX);
    }

    if (d != 0xFF)
    {
      // Communication failed, stop here
      Serial.println(F("Communication failed, abort!"));
      Serial.flush();
      abort();
    }

    myIMU.initAK8963(myIMU.factoryMagCalibration);  // Initialize Magnetometer

    if (SerialDebug)
    {
      Serial.println("Calibration values: ");
      Serial.print("X-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[2], 2);
    }

    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();

    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.

    ///////TURN ON AN LED HERE
    myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
    if (SerialDebug)
    {
      Serial.println("AK8963 mag biases (mG)");
      Serial.println(myIMU.magBias[0]);
      Serial.println(myIMU.magBias[1]);
      Serial.println(myIMU.magBias[2]);
  
      Serial.println("AK8963 mag scale (mG)");
      Serial.println(myIMU.magScale[0]);
      Serial.println(myIMU.magScale[1]);
      Serial.println(myIMU.magScale[2]);
    }

    //TURN OFF AN LED
    // INSTEAD OF DELAY BLINL LED FOR 5 SECS
    delay(5000); // Add delay to see results before serial spew of data

    if(SerialDebug)
    {
      Serial.println("Magnetometer:");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[2], 2);
    }

    last_press_reset = millis();
  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }
  //////////////////////////////////////////////////////////////////////////////////////////// 
    delay(300);  //added delay to give wireless ps2 module some time to startup, before configuring it
     
  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  
  if(error == 0){
    Serial.print("Found Controller, configured successful ");
    Serial.print("pressures = ");
  if (pressures)
    Serial.println("true ");
  else
    Serial.println("false");
  Serial.print("rumble = ");
  if (rumble)
    Serial.println("true)");
  else
    Serial.println("false");
    Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
    Serial.println("holding L1 or R1 will print out the analog stick values.");
    Serial.println("Note: Go to www.billporter.info for updates and to report bugs.");
  }  
  else if(error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
   
  else if(error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

  else if(error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
    
  motorR.attach(2);
  motorL.attach(3);
  motorBH.attach(4);
  motorFH.attach(6);


}

void loop() {

  String motorInputDetected;
  int motorInputArray[4] = {motorStateInfo[0].motorZeroRpmSetting,
                            motorStateInfo[1].motorZeroRpmSetting, motorStateInfo[2].motorZeroRpmSetting, 
                            motorStateInfo[3].motorZeroRpmSetting};
      
  if(error == 1) //skip loop if no controller found
    return; 

  int upArray[] = {0, 0, 60, 90};           //translates to forward motion
  int downArray[] = {0, 0, 105, 45};        //translates to backwards motion
  int forwardArray[] = {80, 110, 0, 0};    //translates to up motion
  int backwardArray[] = {110, 60, 0, 0};     //translates to down motion

  ///////////////////////////.........................................//////////////////////////
  // Pressure Sensor Data recording and filtering
  temperature_c = sensor.getTemperature(CELSIUS, ADC_512);
  pressure_abs = runningPressureAverage(sensor.getPressure(ADC_4096));
  depth_water = depth(pressure_abs , pressure_baseline);

  // IMU data Recording and scaling
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes;// - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes;// - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes;// - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;// - myIMU.gyroBias[0];
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;// - myIMU.gyroBias[1];
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;// - myIMU.gyroBias[2];

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);
                         
  // Serial print and/or display at 0.5 s rate independent of data rates
  myIMU.delt_t = millis() - myIMU.count;

  if(SerialDebug)
  {
    Serial.print("ax = ");  Serial.print((int)1000 * myIMU.ax);
    Serial.print(" ay = "); Serial.print((int)1000 * myIMU.ay);
    Serial.print(" az = "); Serial.print((int)1000 * myIMU.az);
    Serial.println(" mg");

    Serial.print("gx = ");  Serial.print(myIMU.gx, 2);
    Serial.print(" gy = "); Serial.print(myIMU.gy, 2);
    Serial.print(" gz = "); Serial.print(myIMU.gz, 2);
    Serial.println(" deg/s");

    Serial.print("mx = ");  Serial.print((int)myIMU.mx);
    Serial.print(" my = "); Serial.print((int)myIMU.my);
    Serial.print(" mz = "); Serial.print((int)myIMU.mz);
    Serial.println(" mG");

    Serial.print("q0 = ");  Serial.print(*getQ());
    Serial.print(" qx = "); Serial.print(*(getQ() + 1));
    Serial.print(" qy = "); Serial.print(*(getQ() + 2));
    Serial.print(" qz = "); Serial.println(*(getQ() + 3));
  }

  // Calculate RPY using the values obtained from the Quarternion Filter
  myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                * *(getQ()+3));
  myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                * *(getQ()+2)));
  myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                * *(getQ()+3));
  myIMU.pitch *= RAD_TO_DEG;
  myIMU.yaw   *= RAD_TO_DEG;
  myIMU.yaw  += 9.63;  // Specific to Waterloo, ON
  myIMU.roll *= RAD_TO_DEG;

  // Running Average Code
  // Different function has been created for each type of orientation
  myIMU.roll = runningRollAverage(myIMU.roll);
  myIMU.pitch = runningPitchAverage(myIMU.pitch);
  myIMU.yaw = runningYawAverage(myIMU.yaw);

  rollavg += myIMU.roll;
  pitchavg += myIMU.pitch;
  yawavg += myIMU.yaw;
  RPYcounter++;
  
  if(RPYcounter == AVGNUM)
  {
    lowpassFilter.input(yawavg/AVGNUM);
    // Pressure sensor outputs
    if (depth_water < DEPTHSIGNAL + 5 && depth_water > DEPTHSIGNAL-5){
      digitalWrite(depthLed,!digitalRead(depthLed));
    } else {
      digitalWrite(depthLed,LOW);
    }
    if (abs(yawavg/AVGNUM) < YAWSIGNAL){
      digitalWrite(yawLed,!digitalRead(yawLed));
    } else {
      digitalWrite(yawLed,LOW);
    }
    Serial.print(depth_water);
    Serial.print(" ");
    // IMU output
    Serial.print(yawavg/AVGNUM);
    Serial.print(" ");
    Serial.print(pitchavg/AVGNUM);
    Serial.print(" ");
    Serial.println(rollavg/AVGNUM);
    RPYcounter = 0;
    rollavg = 0;
    pitchavg = 0;
    yawavg = 0;
  }
  
  if(SerialDebug)
  {
    Serial.print("rate = ");
    Serial.print((float)myIMU.sumCount / myIMU.sum, 2);
    Serial.println(" Hz");
  }
  myIMU.count = millis();
  myIMU.sumCount = 0;
  myIMU.sum = 0;

  if(last_press_reset - millis() > PRESSURERESET)
  {
    digitalWrite(myLed, !digitalRead(myLed));
    pressureInit();
    last_press_reset = millis();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////
  
  ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
  if
  
  if(ps2x.Button(PSB_PAD_UP)) {      //will be TRUE as long as button is pressed
    Serial.print("UP");
    memcpy( motorInputArray, upArray, 4*sizeof(int) );
  }

  if(ps2x.Button(PSB_PAD_DOWN)) {      //will be TRUE as long as button is pressed
    Serial.print("DOWN");
    memcpy( motorInputArray, downArray, 4*sizeof(int) );
  }

  if(ps2x.Button(PSB_PAD_RIGHT)) {      //will be TRUE as long as button is pressed
    Serial.print("FORWARD");
    memcpy( motorInputArray, forwardArray, 4*sizeof(int) );
  }

  if(ps2x.Button(PSB_PAD_LEFT)) {      //will be TRUE as long as button is pressed
    Serial.print("BACKWARDS");
    memcpy( motorInputArray, backwardArray, 4*sizeof(int) );
  }
    
//  motorInputDetected = detectSerialMonitor();
//  if (motorInputDetected != "")
//  {
//    //Serial.println("User input detected!");
//    Serial.println("Read " + motorInputDetected);
//    Serial.println("\nCurrent State");
//    Serial.println(motorStateInfo[0].motorCurrentRpmSetting);
//    Serial.println(motorStateInfo[1].motorCurrentRpmSetting);
//    Serial.println(motorStateInfo[2].motorCurrentRpmSetting);
//    Serial.println(motorStateInfo[3].motorCurrentRpmSetting);
//    parseMotorInput(motorInputDetected, motorInputArray);
    
//    Serial.println("\nDesired State");
//    Serial.println(motorInputArray[0]);
//    Serial.println(motorInputArray[1]);
//    Serial.println(motorInputArray[2]);
//    Serial.println(motorInputArray[3]);
//    Serial.println("\n");
    
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
//  }

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
      //Serial.println("Case Right Motor");
      writeMotorR(rpmPercentage);
      break;
    case 1:
      //Serial.println("Case Left Motor");
      writeMotorL(rpmPercentage);
      break;
    case 2:
      //Serial.println("Case Back-Heave Motor");
      writeMotorBH(rpmPercentage);
      break;
    case 3:
      //Serial.println("Case Front-Heave Motor");
      writeMotorFH(rpmPercentage);
      break;
    default:
      //Serial.println("No motor written to");
      break;
  }
}


/*
 * Write total RPM percentage to Right-Front Motor
 */
void writeMotorR(int rpmPercentage) {
  //Serial.println("Writing to Motor R");
  int valueMotorInteger = 0;
  int rampValue = motorStateInfo[0].motorCurrentRpmSetting;
  
  //valueMotorInteger = checkMotorInput(rpmPercentage, 0);
  if (valueMotorInteger != -1){

    /*
     * If: ramping faster correct direction
     * Else: ramping slower
     */
    if (motorStateInfo[0].motorCurrentRpmSetting < rpmPercentage){
      /*
       Serial.println("Ramping up: ");
       Serial.println(motorStateInfo[0].motorCurrentRpmSetting);
       Serial.println(rpmPercentage);
       Serial.println("\n");
       */
       while (rampValue < rpmPercentage){
        rampValue += 2;
        motorR.write(rampValue); 
       } 
    }
    else if (motorStateInfo[0].motorCurrentRpmSetting > rpmPercentage){
      /*
      Serial.println("Ramping down");
      Serial.println(motorStateInfo[0].motorCurrentRpmSetting);
      Serial.println(rpmPercentage);
      Serial.println("\n");
      */
      while (rampValue > rpmPercentage){
        rampValue -= 2;
        motorR.write(rampValue); 
       }  
    }
    else{
      //Serial.println("No change\n");
    }
  }
}

/*
 * Write total RPM percentage to Left-Front Motor
 */
void writeMotorL(int rpmPercentage) {
  //Serial.println("Writing to Motor L");
  int valueMotorInteger = 0;
  int rampValue = motorStateInfo[1].motorCurrentRpmSetting;
  
  //valueMotorInteger = checkMotorInput(rpmPercentage, 0);
  if (valueMotorInteger != -1){

    /*
     * If: ramping faster correct direction
     * Else: ramping slower
     */
    if (motorStateInfo[1].motorCurrentRpmSetting < rpmPercentage){
      /*
       Serial.println("Ramping up: ");
       Serial.println(motorStateInfo[1].motorCurrentRpmSetting);
       Serial.println(rpmPercentage);
       Serial.println("\n");
       */
       while (rampValue < rpmPercentage){
        rampValue += 2;
        motorL.write(rampValue); 
       } 
    }
    else if (motorStateInfo[1].motorCurrentRpmSetting > rpmPercentage){
      /*
      Serial.println("Ramping down");
      Serial.println(motorStateInfo[1].motorCurrentRpmSetting);
      Serial.println(rpmPercentage);
      Serial.println("\n");
      */
      while (rampValue > rpmPercentage){
        rampValue -= 2;
        motorL.write(rampValue); 
       }  
    }
    else{
      //Serial.println("No change\n");
    }
  }
}

/*
 * Write total RPM percentage to Right-Back Motor
 */
void writeMotorBH(int rpmPercentage) {
  //Serial.println("Writing to Motor BH");
  int valueMotorInteger = 0;
  int rampValue = motorStateInfo[2].motorCurrentRpmSetting;
  
  //valueMotorInteger = checkMotorInput(rpmPercentage, 0);
  if (valueMotorInteger != -1){

    /*
     * If: ramping faster correct direction
     * Else: ramping slower
     */
    if (motorStateInfo[2].motorCurrentRpmSetting < rpmPercentage){
      /*
       Serial.println("Ramping up: ");
       Serial.println(motorStateInfo[2].motorCurrentRpmSetting);
       Serial.println(rpmPercentage);
       Serial.println("\n");
       */
       while (rampValue < rpmPercentage){
        rampValue += 2;
        motorBH.write(rampValue); 
        delay(10);
       } 
    }
    else if (motorStateInfo[2].motorCurrentRpmSetting > rpmPercentage){
      /*
      Serial.println("Ramping down");
      Serial.println(motorStateInfo[2].motorCurrentRpmSetting);
      Serial.println(rpmPercentage);
      Serial.println("\n");
      */
      while (rampValue > rpmPercentage){
        rampValue -= 2;
        motorBH.write(rampValue); 
       }  
    }
    else{
      //Serial.println("No change\n");
    }
  }
}

/*
 * Write total RPM percentage to Left-Back Motor
 */
void writeMotorFH(int rpmPercentage) {
  //Serial.println("Writing to Motor FH");
  int valueMotorInteger = 0;
  int rampValue = motorStateInfo[3].motorCurrentRpmSetting;
  
  //valueMotorInteger = checkMotorInput(rpmPercentage, 0);
  if (valueMotorInteger != -1){

    /*
     * If: ramping faster correct direction
     * Else: ramping slower
     */
    if (motorStateInfo[3].motorCurrentRpmSetting < rpmPercentage){
       /*
       Serial.println("Ramping up: ");
       Serial.println(motorStateInfo[3].motorCurrentRpmSetting);
       Serial.println(rpmPercentage);
       Serial.println("\n");
       */
       while (rampValue < rpmPercentage){
        rampValue += 2;
        motorFH.write(rampValue); 
       } 
    }
    else if (motorStateInfo[3].motorCurrentRpmSetting > rpmPercentage){
      /*
      Serial.println("Ramping down");
      Serial.println(motorStateInfo[3].motorCurrentRpmSetting);
      Serial.println(rpmPercentage);
      Serial.println("\n");
      */
      while (rampValue > rpmPercentage){
        rampValue -= 2;
        motorFH.write(rampValue); 
       }  
    }
    else{
      //Serial.println("No change\n");
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

///////////////////////////....................................................../////////////////////////
// Pressure Sensor Functions //
double depth(double P, double P0)
{
  return (100*(P-P0)/(0.993*9.806));
}

double runningPressureAverage(double M) 
{
  static double LM[LM_SIZE];      // LastMeasurements
  static int index = 0;
  static double sum = 0;
  static int count = 0;
  
  // keep sum updated to improve speed.
  sum -= LM[index];
  LM[index] = M;
  sum += LM[index];
  index++;
  index = index % LM_SIZE;
  if (count < LM_SIZE) count++;
  
  return (sum) / (count);
}

void pressureInit()
{
  sensor.reset();
  sensor.begin();
}
//___________________________________________//

float runningRollAverage(float M) 
{
  static float LM[LM_SIZE];      // LastMeasurements
  static int index = 0;
  static float sum = 0;
  static int count = 0;

  // keep sum updated to improve speed.
  sum -= LM[index];
  LM[index] = M;
  sum += LM[index];
  index++;
  index = index % LM_SIZE;
  if (count < LM_SIZE) count++;

  return (sum + count*M) / (2*count);
}

float runningPitchAverage(float M) 
{
  static float LM[LM_SIZE];      // LastMeasurements
  static int index = 0;
  static float sum = 0;
  static int count = 0;

  // keep sum updated to improve speed.
  sum -= LM[index];
  LM[index] = M;
  sum += LM[index];
  index++;
  index = index % LM_SIZE;
  if (count < LM_SIZE) count++;

  return (sum + count*M) / (2*count);
}

float runningYawAverage(float M) 
{
  static float LM[LM_SIZE];      // LastMeasurements
  static int index = 0;
  static float sum = 0;
  static int count = 0;

  // keep sum updated to improve speed.
  sum -= LM[index];
  LM[index] = M;
  sum += LM[index];
  index++;
  index = index % LM_SIZE;
  if (count < LM_SIZE) count++;

  return (sum + count*M) / (2*count);
}
////////////////////////////////////////////////////////////////////////////////////////////////
