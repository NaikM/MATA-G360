/* SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the EMSENSR-9250 breakout board.

 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ---------------------- SDA
 SCL ---------------------- SCL
 GND ---------------------- GND
 */

/* Download and Install Libraries 
 *  https://github.com/sparkfun/SparkFun_MS5803-14BA_Breakout_Arduino_Library/tree/V_1.1.1
 *  https://github.com/sparkfun/SparkFun_MPU-9250_Breakout_Arduino_Library/tree/master
 */

// Includes for IMU 
#include "quaternionFilters.h"
#include "MPU9250.h"
// Includes for Pressure sensor
#include <Wire.h>
#include <SparkFun_MS5803_I2C.h>

// Definitions for IMU
#define AHRS true         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging
#define LM_SIZE 16
// Definitions for Pressure Sensor
#define AVGNUM 8          // Number that controls the number fo values that are averaged out
#define PRESSURERESET 4000    // Number that determines the number of milliseonds between resetting and starting communication with the pressure sensor

// Pin definitions
int intPin = 12;  // Interrupt pin
int myLed  = 13;  // Set up pin 13 led for toggling

//IMU initialization
MPU9250 myIMU;
// Pressure Sensor init
MS5803 sensor(ADDRESS_HIGH);

//IMU variables
float rollavg = 0;
float pitchavg = 0;
float yawavg = 0;
int RPYcounter = 0;

// Pressure sensors variables
float temperature_c;
double pressure_abs,pressure_baseline, depth_water;
unsigned long last_press_reset;

void setup()
{
  Wire.begin(); // Required for Pressure sensor
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(38400);

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
}

void loop()
{
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
  myIMU.yaw  += 9.63;
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
    // Pressure sensor outputs //
    Serial.print(depth_water);
    Serial.print(" ");
    //_____________________________________//
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
}

// ADDITIONAL FUNCTIONS //

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
