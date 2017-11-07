#include <SoftwareSerial.h>

#define txPin 3                                            //define pins used for software serial for sonar
#define rxPin 2

boolean stringComplete = false;
SoftwareSerial sonarSerial(rxPin, txPin, true);            //define serial port for recieving data, output from maxSonar is inverted requiring true to be set.

void setup() {
  setupRS232();
  setupAnalogRead();
}

void loop() {
  loopRS232();
  loopAnalogRead();
}


void setupAnalogRead() {
  Serial.begin(9600);
}

void loopAnalogRead() {
  // put your main code here, to run repeatedly:
  double sonar = analogRead(0);
  Serial.print("Analog ");
  Serial.println(sonar*(5.0/1024.0));
    
  //delay(300);  
}

void setupRS232() {
  Serial.begin(9600);
  sonarSerial.begin(9600);                                 //start serial port for maxSonar
  //delay(500);    
}


void loopRS232() {
  int range = EZread();
  if(stringComplete)
  {
    stringComplete = false;                                //reset sringComplete ready for next reading

    Serial.print("Digital ");
    Serial.print(range*(5.0/1024.0));
    Serial.print(" ");
    //delay(500);                                          //delay for debugging
  } 
}

int EZread() 
{
  int result;
  char inData[4];                                          //char array to read data into
  int index = 0;


  sonarSerial.flush();                                     // Clear cache ready for next reading

  while (stringComplete == false) {
    //Serial.print("reading ");    //debug line

    if (sonarSerial.available())
    {
      char rByte = sonarSerial.read();                     //read serial input for "R" to mark start of data
      if(rByte == 'R')
      {
        //Serial.println("rByte set");
        while (index < 3)                                  //read next three character for range from sensor
        {
          if (sonarSerial.available())
          {
            inData[index] = sonarSerial.read(); 
            //Serial.println(inData[index]);               //Debug line

            index++;                                       // Increment where to write next
          }  
        }
        inData[index] = 0x00;                              //add a padding byte at end for atoi() function
      }

      rByte = 0;                                           //reset the rByte ready for next reading

      index = 0;                                           // Reset index ready for next reading
      stringComplete = true;                               // Set completion of read to true
      result = atoi(inData);                               // Changes string data into an integer for use
    }
  }

  return result;
}















