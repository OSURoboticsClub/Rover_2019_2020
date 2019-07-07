/**********************************************
*This is example code written by Anthony Grana
*This code was written for the current sense / temperature sense (CSTS) library
*default calibration values for the csts are 50, 2.2, 158, 81, (3.3/1024)
*
*
*
*
*
**********************************************/





#include <CSTS.h>												//required for library


CSTS sense(50,2.2,158,81,(3.3/1024));							// This is the constructor for the library where you set calibration values

float Temp;
float CS;

  void setColor(int speed, bool forward, int bluepin, int redpin, int greenpin)
{
  analogWrite(bluepin, 127);
  int onPin;
  int offPin;
  
  if (forward){
    onPin = greenpin;
    offPin = redpin;
  }
  if (!forward){
    onPin = redpin;
    offPin = greenpin;
  }
  digitalWrite(offPin, LOW); 
  analogWrite(onPin, speed);   
}

void setup() {
  Serial.begin(9600);
  //Serial.setTimeout(10);
  pinMode(19,OUTPUT);
  
  analogReference(EXTERNAL);
}
  
int spd, last;
bool dir;

void loop() {

  //read serial
  while(Serial.available()){																	// this serial read section allows you to input motor speed through the serial monitor
    spd = Serial.parseInt();																	// the speed range varies from 0 through 255 and direction can be changed by adding an 'f' after the speed
    Serial.print(spd);																			// examples: 110, 50, 40f, 200f, 255, 0, 92f
    if(Serial.read() == 'f'){
      dir = true;
      Serial.println(" Forwards");
    }else{
      dir = false;
      Serial.println(" Backwords");
    }
    Serial.read();
  }


  //MotorWrite
  analogWrite(20,spd);
  digitalWrite(19,dir);

  setColor(spd,dir,6,32,25);

  sense.update(CS,Temp);																		// This is a pass by reference function changes the values "CS" and "Temp" to the correct current and temperature values.

  //Current Sense
  /*
  int raw = analogRead(A4);
  Serial.print(raw);
  Serial.print("  ");
  float raw_v = raw*(3.3/1023);
  Serial.println(raw_v,4);
*/
  // Noise Reduction
  //int Data [10] = 0;


  
  
  delay(20);
}






















  
