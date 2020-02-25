//#define POTHOS_DEBUG
#include <pothos.h>

unsigned long blink_timer = 0;
unsigned long blinkTime = 1000;
bool blinkState = false;

int EnablePin = 2;

enum DATA{
  CHAR_DATA = 0,
  ALT_DATA = 1,
  TIME_DATA = 2,
  TMP_DATA = 3  
};

pothos comms(1, EnablePin);

void setup() {
  Serial1.begin(115200);
  comms.setPort(&Serial1);
  
  Serial.begin(115200);
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  
  set_data_types();
}

void loop() {
  comms.update();

  if(millis() - blink_timer >= blinkTime){
    blinkState = !blinkState;
    digitalWrite(13,blinkState);
    blink_timer = millis();
  }
}


void set_data_types(){
  comms.data.set_type(DATA::CHAR_DATA, "char"); 
  comms.data.set_type(DATA::ALT_DATA, "int");
  comms.data.set_type(DATA::TIME_DATA, "long");
  comms.data.set_type(DATA::TMP_DATA, "float");
}
