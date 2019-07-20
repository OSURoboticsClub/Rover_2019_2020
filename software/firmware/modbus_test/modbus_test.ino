
#include <ModbusRtu.h>
uint16_t modbus_data[] = {1000,15256};
uint8_t num_modbus_registers = 2;
int8_t poll_state = 0;
bool communication_good = false;
//uint8_t message_count = 0;

Modbus slave(1,2,2);

void setup() {
  pinMode(1, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(13, OUTPUT);
  slave.begin(115200);
  slave.setTimeOut(500);
  //Serial.begin(9600);

  digitalWrite(1,HIGH);
  digitalWrite(32,HIGH);
  digitalWrite(6,HIGH);
  digitalWrite(13,LOW);
}

unsigned long blink_timer;
bool led_state = LOW;

void loop() {
  poll_modbus();

  if(blink_timer < millis() - modbus_data[0]){
    led_state = !led_state;
    blink_timer = millis();
  }
  digitalWrite(13,led_state);

  if(modbus_data[0] == modbus_data[1])
    digitalWrite(6,LOW);
  else
    digitalWrite(6,HIGH);

  digitalWrite(1,!communication_good);

/*
  if(Serial.available() > 6){
    modbus_data[0] = Serial.parseInt();
    Serial.read();
    modbus_data[1] = Serial.parseInt();
    while(Serial.available())
      Serial.read();
    Serial.println(modbus_data[0]);
  }
  */
}

void poll_modbus(){
    poll_state = slave.poll(modbus_data, num_modbus_registers);
    communication_good = !slave.getTimeOutState();
}
