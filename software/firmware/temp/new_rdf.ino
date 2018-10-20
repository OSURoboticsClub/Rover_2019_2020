////////// Includes //////////
#include <ModbusRtu.h>

////////// Hardware / Data Enumerations //////////
enum HARDWARE {
  COMMS_RS485_EN = 3,
  COMMS_RS485_RX = 9,
  COMMS_RS485_TX = 10,

  // COMMS_RS485_EN = 2,
  // COMMS_RS485_RX = 0,
  // COMMS_RS485_TX = 1,

  RDF_INPUT = A7,

  LED_BLUE_EXTRA = 13
};

enum MODBUS_REGISTERS {
  SENSITIVITY = 0,
  RAW_DATA = 1,  // Input
  CLEAN_DATA_POSITIVE = 2,
  CLEAN_DATA_NEGATIVE = 3,
  FREQUENCY = 4,
};


////////// Global Variables //////////
///// Modbus
const uint8_t node_id = 1;
const uint8_t modbus_serial_port_number = 2;

uint16_t modbus_data[] = {50, 0, 0, 0, 0};
uint8_t num_modbus_registers = 0;

int8_t poll_state = 0;
bool communication_good = false;
uint8_t message_count = 0;


//////////////// Anothony's stuff /////////////
int state;
float freq;
float ambientNoise;
unsigned long totalDataPoints;
int dataBuff[3];
int data[3];
unsigned long t1,t2,t3;
int dt1,dt2 =0;
float dtavg;
int tcnt =2;
bool upstate = false;


////////// Class Instantiations //////////
Modbus slave(node_id, modbus_serial_port_number, HARDWARE::COMMS_RS485_EN);

void setup() {
  // Debugging
  Serial.begin(9600);
 while (!Serial);

  // Raw pin setup
  setup_hardware();

  // Setup modbus serial communication
  num_modbus_registers = sizeof(modbus_data) / sizeof(modbus_data[0]);
  slave.begin(115200); // baud-rate at 19200
  slave.setTimeOut(150);
}

void loop() {
    anthonys_rdf_code();
  // Do normal polling
  poll_modbus();
  set_leds();

}

void anthonys_rdf_code(){
    modbus_data[MODBUS_REGISTERS::RAW_DATA] = analogRead(HARDWARE::RDF_INPUT);

    for(int i=0;i<3;i++){
      dataBuff[i] = data[i];
  }
    dataSet();
    int change = changeCheck();
    if(change){
      state = change;
      if(change == 1){
        t1 = millis();
        dt1 = t1-t2;
      }else{
        t2 = millis();
        dt2 = t2-t1;
      }
      if(dt2>dt1-100&&dt1>dt2-100){
        dtavg = (dtavg*float(tcnt-2)/float(tcnt))+(((dt1+dt2)/2.0)*float(2.0/tcnt));
        tcnt += 2;
        freq = 500.0/dtavg;
      }
    }
    if(change ==1 || millis() > dtavg*2+t3){
      t3 = millis();
      upstate = true;
    }
    if(millis()<t3+dtavg){
      for(int i=0;i<3;i++){
        int out_data = data[i]-ambientNoise;

        if(out_data >=0){
            modbus_data[MODBUS_REGISTERS::CLEAN_DATA_NEGATIVE] = 0;
            modbus_data[MODBUS_REGISTERS::CLEAN_DATA_POSITIVE] = out_data;
        }else{
            modbus_data[MODBUS_REGISTERS::CLEAN_DATA_POSITIVE] = 0;
            modbus_data[MODBUS_REGISTERS::CLEAN_DATA_NEGATIVE] = out_data;
        }
        modbus_data[MODBUS_REGISTERS::FREQUENCY] = freq * 100;
        // Serial.print(data[i]-ambientNoise);
        // Serial.print(", ");
        Serial.println(freq);
      }
    }else{
      float avgdat,dtot =0;
      for(int i=0;i<3;i++)
        dtot+=data[i];
      avgdat = dtot/3.0;
      ambientNoise = ambientNoise*float(totalDataPoints)/float(totalDataPoints+3)+avgdat*(3.0/float(totalDataPoints+3));
    }
}

void dataSet(){
  for(int i=0;i<3;i++){
    data[i]=analogRead(HARDWARE::RDF_INPUT);
    delay(1);
  }
}

int changeCheck(){
    uint16_t sensitivity = modbus_data[MODBUS_REGISTERS::SENSITIVITY];
    int newSignalState = 0;                // 0= no change      1= signal start       2= signal stop
    if((data[0])>(dataBuff[2]+sensitivity)||data[2] > data[0]+sensitivity)
      newSignalState = 1;
    if((data[2] < data[0]-sensitivity)||(data[0]<(dataBuff[2]-sensitivity)))
      newSignalState = 2;
    return newSignalState;

}

void setup_hardware() {
  // Setup pins as inputs / outputs
  pinMode(HARDWARE::RDF_INPUT, INPUT);
  pinMode(HARDWARE::LED_BLUE_EXTRA, OUTPUT);

}

void poll_modbus() {
  poll_state = slave.poll(modbus_data, num_modbus_registers);
  communication_good = !slave.getTimeOutState();
}

void set_leds() {
  if (poll_state > 4) {
    message_count++;
    if (message_count > 2) {
      digitalWrite(HARDWARE::LED_BLUE_EXTRA, !digitalRead(HARDWARE::LED_BLUE_EXTRA));
      message_count = 0;
    }
  } else if (!communication_good) {
    digitalWrite(HARDWARE::LED_BLUE_EXTRA, LOW);
  }
}
