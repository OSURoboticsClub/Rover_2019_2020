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
  RAW_DATA = 0
};


////////// Global Variables //////////
///// Modbus
const uint8_t node_id = 1;
const uint8_t modbus_serial_port_number = 2;

uint16_t modbus_data[] = {0};
uint8_t num_modbus_registers = 0;

int8_t poll_state = 0;
bool communication_good = false;
uint8_t message_count = 0;

////////// Class Instantiations //////////
Modbus slave(node_id, modbus_serial_port_number, HARDWARE::COMMS_RS485_EN);

void setup() {
  // Debugging
 //  Serial.begin(9600);
 // while (!Serial);

  // Raw pin setup
  setup_hardware();

  // Setup modbus serial communication
  num_modbus_registers = sizeof(modbus_data) / sizeof(modbus_data[0]);
  slave.begin(115200); // baud-rate at 19200
  slave.setTimeOut(100);

}

void loop() {
  // Do normal polling
  poll_modbus();
  set_leds();

}


void setup_hardware() {
  // Setup pins as inputs / outputs
  pinMode(HARDWARE::RDF_INPUT, INPUT);
  pinMode(HARDWARE::LED_BLUE_EXTRA, OUTPUT);

  analogReadResolution(13);

}

void poll_modbus() {
  poll_state = slave.poll(modbus_data, num_modbus_registers);
  communication_good = !slave.getTimeOutState();

  modbus_data[MODBUS_REGISTERS::RAW_DATA] = analogRead(HARDWARE::RDF_INPUT);
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
