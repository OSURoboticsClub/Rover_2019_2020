////////// Includes //////////
#include "SBUS.h"
#include <ModbusRtu.h>
#include "Arduino.h" // Needed so I can nicely define things

////////// Hardware / Data Enumerations //////////
enum HARDWARE {
    TELEM_24V = A0,
    TELEM_5V = A1,
    USB_TELEM_5V = A2,
    TELEM_3V3 = A3,

    BAD_1 = 3,
    BAD_2 = 4,
    BAD_3 = 23,
    BAD_4 = 24,

    LED_RED = 1,
    LED_GREEN = 32,
    LED_BLUE = 6,

    LED_BLUE_EXTRA = 13
};

enum MODBUS_REGISTERS {
    // 0-15 are directly written from SBUS Class
    VOLTAGE_24 = 16,
    VOLTAGE_5 = 17,
    USB_VOLTAGE_5 = 18,
    VOLTAGE_3V3 = 19
};

////////// Global Variables //////////
const uint8_t node_id = 1;
const uint8_t mobus_serial_port_number = 2;

#define SBUS_HARDWARE_PORT Serial3

uint16_t modbus_data[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t num_modbus_registers = 0;

int8_t poll_state = 0;
bool communication_good = false;
uint8_t message_count = 0;

uint8_t failSafe;
uint16_t lostFrames = 0;

uint16_t telem_24v_scalar = 36680;

////////// Class Instantiations //////////
SBUS x8r(SBUS_HARDWARE_PORT);
Modbus slave(node_id, mobus_serial_port_number, 0); // 0 in thrid param means regular UART

void setup() {
    setup_hardware();
    x8r.begin();

    num_modbus_registers = sizeof(modbus_data) / sizeof(modbus_data[0]);
    slave.begin(115200);

    Serial.begin(9600);
}

void loop() {
    poll_sbus();
    poll_modbus();
    set_leds();
    poll_sensors();
}

void setup_hardware(){
    // Setup pins as inputs / outputs
    pinMode(HARDWARE::TELEM_24V, INPUT);
    pinMode(HARDWARE::TELEM_5V, INPUT);
    pinMode(HARDWARE::USB_TELEM_5V, INPUT);
    pinMode(HARDWARE::TELEM_3V3, INPUT);

    pinMode(HARDWARE::BAD_1, INPUT);
    pinMode(HARDWARE::BAD_2, INPUT);
    pinMode(HARDWARE::BAD_3, INPUT);
    pinMode(HARDWARE::BAD_4, INPUT);

    pinMode(HARDWARE::LED_RED, OUTPUT);
    pinMode(HARDWARE::LED_GREEN, OUTPUT);
    pinMode(HARDWARE::LED_BLUE, OUTPUT);

    pinMode(HARDWARE::LED_BLUE_EXTRA, OUTPUT);

    // Set default pin states
    digitalWrite(HARDWARE::LED_RED, LOW);
    digitalWrite(HARDWARE::LED_GREEN, HIGH);
    digitalWrite(HARDWARE::LED_BLUE, HIGH);

    digitalWrite(HARDWARE::LED_BLUE_EXTRA, LOW);

    // Set teensy to increased analog resolution
    analogReadResolution(13);
}


void poll_sbus(){
    x8r.read(&modbus_data[0], &failSafe, &lostFrames);
}

void poll_sensors(){
    modbus_data[MODBUS_REGISTERS::VOLTAGE_24] = telem_24v_scalar * (analogRead(HARDWARE::TELEM_24V) / 8192.0);

}

void poll_modbus(){
    poll_state = slave.poll(modbus_data, num_modbus_registers);
    communication_good = !slave.getTimeOutState();
}

void set_leds(){
    if(poll_state > 4){
        message_count++;
        if(message_count > 2){
            digitalWrite(HARDWARE::LED_BLUE_EXTRA, !digitalRead(HARDWARE::LED_BLUE_EXTRA));
            message_count = 0;
        }

        digitalWrite(HARDWARE::LED_GREEN, LOW);
        digitalWrite(HARDWARE::LED_RED, HIGH);
    }else if(!communication_good){
        digitalWrite(HARDWARE::LED_BLUE_EXTRA, LOW);
        digitalWrite(HARDWARE::LED_GREEN, HIGH);
        digitalWrite(HARDWARE::LED_RED, LOW);
    }
}
