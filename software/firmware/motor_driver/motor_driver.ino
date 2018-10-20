////////// Includes //////////
#include <ModbusRtu.h>

////////// Hardware / Data Enumerations //////////
enum HARDWARE {
    RS485_EN = 2,
    RS485_RX = 7,
    RS485_TX = 8,

    MOTOR_CURRENT_SENSE = A4,
    MOTOR_DIRECTION = 19,
    MOTOR_PWM = 20,
    MOTOR_SLEEP = 21,
    MOTOR_FAULT = 22,

    TEMP = A9,

    LED_RED = 1,
    LED_GREEN = 32,
    LED_BLUE = 6,

    LED_BLUE_EXTRA = 13
};

enum MODBUS_REGISTERS {
    DIRECTION = 0,  // Input
    SPEED = 1,      // Input
    SLEEP = 2,      // Input

    CURRENT = 3,    // Output
    FAULT = 4,      // Output

    TEMPERATURE = 5        // Output
};

////////// Global Variables //////////
const uint8_t node_id = 2;
const uint8_t mobus_serial_port_number = 3;

uint16_t modbus_data[] = {0, 0, 0, 0, 0, 0};
uint8_t num_modbus_registers = 0;

int8_t poll_state = 0;
bool communication_good = false;
uint8_t message_count = 0;

uint16_t rampdown_step = 2000;

////////// Class Instantiations //////////
Modbus slave(node_id, mobus_serial_port_number, HARDWARE::RS485_EN);

void setup() {
    setup_hardware();

    num_modbus_registers = sizeof(modbus_data) / sizeof(modbus_data[0]);
    slave.begin(115200); // baud-rate at 19200
    slave.setTimeOut(150);

    Serial.begin(9600);
}

void loop() {
    poll_modbus();
    set_leds();
    set_motor();
    poll_sensors_and_motor_state();
}

void setup_hardware(){
    // Setup pins as inputs / outputs
    pinMode(HARDWARE::RS485_EN, OUTPUT);

    pinMode(HARDWARE::MOTOR_CURRENT_SENSE, INPUT);
    pinMode(HARDWARE::MOTOR_DIRECTION, OUTPUT);
    pinMode(HARDWARE::MOTOR_PWM, OUTPUT);
    pinMode(HARDWARE::MOTOR_SLEEP, OUTPUT);
    pinMode(HARDWARE::MOTOR_FAULT, INPUT);

    pinMode(HARDWARE::TEMP, INPUT);

    pinMode(HARDWARE::LED_RED, OUTPUT);
    pinMode(HARDWARE::LED_GREEN, OUTPUT);
    pinMode(HARDWARE::LED_BLUE, OUTPUT);

    pinMode(HARDWARE::LED_BLUE_EXTRA, OUTPUT);

    // Set default pin states
    digitalWrite(HARDWARE::MOTOR_SLEEP, HIGH);

    digitalWrite(HARDWARE::LED_RED, LOW);
    digitalWrite(HARDWARE::LED_GREEN, HIGH);
    digitalWrite(HARDWARE::LED_BLUE, HIGH);

    digitalWrite(HARDWARE::LED_BLUE_EXTRA, LOW);

    // Set the PWM resolution to 16-bits
    analogWriteResolution(16);

    // Change motor PWM frequency so it's not in the audible range
    analogWriteFrequency(HARDWARE::MOTOR_PWM, 25000);

    // Set teensy to increased analog resolution
    analogReadResolution(13);
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

void set_motor(){
    if(communication_good){
        digitalWrite(HARDWARE::MOTOR_DIRECTION, modbus_data[MODBUS_REGISTERS::DIRECTION]);
        analogWrite(HARDWARE::MOTOR_PWM, modbus_data[MODBUS_REGISTERS::SPEED]);
        digitalWrite(HARDWARE::MOTOR_SLEEP, modbus_data[MODBUS_REGISTERS::SLEEP]);
    }else{
        while(modbus_data[MODBUS_REGISTERS::SPEED] != 0 && modbus_data[MODBUS_REGISTERS::SPEED] > rampdown_step){
            modbus_data[MODBUS_REGISTERS::SPEED] -= rampdown_step;
            analogWrite(HARDWARE::MOTOR_PWM, modbus_data[MODBUS_REGISTERS::SPEED]);
            delay(2);
        }

        modbus_data[MODBUS_REGISTERS::SPEED] = 0;
        analogWrite(HARDWARE::MOTOR_PWM, modbus_data[MODBUS_REGISTERS::SPEED]);
    }

}

void poll_sensors_and_motor_state(){
    // Not the most elegant calculations, could clean up.
    modbus_data[MODBUS_REGISTERS::CURRENT] = (uint16_t)(((((analogRead(HARDWARE::MOTOR_CURRENT_SENSE) / 8192.0) * 3.3) - 0.05) / 0.02) * 1000);
    modbus_data[MODBUS_REGISTERS::FAULT] = !digitalRead(HARDWARE::MOTOR_FAULT);
    modbus_data[MODBUS_REGISTERS::TEMPERATURE] = (uint16_t)(((((analogRead(HARDWARE::TEMP) / 8192.0) * 3.3) - 0.750) / 0.01) * 1000);

    Serial.println(modbus_data[MODBUS_REGISTERS::CURRENT]);
}
