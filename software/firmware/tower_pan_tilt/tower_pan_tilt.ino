////////// Includes //////////
#include <ModbusRtu.h>
#include <Servo.h>

////////// Hardware / Data Enumerations //////////
enum HARDWARE {
  RS485_EN = 2,
  RS485_RX = 7,
  RS485_TX = 8,

  SERVO_PAN = 5,
  SERVO_TILT = 4,

  LED_RED = 1,
  LED_GREEN = 32,
  LED_BLUE = 6,

  LED_BLUE_EXTRA = 13
};

enum MODBUS_REGISTERS {
  CENTER_ALL = 0,           // Input/Output
  PAN_ADJUST_POSITIVE = 1,  // Input/Output
  PAN_ADJUST_NEGATIVE = 2,  // Input/Output
  TILT_ADJUST_POSITIVE = 3, // Input/Output
  TILT_ADJUST_NEGATIVE = 4, // Input/Output
};

////////// Global Variables //////////
const uint8_t node_id = 2;
const uint8_t mobus_serial_port_number = 3;

uint16_t modbus_data[] = {0, 0, 0, 0, 0};
uint8_t num_modbus_registers = 0;

int8_t poll_state = 0;
bool communication_good = false;
uint8_t message_count = 0;

// Pan/tilt hard limits
const int pan_min = 1470;
const int pan_center = 1605;
const int pan_max = 1725;

const int tilt_min = 1020;
const int tilt_center = 1820;
const int tilt_max = 2400;

// Pan/tilt positions
int pan_position = pan_center;
int tilt_position = tilt_center;

////////// Class Instantiations //////////
Modbus slave(node_id, mobus_serial_port_number, HARDWARE::RS485_EN);

Servo pan_servo;
Servo tilt_servo;

void setup() {
 Serial.begin(9600);
//  while(!Serial);
  setup_hardware();

  num_modbus_registers = sizeof(modbus_data) / sizeof(modbus_data[0]);

  slave.begin(115200);
  slave.setTimeOut(150);

}

void loop() {
  poll_modbus();
  set_leds();
  set_pan_tilt_adjustments();
}

void setup_hardware() {
  // Setup pins as inputs / outputs
  pinMode(HARDWARE::RS485_EN, OUTPUT);

  pinMode(HARDWARE::SERVO_PAN, OUTPUT);
  pinMode(HARDWARE::SERVO_TILT, OUTPUT);

  pan_servo.attach(HARDWARE::SERVO_PAN);
  tilt_servo.attach(HARDWARE::SERVO_TILT);

  pan_servo.writeMicroseconds(pan_center);
  tilt_servo.writeMicroseconds(tilt_center);

  pinMode(HARDWARE::LED_RED, OUTPUT);
  pinMode(HARDWARE::LED_GREEN, OUTPUT);
  pinMode(HARDWARE::LED_BLUE, OUTPUT);

  pinMode(HARDWARE::LED_BLUE_EXTRA, OUTPUT);

  // Set default pin states
  digitalWrite(HARDWARE::LED_RED, LOW);
  digitalWrite(HARDWARE::LED_GREEN, HIGH);
  digitalWrite(HARDWARE::LED_BLUE, HIGH);

  digitalWrite(HARDWARE::LED_BLUE_EXTRA, LOW);
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

    digitalWrite(HARDWARE::LED_GREEN, LOW);
    digitalWrite(HARDWARE::LED_RED, HIGH);
  } else if (!communication_good) {
    digitalWrite(HARDWARE::LED_BLUE_EXTRA, LOW);
    digitalWrite(HARDWARE::LED_GREEN, HIGH);
    digitalWrite(HARDWARE::LED_RED, LOW);
  }
}

void set_pan_tilt_adjustments() {
  if (communication_good) {
    if (modbus_data[MODBUS_REGISTERS::CENTER_ALL]) {
      pan_servo.writeMicroseconds(constrain(pan_position, pan_min, pan_max));
      tilt_servo.writeMicroseconds(constrain(tilt_position, tilt_min, tilt_max));

      pan_position = pan_center;
      tilt_position = tilt_center;

      modbus_data[MODBUS_REGISTERS::CENTER_ALL] = 0;
    }

    pan_position = constrain(pan_position - modbus_data[MODBUS_REGISTERS::PAN_ADJUST_POSITIVE] + modbus_data[MODBUS_REGISTERS::PAN_ADJUST_NEGATIVE], pan_min, pan_max);
    tilt_position = constrain(tilt_position + modbus_data[MODBUS_REGISTERS::TILT_ADJUST_POSITIVE] - modbus_data[MODBUS_REGISTERS::TILT_ADJUST_NEGATIVE], tilt_min, tilt_max);

    pan_servo.writeMicroseconds(pan_position);
    tilt_servo.writeMicroseconds(tilt_position);

//    Serial.print(pan_position);
//    Serial.print("\t");
//    Serial.println(tilt_position);

    modbus_data[MODBUS_REGISTERS::PAN_ADJUST_POSITIVE] = 0;
    modbus_data[MODBUS_REGISTERS::PAN_ADJUST_NEGATIVE] = 0;
    modbus_data[MODBUS_REGISTERS::TILT_ADJUST_POSITIVE] = 0;
    modbus_data[MODBUS_REGISTERS::TILT_ADJUST_NEGATIVE] = 0;
  }
}
