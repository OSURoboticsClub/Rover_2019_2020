////////// Includes //////////
#include "HX711.h"
#include <ModbusRtu.h>
#include <camera.h>

////////// Hardware / Data Enumerations //////////
enum HARDWARE {
  RS485_EN = 6,
  RS485_RX = 9,
  RS485_TX = 10,

  MOTOR_LIFT_A = 27,
  MOTOR_LIFT_B = 28,
  MOTOR_LIFT_PWM = 25,
  MOTOR_LIFT_CS = 31,
  MOTOR_LIFT_EN = 24,
  MOTOR_LIFT_FB = A10,

  MOTOR_TILT_A = 30,
  MOTOR_TILT_B = 29,
  MOTOR_TILT_PWM = 32,
  MOTOR_TILT_CS = 26,
  MOTOR_TILT_EN = 33,
  MOTOR_TILT_FB = A11,

  CAMERA_CONTROL = 0,
  CAMERA_VIDEO_EN = 1,

  LED_13 = 13,

  LED_RED = 20,
  LED_BLUE = 21,
  LED_GREEN = 22,

  SCALE_DOUT = 8,
  SCALE_CLK = 7

};

enum MOTORS {
  LIFT = 0,
  TILT = 1,
};

// void shoot();
// void slowZoomIn();
// void slowZoomOut();
// void fullZoomIn();
// void fullZoomOut();
// void focus();

enum MODBUS_REGISTERS {
    // Inputs
    SET_POSITION_LIFT_POSITIVE = 0,
    SET_POSITION_LIFT_NEGATIVE = 1,
    SET_POSITION_TILT_POSITIVE = 2,
    SET_POSITION_TILT_NEGATIVE = 3,
    SET_POSITION_LIFT_ABSOLUTE = 4,
    SET_POSITION_TILT_ABSOLUTE = 5,
    MEASURE = 6,
    TARE = 7,
    CALIBRATION_FACTOR = 8,

    CHANGE_VIEW_MODE = 9,
    ZOOM_IN = 10,
    ZOOM_OUT = 11,
    FULL_ZOOM_IN = 12,
    FULL_ZOOM_OUT = 13,
    SHOOT = 14,

    // Outputs
    CURRENT_POSITION_LIFT = 15,
    CURRENT_POSITION_TILT = 16,
    MEASURED_WEIGHT = 17
};

enum CAMERA_VIEW_MODES {
        NO_CHANGE = 0,
        LCD = 1,
        NETWORK = 2
};

////////// Global Variables //////////
int set_position_lift = 1023;
int set_position_tilt = 350;
int current_position_lift = 0;
int current_position_tilt = 0;
int tolerance = 20; //tolerance for position

float last_calibration_factor = -120000; //for the scale

// modbus stuff
const uint8_t node_id = 2;
const uint8_t mobus_serial_port_number = 2;
uint16_t modbus_data[] = {0, 0, 0, 0, 9999, 9999, 0, 0, 895, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t num_modbus_registers = 0;
int8_t poll_state = 0;
bool communication_good = false;
uint8_t message_count = 0;

// nice human words for motor states
#define BRAKEVCC  0
#define CW        1
#define CCW       2
#define BRAKEGND  3

////////// Class Instantiations //////////
HX711 scale(HARDWARE::SCALE_DOUT, HARDWARE::SCALE_CLK);
Modbus slave(node_id, mobus_serial_port_number, HARDWARE::RS485_EN);
camera science_camera(HARDWARE::CAMERA_CONTROL);

void setup() {
  Serial.begin(9600); // debug
  // while(!Serial);

  setup_hardware();
  num_modbus_registers = sizeof(modbus_data) / sizeof(modbus_data[0]);
  slave.begin(115200); // baud-rate at 19200
  slave.setTimeOut(200);
}

void loop() {
  poll_modbus();
  set_leds();
  set_motors();
  set_scale();
  poll_scale();
  apply_camera_commands();
}

void setup_hardware() {
  pinMode(HARDWARE::RS485_EN, OUTPUT);

  pinMode(HARDWARE::MOTOR_LIFT_A, OUTPUT);
  pinMode(HARDWARE::MOTOR_LIFT_B, OUTPUT);
  pinMode(HARDWARE::MOTOR_LIFT_PWM, OUTPUT);
  pinMode(HARDWARE::MOTOR_LIFT_EN, OUTPUT);
  pinMode(HARDWARE::MOTOR_LIFT_CS, INPUT);
  pinMode(HARDWARE::MOTOR_LIFT_FB, INPUT);

  pinMode(HARDWARE::MOTOR_TILT_A, OUTPUT);
  pinMode(HARDWARE::MOTOR_TILT_B, OUTPUT);
  pinMode(HARDWARE::MOTOR_TILT_PWM, OUTPUT);
  pinMode(HARDWARE::MOTOR_TILT_EN, OUTPUT);
  pinMode(HARDWARE::MOTOR_TILT_CS, INPUT);
  pinMode(HARDWARE::MOTOR_TILT_FB, INPUT);


  pinMode(HARDWARE::CAMERA_CONTROL, OUTPUT);
  pinMode(HARDWARE::CAMERA_VIDEO_EN, OUTPUT);

  pinMode(HARDWARE::LED_13, OUTPUT);
  pinMode(HARDWARE::LED_RED, OUTPUT);
  pinMode(HARDWARE::LED_BLUE, OUTPUT);
  pinMode(HARDWARE::LED_GREEN, OUTPUT);

  // set defualt states
  digitalWrite(HARDWARE::LED_RED, LOW);
  digitalWrite(HARDWARE::LED_GREEN, HIGH);
  digitalWrite(HARDWARE::LED_BLUE, HIGH);
  digitalWrite(HARDWARE::LED_13, LOW);
  digitalWrite(HARDWARE::MOTOR_LIFT_EN, HIGH);
  digitalWrite(HARDWARE::MOTOR_TILT_EN, HIGH);

  digitalWrite(HARDWARE::CAMERA_VIDEO_EN, LOW);

  // Change motor PWM frequency so it's not in the audible range
  analogWriteFrequency(HARDWARE::MOTOR_LIFT_PWM, 25000);
  analogWriteFrequency(HARDWARE::MOTOR_TILT_PWM, 25000);

  // set the current desired position to the current position
  // set_position_lift = analogRead(HARDWARE::MOTOR_LIFT_FB);
  // set_position_tilt = analogRead(HARDWARE::MOTOR_TILT_FB);
  current_position_lift = analogRead(HARDWARE::MOTOR_LIFT_FB);
  current_position_tilt = analogRead(HARDWARE::MOTOR_TILT_FB);

  // setup scale
  scale.set_scale();
  scale.tare();	//Reset the scale to 0
}

void poll_modbus(){
    poll_state = slave.poll(modbus_data, num_modbus_registers);
    communication_good = !slave.getTimeOutState();
}

void set_leds(){
    if(poll_state > 4){
        message_count++;
        if(message_count > 2){
            digitalWrite(HARDWARE::LED_13, !digitalRead(HARDWARE::LED_13));
            message_count = 0;
        }

        digitalWrite(HARDWARE::LED_GREEN, LOW);
        digitalWrite(HARDWARE::LED_RED, HIGH);
    }else if(!communication_good){
        digitalWrite(HARDWARE::LED_13, LOW);
        digitalWrite(HARDWARE::LED_GREEN, HIGH);
        digitalWrite(HARDWARE::LED_RED, LOW);
    }
}

void set_motors() {
    if (modbus_data[MODBUS_REGISTERS::SET_POSITION_LIFT_ABSOLUTE] < 1024 ){
        set_position_lift = modbus_data[MODBUS_REGISTERS::SET_POSITION_LIFT_ABSOLUTE];
        modbus_data[MODBUS_REGISTERS::SET_POSITION_LIFT_ABSOLUTE] = 1024;
    }else{
        set_position_lift += modbus_data[MODBUS_REGISTERS::SET_POSITION_LIFT_POSITIVE] - modbus_data[MODBUS_REGISTERS::SET_POSITION_LIFT_NEGATIVE];
        set_position_lift = constrain(set_position_lift, 0, 1023);

        modbus_data[MODBUS_REGISTERS::SET_POSITION_LIFT_POSITIVE] = 0;
        modbus_data[MODBUS_REGISTERS::SET_POSITION_LIFT_NEGATIVE] = 0;
    }

    if(modbus_data[MODBUS_REGISTERS::SET_POSITION_TILT_ABSOLUTE] < 1024){
        set_position_tilt = modbus_data[MODBUS_REGISTERS::SET_POSITION_TILT_ABSOLUTE];
        modbus_data[MODBUS_REGISTERS::SET_POSITION_TILT_ABSOLUTE] = 1024;
    }else{
        set_position_tilt += modbus_data[MODBUS_REGISTERS::SET_POSITION_TILT_POSITIVE] - modbus_data[MODBUS_REGISTERS::SET_POSITION_TILT_NEGATIVE] ;
        set_position_tilt = constrain(set_position_tilt, 0, 1023);

        modbus_data[MODBUS_REGISTERS::SET_POSITION_TILT_POSITIVE] = 0;
        modbus_data[MODBUS_REGISTERS::SET_POSITION_TILT_NEGATIVE] = 0;
    }

  current_position_lift = analogRead(HARDWARE::MOTOR_LIFT_FB);
  current_position_tilt = analogRead(HARDWARE::MOTOR_TILT_FB);

  modbus_data[MODBUS_REGISTERS::CURRENT_POSITION_LIFT] = current_position_lift;
  modbus_data[MODBUS_REGISTERS::CURRENT_POSITION_TILT] = current_position_tilt;

  if (abs(current_position_lift - set_position_lift) > tolerance) {
    if (current_position_lift < set_position_lift) {
      set_motor_output(MOTORS::LIFT, CCW, 255);
    }
    else {
      set_motor_output(MOTORS::LIFT, CW, 255);
    }
  }
  else {
    motor_off(MOTORS::LIFT);
  }

  if (abs(current_position_tilt - set_position_tilt) > tolerance) {
    if (current_position_tilt < set_position_tilt) {
      set_motor_output(MOTORS::TILT, CCW, 255);
    }
    else {
      set_motor_output(MOTORS::TILT, CW, 255);
    }
  }
  else {
    motor_off(MOTORS::TILT);
  }
}

void set_scale(){
    float cal_factor = modbus_data[MODBUS_REGISTERS::CALIBRATION_FACTOR] * 100;
    if(cal_factor != last_calibration_factor){
        scale.set_scale(cal_factor);
        last_calibration_factor = cal_factor;
    }

    if (modbus_data[MODBUS_REGISTERS::TARE] == 1){
        scale.tare();
        modbus_data[MODBUS_REGISTERS::TARE] = 0;
    }
}

void poll_scale(){
    if(modbus_data[MODBUS_REGISTERS::MEASURE] == 1){
        Serial.println(scale.get_units()*-1000);
        modbus_data[MODBUS_REGISTERS::MEASURED_WEIGHT] = scale.get_units()*-1000;
        modbus_data[MODBUS_REGISTERS::MEASURE] = 0;
    }
}

void apply_camera_commands(){
    ///// Camea video output control /////
    switch (modbus_data[MODBUS_REGISTERS::CHANGE_VIEW_MODE]) {
        case CAMERA_VIEW_MODES::LCD:
            digitalWrite(HARDWARE::CAMERA_VIDEO_EN, HIGH);
            break;
        case CAMERA_VIEW_MODES::NETWORK:
            digitalWrite(HARDWARE::CAMERA_VIDEO_EN, LOW);
            break;
        default:
            break;
    }
    modbus_data[MODBUS_REGISTERS::CHANGE_VIEW_MODE] = CAMERA_VIEW_MODES::NO_CHANGE;

    ////// Zoom controls, process all, even if conflicting ///////
    if(modbus_data[MODBUS_REGISTERS::ZOOM_IN]){
        science_camera.slowZoomIn();
        modbus_data[MODBUS_REGISTERS::ZOOM_IN] = 0;
    }else if(modbus_data[MODBUS_REGISTERS::ZOOM_OUT]){
        science_camera.slowZoomOut();
        modbus_data[MODBUS_REGISTERS::ZOOM_OUT] = 0;
    }else if(modbus_data[MODBUS_REGISTERS::FULL_ZOOM_IN]){
        science_camera.fullZoomIn();
        modbus_data[MODBUS_REGISTERS::FULL_ZOOM_IN] = 0;
    }else if(modbus_data[MODBUS_REGISTERS::FULL_ZOOM_OUT]){
        science_camera.fullZoomOut();
        modbus_data[MODBUS_REGISTERS::FULL_ZOOM_OUT] = 0;
    } else if(modbus_data[MODBUS_REGISTERS::SHOOT]){
        science_camera.shoot();
        modbus_data[MODBUS_REGISTERS::SHOOT] = 0;
    }
    ///// camera process here
    science_camera.update();
}

//---Set Motor Output---//
/*
  Inputs: motor number, direction, pwm value
  Returns: nothing

  Will set a motor going in a specific direction the motor will continue
  going in that direction, at that speed until told to do otherwise.

  direct: Should be between 0 and 3, with the following result
  0: Brake to VCC
  1: Clockwise
  2: CounterClockwise
  3: Brake to GND

  pwm: should be a value between 0 and 255, higher the number, the faster
  it'll go
  ----------------
  Control Logic:
  ----------------
	         	 A | B
  Brake VCC: 1   1
	       CW: 1   0
	      CCW: 0   1
  Brake GND: 0   0
  ----------------
*/
void set_motor_output(int motor, int direction, int pwm_input) {

  int a;
  int b;
  int pwm;

  if (motor == MOTORS::LIFT) {
    a = HARDWARE::MOTOR_LIFT_A;
    b = HARDWARE::MOTOR_LIFT_B;
    pwm = HARDWARE::MOTOR_LIFT_PWM;
  }
  else if (motor == MOTORS::TILT) {
    a = HARDWARE::MOTOR_TILT_A;
    b = HARDWARE::MOTOR_TILT_B;
    pwm = HARDWARE::MOTOR_TILT_PWM;
  }
  else {
    return;
  }

  if (direction <= 4) {
    // Set A
    if (direction <= 1) {
      digitalWrite(a, HIGH);
    }
    else {
      digitalWrite(a, LOW);
    }

    // Set B
    if ((direction == 0) || (direction == 2)) {
      digitalWrite(b, HIGH);
    }
    else {
      digitalWrite(b, LOW);
    }
    analogWrite(pwm, pwm_input);
  }
}

void motor_off(int motor) {
  int a;
  int b;
  int pwm;

  if (motor == MOTORS::LIFT) {
    a = HARDWARE::MOTOR_LIFT_A;
    b = HARDWARE::MOTOR_LIFT_B;
    pwm = HARDWARE::MOTOR_LIFT_PWM;
  }
  else if (motor == MOTORS::TILT) {
    a = HARDWARE::MOTOR_TILT_A;
    b = HARDWARE::MOTOR_TILT_B;
    pwm = HARDWARE::MOTOR_TILT_PWM;
  }
  else {
    return;
  }

  digitalWrite(a, LOW);
  digitalWrite(b, LOW);
  analogWrite(pwm, 0);
}
