////////// Includes //////////
#include <PID_v1.h>
#include <Encoder.h>
#include <ModbusRtu.h>

////////// Hardware / Data Enumerations //////////
enum HARDWARE {
    RS485_EN = 14,
    RS485_RX = 7,
    RS485_TX = 8,

    // Motor 1
    PINCH_MOTOR_PWM_1 = 25,
    PINCH_MOTOR_PWM_2 = 22,
    PINCH_ENCODER_A = 2,
    PINCH_ENCODER_B = 30,
    PINCH_MOTOR_CURRENT_SENSE = A2,

    // Motor 2
    FOREFINGER_MOTOR_PWM_1 = 23,
    FOREFINGER_MOTOR_PWM_2 = 9,
    FOREFINGER_ENCODER_A = 29,
    FOREFINGER_ENCODER_B = 27,
    FOREFINGER_MOTOR_CURRENT_SENSE = A3,

    // Motor 3
    THUMB_MOTOR_PWM_1 = 10,
    THUMB_MOTOR_PWM_2 = 20,
    THUMB_ENCODER_A = 28,
    THUMB_ENCODER_B = 12,
    THUMB_MOTOR_CURRENT_SENSE = A5,

    // Motor 4
    MIDDLEFINGER_MOTOR_PWM_1 = 21,
    MIDDLEFINGER_MOTOR_PWM_2 = 5,
    MIDDLEFINGER_ENCODER_A = 11,
    MIDDLEFINGER_ENCODER_B = 13,
    MIDDLEFINGER_MOTOR_CURRENT_SENSE = A4,

    // LEDs
    LED_RED = 6,
    LED_GREEN = 32,
    LED_BLUE = 13,
    WORKLIGHT = 15
};

enum MG_INDEX {
    PINCH = 0,
    FOREFINGER = 1,
    THUMB = 2,
    MIDDLEFINGER = 3
};

enum MODES {
    NO_CHANGE = 0,
    NORMAL = 1,
    TWO_FINGER_PINCH = 2,
    WIDE = 3,
    SCISSOR = 4
};

struct MOTOR_GROUP {
        int PWM_1_PIN;
        int PWM_2_PIN;
        int CURRENT_PIN;
        Encoder *ENC;
        PID *PID_CONTROL;
        double INPUT_POS;
        double OUTPUT_POS;
        double SETPOINT_POS;
        int CURRENT_READING;
        static const int NUM_READINGS = 20;
        int READINGS[MOTOR_GROUP::NUM_READINGS];
        int READ_INDEX = 0;
        int TOTAL = 0;
        int TRIP_THRESHOLD;
        int HOME_BACKOFF;
        bool IS_HOMING = false;
        int GRIP_THRESHOLD;
        int TRAVEL_MAX;
        int TRAVEL_MIN;
        int LAST_GOOD_POSITION;
        bool LAST_GOOD_POSITION_SET = false;
};

struct MOTOR_GROUP motor_groups[] = {
        {HARDWARE::PINCH_MOTOR_PWM_1,
        HARDWARE::PINCH_MOTOR_PWM_2,
        HARDWARE::PINCH_MOTOR_CURRENT_SENSE},

        {HARDWARE::FOREFINGER_MOTOR_PWM_1,
        HARDWARE::FOREFINGER_MOTOR_PWM_2,
        HARDWARE::FOREFINGER_MOTOR_CURRENT_SENSE},

        {HARDWARE::THUMB_MOTOR_PWM_1,
        HARDWARE::THUMB_MOTOR_PWM_2,
        HARDWARE::THUMB_MOTOR_CURRENT_SENSE},

        {HARDWARE::MIDDLEFINGER_MOTOR_PWM_1,
        HARDWARE::MIDDLEFINGER_MOTOR_PWM_2,
        HARDWARE::MIDDLEFINGER_MOTOR_CURRENT_SENSE}
};

enum MODBUS_REGISTERS {
    // Inputs
    MODE = 0, // if this is zero don't apply position updates, don't make changes
    FINGER_POSITION = 1,
    HOME = 2,
    LIGHT_STATE = 3,

    // Outputs
    CURRENT_PINCH = 4,
    CURRENT_FOREFINGER = 5,
    CURRENT_THUMB = 6,
    CURRENT_MIDDLEFINGER = 7,
    POSITION_PINCH = 8,
    POSITION_FOREFINGER = 9,
    POSITION_THUMB = 10,
    POSITION_MIDDLEFINGER = 11,
    LIGHT_STATE_OUPUT = 12,
    MODE_OUTPUT = 13,
    FINGER_POSITION_OUTPUT = 14,
};

////////// Global Variables //////////
// Modbus stuff
const uint8_t node_id = 1;
const uint8_t mobus_serial_port_number = 3;
uint16_t modbus_data[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t num_modbus_registers = 0;
int8_t poll_state = 0;
bool communication_good = false;
uint8_t message_count = 0;

// Define nice words for motor states
#define BRAKE 0
#define FWD   1
#define REV   2
#define COAST 3

// PID gains
double kp = 5;
double ki = 0;
double kd = 0;

// Timing
long prev_millis = 0;
unsigned int interval = 10;

// Current offset
int current_midpoint = 375;

// Joystick
int joy_value = 0;
int mid_value = 290;
int deadzone = 10;
int scale = 4;
long joy_interval = 10;

// Limits
int universal_max = 6750;
int universal_min = -3000;

int two_finger_pinch_travel_min = 0;
int two_finger_pinch_travel_max = 3000;

int two_finger_pinch_setpoint = 3000;
int normal_setpoint = 3000;
int wide_setpoint = 2000;
int scissor_setpoint = 0;

// Misc
int universal_position = 0;
int backoff = 50;
long curr_millis = 0;

// Local modbus variables
bool global_home = false;
bool global_home_started = false;
bool pinch_home_started = false;
uint16_t global_state = 0;
uint16_t modbus_position = 0;
bool modbus_light_state = false;

////////// Class Instantiations //////////
// Encoder
Encoder encoder_pinch(HARDWARE::PINCH_ENCODER_A, HARDWARE::PINCH_ENCODER_B);
Encoder encoder_forefinger(HARDWARE::FOREFINGER_ENCODER_A, HARDWARE::FOREFINGER_ENCODER_B);
Encoder encoder_thumb(HARDWARE::THUMB_ENCODER_A, HARDWARE::THUMB_ENCODER_B);
Encoder encoder_middlefinger(HARDWARE::MIDDLEFINGER_ENCODER_A, HARDWARE::MIDDLEFINGER_ENCODER_B);

// PID
PID motor_pid_pinch(&motor_groups[MG_INDEX::PINCH].INPUT_POS, &motor_groups[MG_INDEX::PINCH].OUTPUT_POS, &motor_groups[MG_INDEX::PINCH].SETPOINT_POS, kp, ki, kd, DIRECT);
PID motor_pid_forefinger(&motor_groups[MG_INDEX::FOREFINGER].INPUT_POS, &motor_groups[MG_INDEX::FOREFINGER].OUTPUT_POS, &motor_groups[MG_INDEX::FOREFINGER].SETPOINT_POS, kp, ki, kd, DIRECT);
PID motor_pid_thumb(&motor_groups[MG_INDEX::THUMB].INPUT_POS, &motor_groups[MG_INDEX::THUMB].OUTPUT_POS, &motor_groups[MG_INDEX::THUMB].SETPOINT_POS, kp, ki, kd, DIRECT);
PID motor_pid_middlefinger(&motor_groups[MG_INDEX::MIDDLEFINGER].INPUT_POS, &motor_groups[MG_INDEX::MIDDLEFINGER].OUTPUT_POS, &motor_groups[MG_INDEX::MIDDLEFINGER].SETPOINT_POS, kp, ki, kd, DIRECT);

// Modbus
Modbus slave(node_id, mobus_serial_port_number, HARDWARE::RS485_EN);

////////// Setup //////////
void setup() {
  setup_hardware();

  num_modbus_registers = sizeof(modbus_data) / sizeof(modbus_data[0]);
  slave.begin(115200);
  slave.setTimeOut(150);

  Serial.begin(9600);

  motor_groups[MG_INDEX::PINCH].ENC = &encoder_pinch;
  motor_groups[MG_INDEX::FOREFINGER].ENC = &encoder_forefinger;
  motor_groups[MG_INDEX::THUMB].ENC = &encoder_thumb;
  motor_groups[MG_INDEX::MIDDLEFINGER].ENC = &encoder_middlefinger;

  motor_groups[MG_INDEX::PINCH].PID_CONTROL = &motor_pid_pinch;
  motor_pid_pinch.SetMode(AUTOMATIC);
  motor_pid_pinch.SetSampleTime(1);

  motor_groups[MG_INDEX::FOREFINGER].PID_CONTROL = &motor_pid_forefinger;
  motor_pid_forefinger.SetMode(AUTOMATIC);
  motor_pid_forefinger.SetSampleTime(1);

  motor_groups[MG_INDEX::THUMB].PID_CONTROL = &motor_pid_thumb;
  motor_pid_thumb.SetMode(AUTOMATIC);
  motor_pid_thumb.SetSampleTime(1);

  motor_groups[MG_INDEX::MIDDLEFINGER].PID_CONTROL = &motor_pid_middlefinger;
  motor_pid_middlefinger.SetMode(AUTOMATIC);
  motor_pid_middlefinger.SetSampleTime(1);

  // initialize all the readings to 0:
  for (int i = 0; i < MOTOR_GROUP::NUM_READINGS; i++) {
    motor_groups[0].READINGS[i] = 0;
    motor_groups[1].READINGS[i] = 0;
    motor_groups[2].READINGS[i] = 0;
    motor_groups[3].READINGS[i] = 0;
  }

  motor_groups[MG_INDEX::PINCH].TRIP_THRESHOLD = 200;
  motor_groups[MG_INDEX::PINCH].HOME_BACKOFF = 200;
  motor_groups[MG_INDEX::PINCH].GRIP_THRESHOLD = -100;
  motor_groups[MG_INDEX::PINCH].TRAVEL_MAX = 6750;
  motor_groups[MG_INDEX::PINCH].TRAVEL_MIN = 0;
  motor_groups[MG_INDEX::PINCH].LAST_GOOD_POSITION = 0;

  motor_groups[MG_INDEX::FOREFINGER].TRIP_THRESHOLD = 200;
  motor_groups[MG_INDEX::FOREFINGER].HOME_BACKOFF = 200;
  motor_groups[MG_INDEX::FOREFINGER].GRIP_THRESHOLD = -100;
  motor_groups[MG_INDEX::FOREFINGER].TRAVEL_MAX = 6750;
  motor_groups[MG_INDEX::FOREFINGER].TRAVEL_MIN = 0;
  motor_groups[MG_INDEX::FOREFINGER].LAST_GOOD_POSITION = 0;

  motor_groups[MG_INDEX::THUMB].TRIP_THRESHOLD = 200;
  motor_groups[MG_INDEX::THUMB].HOME_BACKOFF = 200;
  motor_groups[MG_INDEX::THUMB].GRIP_THRESHOLD = -100;
  motor_groups[MG_INDEX::THUMB].TRAVEL_MAX = 6750;
  motor_groups[MG_INDEX::THUMB].TRAVEL_MIN = 0;
  motor_groups[MG_INDEX::THUMB].LAST_GOOD_POSITION = 0;

  motor_groups[MG_INDEX::MIDDLEFINGER].TRIP_THRESHOLD = 200;
  motor_groups[MG_INDEX::MIDDLEFINGER].HOME_BACKOFF = 200;
  motor_groups[MG_INDEX::MIDDLEFINGER].GRIP_THRESHOLD = -100;
  motor_groups[MG_INDEX::MIDDLEFINGER].TRAVEL_MAX = 6750;
  motor_groups[MG_INDEX::MIDDLEFINGER].TRAVEL_MIN = 0;
  motor_groups[MG_INDEX::MIDDLEFINGER].LAST_GOOD_POSITION = 0;

}

////////// Loop //////////
void loop() {
    poll_modbus();
    poll_sensors();
    set_leds();
    home_routine();
    set_motor_states();
}

void setup_hardware(){
  // setup IO
  pinMode(HARDWARE::PINCH_MOTOR_PWM_1, OUTPUT);
  pinMode(HARDWARE::PINCH_MOTOR_PWM_2, OUTPUT);
  pinMode(HARDWARE::PINCH_MOTOR_CURRENT_SENSE, INPUT);

  pinMode(HARDWARE::FOREFINGER_MOTOR_PWM_1, OUTPUT);
  pinMode(HARDWARE::FOREFINGER_MOTOR_PWM_2, OUTPUT);
  pinMode(HARDWARE::FOREFINGER_MOTOR_CURRENT_SENSE, INPUT);

  pinMode(HARDWARE::THUMB_MOTOR_PWM_1, OUTPUT);
  pinMode(HARDWARE::THUMB_MOTOR_PWM_2, OUTPUT);
  pinMode(HARDWARE::THUMB_MOTOR_CURRENT_SENSE, INPUT);

  pinMode(HARDWARE::MIDDLEFINGER_MOTOR_PWM_1, OUTPUT);
  pinMode(HARDWARE::MIDDLEFINGER_MOTOR_PWM_2, OUTPUT);
  pinMode(HARDWARE::MIDDLEFINGER_MOTOR_CURRENT_SENSE, INPUT);

  pinMode(HARDWARE::LED_RED, OUTPUT);
  pinMode(HARDWARE::LED_GREEN, OUTPUT);
  pinMode(HARDWARE::LED_BLUE, OUTPUT);

  // setup default states
  set_motor_output(MG_INDEX::PINCH, BRAKE, 0);
  set_motor_output(MG_INDEX::FOREFINGER, BRAKE, 0);
  set_motor_output(MG_INDEX::THUMB, BRAKE, 0);
  set_motor_output(MG_INDEX::MIDDLEFINGER, BRAKE, 0);

  digitalWrite(HARDWARE::LED_RED, HIGH);
  digitalWrite(HARDWARE::LED_GREEN, HIGH);
  digitalWrite(HARDWARE::LED_BLUE, HIGH);

  analogReadAveraging(32);
}

void poll_modbus(){
    poll_state = slave.poll(modbus_data, num_modbus_registers);
    communication_good = !slave.getTimeOutState();

    // Set modes
    if (modbus_data[MODBUS_REGISTERS::MODE] != MODES::NO_CHANGE){
        global_state = modbus_data[MODBUS_REGISTERS::MODE];
        modbus_position = modbus_data[MODBUS_REGISTERS::FINGER_POSITION];
        global_home = modbus_data[MODBUS_REGISTERS::HOME];
        global_home_started = global_home;
        modbus_light_state = modbus_data[MODBUS_REGISTERS::LIGHT_STATE];

        // when shit dies
        modbus_data[MODBUS_REGISTERS::LIGHT_STATE_OUPUT] = modbus_light_state;
        modbus_data[MODBUS_REGISTERS::MODE_OUTPUT] = global_state;
        modbus_data[MODBUS_REGISTERS::FINGER_POSITION_OUTPUT] = modbus_position;
    }
}

void set_leds(){
    if(poll_state > 4){
        message_count++;
        if(message_count > 2){
            digitalWrite(HARDWARE::LED_BLUE, !digitalRead(HARDWARE::LED_BLUE));
            message_count = 0;
        }
        digitalWrite(HARDWARE::LED_GREEN, LOW);
        digitalWrite(HARDWARE::LED_RED, HIGH);
    }
    else if(!communication_good){
        digitalWrite(HARDWARE::LED_BLUE, LOW);
        digitalWrite(HARDWARE::LED_GREEN, HIGH);
        digitalWrite(HARDWARE::LED_RED, LOW);
    }
    digitalWrite(HARDWARE::WORKLIGHT, modbus_light_state);
}

void set_motor_states(){
    // update positions based on modes
    if (!global_home){
        if (global_state == MODES::NORMAL){
            /* In normal operation the fingers are straight and parallel
            universal_position moves the forefinger, thumb, and middlefinger while pinch is fixed.
            */
            universal_position = map(modbus_position, 0, 65535, motor_groups[MG_INDEX::FOREFINGER].TRAVEL_MIN, motor_groups[MG_INDEX::FOREFINGER].TRAVEL_MAX);

            motor_groups[MG_INDEX::PINCH].SETPOINT_POS = normal_setpoint;
            update_motor_position(MG_INDEX::FOREFINGER);
            update_motor_position(MG_INDEX::THUMB);
            update_motor_position(MG_INDEX::MIDDLEFINGER);
        }
        else if (global_state == MODES::TWO_FINGER_PINCH){
            /* In two finger pinch the pinch moves the fore and middle together
            universal_position moves the forefinger, thumb, and middlefinger while pinch is fixed.
            */
            universal_position = map(modbus_position, 0, 65535, two_finger_pinch_travel_min, two_finger_pinch_travel_max);

            motor_groups[MG_INDEX::PINCH].SETPOINT_POS = two_finger_pinch_setpoint;
            update_motor_position(MG_INDEX::FOREFINGER);
            update_motor_position(MG_INDEX::THUMB);
            update_motor_position(MG_INDEX::MIDDLEFINGER);
        }
        else if (global_state == MODES::WIDE){
            /* In wide mode the pinch moves the fore and middle apart for a wider grip
            universal_position moves the forefinger, thumb, and middlefinger while pinch is fixed.
            */
            universal_position = map(modbus_position, 0, 65535, motor_groups[MG_INDEX::FOREFINGER].TRAVEL_MIN, motor_groups[MG_INDEX::FOREFINGER].TRAVEL_MAX);

            motor_groups[MG_INDEX::PINCH].SETPOINT_POS = wide_setpoint;
            update_motor_position(MG_INDEX::FOREFINGER);
            update_motor_position(MG_INDEX::THUMB);
            update_motor_position(MG_INDEX::MIDDLEFINGER);

        }
        else if (global_state == MODES::SCISSOR){
            /* In wide mode the pinch moves the fore and middle apart for a wider grip
            universal_position moves the forefinger, thumb, and middlefinger while pinch is fixed.
            */
            universal_position = map(modbus_position, 0, 65535, motor_groups[MG_INDEX::PINCH].TRAVEL_MIN, motor_groups[MG_INDEX::PINCH].TRAVEL_MAX);

            motor_groups[MG_INDEX::FOREFINGER].SETPOINT_POS = scissor_setpoint;
            motor_groups[MG_INDEX::THUMB].SETPOINT_POS = scissor_setpoint;
            motor_groups[MG_INDEX::MIDDLEFINGER].SETPOINT_POS = scissor_setpoint;
            update_motor_position(MG_INDEX::PINCH);
        }
    }
    // Update the position
    set_position(MG_INDEX::PINCH);
    set_position(MG_INDEX::FOREFINGER);
    set_position(MG_INDEX::THUMB);
    set_position(MG_INDEX::MIDDLEFINGER);
}

void home_routine(){
    if (global_home){
        // need to home all axis
        // home 2 / 3 / 4 first
        if (global_home_started){
            motor_groups[MG_INDEX::FOREFINGER].IS_HOMING = true;
            motor_groups[MG_INDEX::THUMB].IS_HOMING = true;
            motor_groups[MG_INDEX::MIDDLEFINGER].IS_HOMING = true;
            global_home_started = false;
            pinch_home_started = true;
        }
        // wait till done
        if (!motor_groups[MG_INDEX::FOREFINGER].IS_HOMING && !motor_groups[MG_INDEX::THUMB].IS_HOMING && !motor_groups[MG_INDEX::MIDDLEFINGER].IS_HOMING){
            // home 1
            if (pinch_home_started){
                motor_groups[MG_INDEX::PINCH].IS_HOMING = true;
                pinch_home_started = false;
            }
        }
        // reset
        if (!motor_groups[MG_INDEX::FOREFINGER].IS_HOMING && !motor_groups[MG_INDEX::THUMB].IS_HOMING && !motor_groups[MG_INDEX::MIDDLEFINGER].IS_HOMING && !motor_groups[MG_INDEX::PINCH].IS_HOMING){
            global_home = false;
        }
    }
    home(MG_INDEX::FOREFINGER);
    home(MG_INDEX::THUMB);
    home(MG_INDEX::MIDDLEFINGER);
    home(MG_INDEX::PINCH);
}

void home(int motor){
    if (motor_groups[motor].IS_HOMING){
        // if it is time to move the motor and we haven't tripped the current yet:
        unsigned long curr_millis = millis();
        if (curr_millis - prev_millis > interval){
            prev_millis = curr_millis;
            motor_groups[motor].SETPOINT_POS -= 10; // increment by counts
        }
        if (motor_groups[motor].CURRENT_READING > motor_groups[motor].TRIP_THRESHOLD){
            motor_groups[motor].ENC->write(0); //
            motor_groups[motor].SETPOINT_POS = motor_groups[motor].HOME_BACKOFF; // backoff
            motor_groups[motor].IS_HOMING = false; // don't home anymore
        }
    }
}

void poll_sensors(){

    // Update current
    get_current(MG_INDEX::PINCH);
    get_current(MG_INDEX::FOREFINGER);
    get_current(MG_INDEX::THUMB);
    get_current(MG_INDEX::MIDDLEFINGER);

    // Update motor positions
    motor_groups[MG_INDEX::PINCH].INPUT_POS = motor_groups[MG_INDEX::PINCH].ENC->read();
    motor_groups[MG_INDEX::FOREFINGER].INPUT_POS = motor_groups[MG_INDEX::FOREFINGER].ENC->read();
    motor_groups[MG_INDEX::THUMB].INPUT_POS = motor_groups[MG_INDEX::THUMB].ENC->read();
    motor_groups[MG_INDEX::MIDDLEFINGER].INPUT_POS = motor_groups[MG_INDEX::MIDDLEFINGER].ENC->read();

    // Report current
    modbus_data[MODBUS_REGISTERS::CURRENT_PINCH] = abs(motor_groups[MG_INDEX::PINCH].CURRENT_READING);
    modbus_data[MODBUS_REGISTERS::CURRENT_FOREFINGER] = abs(motor_groups[MG_INDEX::FOREFINGER].CURRENT_READING);
    modbus_data[MODBUS_REGISTERS::CURRENT_THUMB] = abs(motor_groups[MG_INDEX::THUMB].CURRENT_READING);
    modbus_data[MODBUS_REGISTERS::CURRENT_MIDDLEFINGER] = abs(motor_groups[MG_INDEX::MIDDLEFINGER].CURRENT_READING);

    // Report motor position
    modbus_data[MODBUS_REGISTERS::POSITION_PINCH] = motor_groups[MG_INDEX::PINCH].INPUT_POS;
    modbus_data[MODBUS_REGISTERS::POSITION_FOREFINGER] = motor_groups[MG_INDEX::FOREFINGER].INPUT_POS;
    modbus_data[MODBUS_REGISTERS::POSITION_THUMB] = motor_groups[MG_INDEX::THUMB].INPUT_POS;
    modbus_data[MODBUS_REGISTERS::POSITION_MIDDLEFINGER] = motor_groups[MG_INDEX::MIDDLEFINGER].INPUT_POS;
}

void update_motor_position(int motor){
    // If we overcurrent and the LGPS flag is not set, set the last good position and enable the flag
    if (motor_groups[motor].CURRENT_READING < motor_groups[motor].GRIP_THRESHOLD){
        if(!motor_groups[motor].LAST_GOOD_POSITION_SET){
            motor_groups[motor].LAST_GOOD_POSITION = universal_position - backoff;
            motor_groups[motor].LAST_GOOD_POSITION_SET = true;
        }
    }
    // If we back off reset the LGPS flag
    if (universal_position < motor_groups[motor].LAST_GOOD_POSITION){
        motor_groups[motor].LAST_GOOD_POSITION_SET = false;
    }
    // if the LGPS flag is set we don't want to move forward
    if (motor_groups[motor].LAST_GOOD_POSITION_SET){
        motor_groups[motor].SETPOINT_POS = motor_groups[motor].LAST_GOOD_POSITION;
    }
    else{
        // set to universal position
        motor_groups[motor].SETPOINT_POS = universal_position;
    }
    // Always constrain to the valid ranges
    motor_groups[motor].SETPOINT_POS = constrain(motor_groups[motor].SETPOINT_POS, motor_groups[motor].TRAVEL_MIN, motor_groups[motor].TRAVEL_MAX);
}

void get_current(int motor){
    // subtract the last reading:
    motor_groups[motor].TOTAL = motor_groups[motor].TOTAL - motor_groups[motor].READINGS[motor_groups[motor].READ_INDEX];
    // read from the sensor:
    motor_groups[motor].READINGS[motor_groups[motor].READ_INDEX] = analogRead(motor_groups[motor].CURRENT_PIN) - current_midpoint;
    // add the reading to the TOTAL:
    motor_groups[motor].TOTAL = motor_groups[motor].TOTAL + motor_groups[motor].READINGS[motor_groups[motor].READ_INDEX];
    // advance to the next position in the array:
    motor_groups[motor].READ_INDEX++;

    // if we're at the end of the array...
    if (motor_groups[motor].READ_INDEX >= motor_groups[motor].NUM_READINGS) {
    // go to begginging:
    motor_groups[motor].READ_INDEX = 0;
    }

    // calculate the average:
    motor_groups[motor].CURRENT_READING = motor_groups[motor].TOTAL / motor_groups[motor].NUM_READINGS;
}

void set_position(int motor){

    //move to new position
    if (motor_groups[motor].INPUT_POS > motor_groups[motor].SETPOINT_POS) { //need to move negative
        motor_groups[motor].PID_CONTROL->Compute();
        motor_groups[motor].PID_CONTROL->SetControllerDirection(REVERSE);
        set_motor_output(motor, REV, motor_groups[motor].OUTPUT_POS);
    }
    else{ //need to move positive
        motor_groups[motor].PID_CONTROL->SetControllerDirection(DIRECT);
        motor_groups[motor].PID_CONTROL->Compute();
        set_motor_output(motor, FWD, motor_groups[motor].OUTPUT_POS);
    }
}

void set_motor_output(int motor, int direct, int pwm){
  /*
  Control Logic
                A  |  B
  0) BRAKE:     1     1
  1) FWD:      1/0    0
  2) REV:       0    1/0
  3) COAST:     0     0
  */

  // make sure the PWM doesn't go too high
  // pwm = map(pwm, 0, 255, 0, 110);

  if (direct <= 4){
    switch (direct){
      case 0:
      analogWrite(motor_groups[motor].PWM_1_PIN, 255);
      analogWrite(motor_groups[motor].PWM_2_PIN, 255);
      break;
      case 1:
      analogWrite(motor_groups[motor].PWM_1_PIN, pwm);
      analogWrite(motor_groups[motor].PWM_2_PIN, 0);
      break;
      case 2:
      analogWrite(motor_groups[motor].PWM_1_PIN, 0);
      analogWrite(motor_groups[motor].PWM_2_PIN, pwm);
      break;
      case 3:
      analogWrite(motor_groups[motor].PWM_1_PIN, 0);
      analogWrite(motor_groups[motor].PWM_2_PIN, 0);
      break;
      default:
      analogWrite(motor_groups[motor].PWM_1_PIN, 0);
      analogWrite(motor_groups[motor].PWM_1_PIN, 0);
    }
  }
}
