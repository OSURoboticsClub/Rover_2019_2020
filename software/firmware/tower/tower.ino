////////// Includes //////////
#include <ModbusRtu.h>
#include <Adafruit_BNO055_t3.h>
#include <ArduinoJson.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

/*
   Imu/data (Imu)
   Imu/raw (Imu)
   Imu/mag (MagneticField)
   Imu/temp (Temperature)

   ---Raw Data (Imu/raw) ---
   LinearAccel (x, y, z)
   AngularVel (x, y, z);

   --- Filtered Data (Imu/data) ---
   Orientation(z, y, z, w)
   LinearAccel(x, y, z)
   AngularVel(x, y, z)

   --- Mag Data (Imu/mag) ---
   MagField(x, y, z)

   --- IMU Temp (Imu/temp)
   temp (deg c)
*/

////////// Hardware / Data Enumerations //////////
enum HARDWARE {
  GPS_IMU_RS485_EN = 3,
  GPS_IMU_RS485_RX = 9,
  GPS_IMU_RS485_TX = 10,

  COMMS_RS485_EN = 2,
  COMMS_RS485_RX = 0,
  COMMS_RS485_TX = 1,

  GPS_UART_RX = 7,
  GPS_UART_TX = 8,

  IMU_SDA = 18,
  IMU_SCL = 19,

  WHITE_LED_CONTROL = 11,
  C02_SENSOR = A7,
  MISC_PIN = A8,

  LED_BLUE_EXTRA = 13
};

enum MODBUS_REGISTERS {
  LED_CONTROL = 0,  // Input
  CO2_READING_PPM = 1
};

enum LIGHT_STATES {
  NO_CHANGE = 0,
  LIGHT_OFF = 1,
  LIGHT_FLASH = 2,
  LIGHT_MED = 3,
  LIGHT_HIGH = 4
};

#define GPS_SERIAL_PORT Serial3
#define GPS_IMU_STREAMING_PORT Serial2

////////// Global Variables //////////
///// Modbus
const uint8_t node_id = 1;
const uint8_t mobus_serial_port_number = 1;

uint16_t modbus_data[] = {0, 0};
uint8_t num_modbus_registers = 0;

int8_t poll_state = 0;
bool communication_good = false;
uint8_t message_count = 0;

///// IMU
imu::Vector<3> linear_accel;
imu::Vector<3>  angular_vel;
imu::Quaternion quat;

char float_decimal_places = 8;

///// GPS
char current_byte = '$';
String nmea_sentence = "";
char gps_buffer[255];
unsigned char buffer_count = 0;

////////// Class Instantiations //////////
Modbus slave(node_id, mobus_serial_port_number, HARDWARE::COMMS_RS485_EN);

Adafruit_BNO055 bno = Adafruit_BNO055(WIRE_BUS, -1, BNO055_ADDRESS_A, I2C_MASTER, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_100, I2C_OP_MODE_IMM);

const char baud115200[] = "PUBX,41,1,3,3,115200,0";

void setup() {
  // Debugging
  Serial.begin(9600);
 // while (!Serial);

  // Raw pin setup
  setup_hardware();

  // Setup modbus serial communication
  num_modbus_registers = sizeof(modbus_data) / sizeof(modbus_data[0]);
  slave.begin(115200); // baud-rate at 19200
  slave.setTimeOut(150);

  // GPS & IMU serial streaming setup
  GPS_IMU_STREAMING_PORT.begin(115200);
  GPS_IMU_STREAMING_PORT.transmitterEnable(HARDWARE::GPS_IMU_RS485_EN);

  setup_imu();

  // GPS Setup
  GPS_SERIAL_PORT.begin(9600);

}

void loop() {
  // Reset JSON for next loop
  StaticJsonBuffer<1000> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();

  // Do normal polling
  poll_modbus();
  set_leds();
  send_imu_stream_line(root);
  add_cal_status(root);
  process_gps_and_send_if_ready(root);
  process_white_led_command();
  get_co2_data();

  // Print JSON and newline
  root.printTo(GPS_IMU_STREAMING_PORT);
  GPS_IMU_STREAMING_PORT.println();


}

void setup_hardware() {
  // Setup pins as inputs / outputs
  pinMode(HARDWARE::WHITE_LED_CONTROL, OUTPUT);
  pinMode(HARDWARE::C02_SENSOR, INPUT);
  pinMode(HARDWARE::MISC_PIN, OUTPUT);

  pinMode(HARDWARE::LED_BLUE_EXTRA, OUTPUT);

  // Set default pin states
  digitalWrite(HARDWARE::WHITE_LED_CONTROL, LOW);
  digitalWrite(HARDWARE::LED_BLUE_EXTRA, LOW);

  // Set teensy to increased analog resolution
  analogReadResolution(13);

}

void send_imu_stream_line(JsonObject &root) {
  JsonObject& imu_object = root.createNestedObject("imu");

  quat = bno.getQuat();
  linear_accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  angular_vel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  imu_object["ox"] = quat.x();
  imu_object["oy"] = quat.y();
  imu_object["oz"] = quat.z();
  imu_object["ow"] = quat.w();

  imu_object["lax"] = linear_accel.x();
  imu_object["lay"] = linear_accel.y();
  imu_object["laz"] = linear_accel.z();

  imu_object["avx"] = angular_vel.x();
  imu_object["avy"] = angular_vel.y();
  imu_object["avz"] = angular_vel.z();

}

void setup_imu(){

    // IMU Setup
    Serial.println("Setting up IMU");
    if (!bno.begin()) {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while (1);
    }
    Serial.println("IMU Online. Setting to external crystal.");

    int eeAddress = 0;
    long bnoID;
    bool foundCalib = false;

    EEPROM.get(eeAddress, bnoID);

    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;

    /*
    *  Look for the sensor's unique ID at the beginning oF EEPROM.
    *  This isn't foolproof, but it's better than nothing.
    */
    bno.getSensor(&sensor);
    eeAddress += sizeof(long);
    EEPROM.get(eeAddress, calibrationData);
    bno.setSensorOffsets(calibrationData);
    bno.setAxisConfig(Adafruit_BNO055::REMAP_CONFIG_P6);
    bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P6);
    bno.setMode(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
    delay(300);

    bno.setExtCrystalUse(true);
    Serial.println("IMU Configured");
    // while(1){
    //     display_cal_status();
    // }
}

void add_cal_status(JsonObject &root)
{
    JsonObject& imu_cal_object = root.createNestedObject("imu_cal");
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;

    bno.getCalibration(&system, &gyro, &accel, &mag);

    imu_cal_object["gyro"] = gyro;
    imu_cal_object["accel"] = accel;
    imu_cal_object["mag"] = mag;
}

void process_gps_and_send_if_ready(JsonObject &root) {
  root["gps"] = "";

  char num_in_bytes = GPS_SERIAL_PORT.available();

  if (num_in_bytes > 0) {
    for (char i = 0 ; i < num_in_bytes ; i++) {
      char in_byte = GPS_SERIAL_PORT.read();

      if (in_byte != '\n' && in_byte != '\r') {
        gps_buffer[buffer_count] = in_byte;
        buffer_count++;
      }

      if (in_byte == '\r') {
        gps_buffer[buffer_count] = '\0';
        root["gps"] = gps_buffer;
        buffer_count = 0;
      }

    }
  }

}

void process_white_led_command() {
  uint16_t light_command = modbus_data[MODBUS_REGISTERS::LED_CONTROL];


  if (light_command == LIGHT_STATES::LIGHT_MED) {
    digitalWrite(HARDWARE::WHITE_LED_CONTROL, HIGH);
    delay(100);               // wait for a second
    digitalWrite(HARDWARE::WHITE_LED_CONTROL, LOW);
    delay(100);               // wait for a second
  } else if (light_command == LIGHT_STATES::LIGHT_FLASH) {
    digitalWrite(HARDWARE::WHITE_LED_CONTROL, HIGH);
    delay(100);               // wait for a second
    digitalWrite(HARDWARE::WHITE_LED_CONTROL, LOW);
    delay(100);               // wait for a second
    digitalWrite(HARDWARE::WHITE_LED_CONTROL, HIGH);
    delay(100);               // wait for a second
    digitalWrite(HARDWARE::WHITE_LED_CONTROL, LOW);
    delay(100);
  } else if (light_command == LIGHT_STATES::LIGHT_HIGH) {
    digitalWrite(HARDWARE::WHITE_LED_CONTROL, HIGH);
    delay(750);               // wait for a second
    digitalWrite(HARDWARE::WHITE_LED_CONTROL, LOW);
    delay(100);
  } else if (light_command == LIGHT_STATES::LIGHT_OFF) {
    digitalWrite(HARDWARE::WHITE_LED_CONTROL, HIGH);
    delay(2000);               // wait for a second
    digitalWrite(HARDWARE::WHITE_LED_CONTROL, LOW);
    delay(2000);
  }

  modbus_data[MODBUS_REGISTERS::LED_CONTROL] = LIGHT_STATES::NO_CHANGE;
}

void get_co2_data() {
  int voltage = ((analogRead(HARDWARE::C02_SENSOR) / 8192.0) * 3300);

  if (voltage < 400 || voltage > 2000) {
    modbus_data[MODBUS_REGISTERS::CO2_READING_PPM] = 9999;
  } else {
    modbus_data[MODBUS_REGISTERS::CO2_READING_PPM] = map(voltage, 400, 2000, 0, 5000);
  }

  //  Serial.println(modbus_data[MODBUS_REGISTERS::CO2_READING_PPM]);

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
