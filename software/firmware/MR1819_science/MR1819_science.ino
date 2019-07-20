#include <Adafruit_VEML6070.h>
#include <Adafruit_Si7021.h>
#include <Adafruit_CCS811.h>
#include <ModbusRtu.h>
#include <Wire.h>
#include <WindTemp.h>

#define DEBUG							//uncomment this line for a serial print of sensor data

//Hardware//
enum HARDWARE{	
	RS485_RX = 7,
	RS485_TX = 8,
	RS485_EN = 5,
	
	I2C_SDA = 18,
	i2c_SCL = 19,
	
	DUST_PIN = 23,
	
	LED_BLUE_EXTRA = 13,
	LED_GREEN = 32,
	LED_RED = 1,
	LED_BLUE = 6,

  WIND_PIN = A0,
  W_TEMP_PIN = A8
};

//Modbus//
enum MODBUS_REGISTERS{
	DUST = 0,	  //Pts/L 			//unkown range
	UV = 1,       //unitless        //unknown range
	CO2 = 2,      //pts/mil         //ranges from 400 to 8192
	TVOC = 3,     //pts/bil			//ranges from 0 to 1187
	CC_TEMP = 4,  //C			    //EARLY TESTING SHOWS CC_TEMP TO BE EXTREMLY INACURATE.
	SI_TEMP = 5,  //C	+/- .4C		// SI_TEMP IS FAR MORE ACURATE.   ranges from -10C to 85C
	HUMIDITY = 6,  //RH% +/- 3%		//ranges from 0%-80%
  WIND = 7,
  W_TEMP = 8
};

const uint8_t node_id = 1;
const uint8_t modbus_serial_port_number = 3;
uint16_t modbus_data[] = {0,0,0,0,0,0,0,0,0};
uint8_t num_modbus_registers = 0;
int8_t poll_state =0;
bool communication_good = false;
uint8_t message_count = 0;

//Dust Sensor//
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 30000;//recomended sample time: 30s
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;

//sensor registors//
float dust_raw =0.0;
int UV_raw = 0;
int CO2_raw = 0;
int TVOC_raw = 0;
float CC_temp_raw = 0.0;
float si_temp_raw = 0.0;
float humidity_raw = 0.0;
float wind_raw = 0.0;
float w_temp_raw = 0.0;

//Class Instantiations//
Modbus slave(node_id, modbus_serial_port_number, HARDWARE::RS485_EN);
Adafruit_CCS811 ccs;
Adafruit_Si7021 tempAndHumidity = Adafruit_Si7021();
Adafruit_VEML6070 uv = Adafruit_VEML6070();
WindTemp MD(HARDWARE::WIND_PIN,HARDWARE::W_TEMP_PIN);


void setup() {
	#ifdef DEBUG
		Serial.begin(9600);
	#endif
	setup_hardware();
	
	//setup modbus
	num_modbus_registers = sizeof(modbus_data) / sizeof(modbus_data[0]);
	slave.begin(115200);
	slave.setTimeOut(3000);		//timeout of 3 seconds because sensors don't need to be pulled often.
}

void loop() {
  update_sensor_data();
  
  #ifdef DEBUG
  Serial.print("Dust: ");
  Serial.print(dust_raw);
  Serial.print("pcs/L    UV: ");
  Serial.print(UV_raw);
  Serial.print("    humidity: ");
  Serial.print(humidity_raw);
  Serial.print("    si_temperature: ");
  Serial.print(si_temp_raw);
  Serial.print("C    CO2: ");
  Serial.print(CO2_raw);
  Serial.print("ppm    TVOC: ");
  Serial.print(TVOC_raw);
  Serial.print("ppb    cc_temperature: ");
  Serial.print(CC_temp_raw);
  Serial.print("C     wind: ");
  Serial.print(wind_raw);
  Serial.print("        w_temperature: ");
  Serial.println(w_temp_raw);
  #endif

  // Do normal polling
  poll_modbus();
  set_leds();
}

void setup_hardware(){
  
  //sensors//
  pinMode(HARDWARE::DUST_PIN, INPUT);		//set to read pulses from dust sensor
  starttime = millis();					//sets start time
  uv.begin(VEML6070_1_T);
  #ifdef DEBUG
  if (!ccs.begin())
    Serial.println("Sensor (CCS811) failed to be found.");
  if (!tempAndHumidity.begin())
    Serial.println("Sensor (Temperature & Humidity) failed to be found.");
  #else
  ccs.begin();
  tempAndHumidity.begin();
  #endif
  while(!ccs.available());
  float temp = ccs.calculateTemperature();
  ccs.setTempOffset(temp - 25.0);			//default temperature calibration
  
  //LEDs//
  pinMode(HARDWARE::LED_RED, OUTPUT);
  pinMode(HARDWARE::LED_GREEN, OUTPUT);
  pinMode(HARDWARE::LED_BLUE, OUTPUT);
  pinMode(HARDWARE::LED_BLUE_EXTRA, OUTPUT);
  digitalWrite(HARDWARE::LED_RED, LOW);
  digitalWrite(HARDWARE::LED_GREEN, HIGH);
  digitalWrite(HARDWARE::LED_BLUE, HIGH);
  digitalWrite(HARDWARE::LED_BLUE_EXTRA, LOW);
}

void update_sensor_data(){
  if(get_dust() >1)
    dust_raw = concentration;
  get_humidity(humidity_raw);
  get_temperature(si_temp_raw);
  get_airQuality(CO2_raw,TVOC_raw,CC_temp_raw);
  get_UV(UV_raw);
  MD.update(wind_raw,w_temp_raw);
  
  modbus_data[MODBUS_REGISTERS::DUST] = uint16_t(dust_raw);
  modbus_data[MODBUS_REGISTERS::UV] = uint16_t(UV_raw);
  modbus_data[MODBUS_REGISTERS::CO2] = uint16_t(CO2_raw);
  modbus_data[MODBUS_REGISTERS::TVOC] = uint16_t(TVOC_raw);
  modbus_data[MODBUS_REGISTERS::CC_TEMP] = uint16_t(CC_temp_raw*100.0);
  modbus_data[MODBUS_REGISTERS::SI_TEMP] = uint16_t(si_temp_raw*100.0);
  modbus_data[MODBUS_REGISTERS::HUMIDITY] = uint16_t(humidity_raw*100.0);
  modbus_data[MODBUS_REGISTERS::WIND] = uint16_t(wind_raw*100.0);
  modbus_data[MODBUS_REGISTERS::W_TEMP] = uint16_t(w_temp_raw*100.0);
}

float get_dust() {
  duration = pulseIn(HARDWARE::DUST_PIN, LOW);
  lowpulseoccupancy = lowpulseoccupancy + duration;
  if ((millis() - starttime) > sampletime_ms) //if the sampel time == 30s
  {
    ratio = lowpulseoccupancy / (sampletime_ms * 10.0); // Integer percentage 0=>100
    concentration = (1000.0 / 283.0) * 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62; // using spec sheet curve
    lowpulseoccupancy = 0;
    starttime = millis();
  }
  return concentration;
}

void get_humidity(float& humidity) {
  humidity = tempAndHumidity.readHumidity();
}

void get_temperature(float& si_temp) {
  si_temp = tempAndHumidity.readTemperature();
}

void get_UV(int & UV){
  UV = uv.readUV();  
}

void get_airQuality(int& CO2, int& TVOC, float& temp) {
  if (ccs.available() && !ccs.readData()) {
    CO2 = ccs.geteCO2();
    TVOC = ccs.getTVOC();
    temp = ccs.calculateTemperature();
  }
  #ifdef DEBUG
  else
    Serial.println("CCS811 reading failed");
  #endif
}

void poll_modbus() {
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
