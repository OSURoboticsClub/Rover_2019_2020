////////// Includes //////////
#include <pothos.h>
//#include <Adafruit_BNO055_t3.h>
#include <ArduinoJson.h>
//#include <utility/imumaths.h>
#include <EEPROM.h>
//#include <Adafruit_NeoPixel.h>
#include <FastLED.h>
FASTLED_USING_NAMESPACE

//#define debug

////////// Hardware / Data Enumerations //////////
enum HARDWARE {
  GPS_IMU_RS485_EN = 15,
  GPS_IMU_RS485_RX = 0,
  GPS_IMU_RS485_TX = 1,

  COMMS_RS485_EN = 2,
  COMMS_RS485_RX = 7,
  COMMS_RS485_TX = 8,

  GPS_UART_RX = 9,
  GPS_UART_TX = 10,

  IMU_SDA = 18,
  IMU_SCL = 19,

  NEO_PIXEL = 23,

  LED_BLUE_EXTRA = 13,

  LASER_SIGNAL = 14
};

enum DATA {
  LED_LIGHTS_STATE = 0,
  LED_DRIVE_STATE = 1,
  LED_WAYPOINT_STATE = 2,
  LED_COMMS_STATE = 3,
  LASERS = 4
};

enum MODBUS_REGISTERS {
  LED_CONTROL = 0,  // Input
};

#define GPS_SERIAL_PORT Serial2
#define GPS_IMU_STREAMING_PORT Serial1
#define Num_Pixels 60
#define Num_rainbow_pixels 57

////////// Global Variables //////////
///// Modbus
const uint8_t node_id = 1;
const uint8_t mobus_serial_port_number = 3;

uint16_t modbus_data[] = {0};
uint8_t num_modbus_registers = 0;

int8_t poll_state = 0;
bool communication_good = false;
uint8_t message_count = 0;

///// GPS
char current_byte = '$';
String nmea_sentence = "";
char gps_buffer[255];
unsigned char buffer_count = 0;

///// LEDS
CRGB red = CRGB(255,0,0);
CRGB green = CRGB(0,255,0);
CRGB blue = CRGB(0,0,255);
CRGB orange = CRGB(255,20,0);
CRGB yellow = CRGB(255,130,0);
CRGB pink = CRGB(255,20,20);
CRGB purple = CRGB(60,0,50);
CRGB white = CRGB(255,255,255);
CRGB turquoise = CRGB(0,130,60);
CRGB leds[Num_Pixels];
uint8_t gHue = 0;
uint8_t laser_last_state = 0;

unsigned long pixel_clock = 0 ;
int pixel_timer = 15;


////////// Class Instantiations //////////
pothos comms(node_id, HARDWARE::COMMS_RS485_EN);

const char baud115200[] = "PUBX,41,1,3,3,115200,0";

void setup() {
  
  comms.setPort(&serial1);

  setup_hardware();

  Serial.begin(9600);

  set_data_types();

}

void loop() {
  // Reset JSON for next loop
  StaticJsonBuffer<1000> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();

  // Do normal polling
  comms.update();
  
  set_leds();
  process_gps_and_send_if_ready(root);

  // Print JSON and newline
  root.printTo(GPS_IMU_STREAMING_PORT);
  GPS_IMU_STREAMING_PORT.println();
}

void setup_hardware() {
  // Setup pins as inputs / outputs
  pinMode(HARDWARE::LED_BLUE_EXTRA, OUTPUT);

  // Set default pin states
  digitalWrite(HARDWARE::LED_BLUE_EXTRA, LOW);
  
  // Initialize Fast LED Object
  FastLED.addLeds<WS2811,HARDWARE::NEO_PIXEL,GRB>(leds, Num_Pixels).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(120);
}

/**
 * @description: Sets the Datatypes for Pothos registers
 **/
void set_data_types(){
  gps_comms.data.set_type(DATA::LED_WAYPOINT_STATE, "int"); 
  gps_comms.data.set_type(DATA::LED_DRIVE_STATE, "int"); 
  gps_comms.data.set_type(DATA::LED_COMMS_STATE, "int"); 
  gps_comms.data.set_type(DATA::LED_LIGHTS_STATE, "int"); 
  gps_comms.data.set_type(DATA::LASERS, "char");
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

/**
 * @description: Sets the LASERS based on pothos register DATA
 * 
 **/
void laser_zone() {

  laser_state = DATA::LASERS;

  if(laser_last_state != laser_state) {

    laser_last_state = laser_state;

    if(laser_state) {
      digitalWrite(HARDWARE::LASER_SIGNAL, HIGH);
    } else {
      digitalWrite(HARDWARE::LASER_SIGNAL, LOW);
    }
  }
}

void rainbow() {
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, Num_rainbow_pixels , gHue, 7);
}

void rainbowWithGlitter() {
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
}

void addGlitter( fract8 chanceOfGlitter) {
  if( random8() < chanceOfGlitter) {
    leds[ random16(Num_rainbow_pixels) ] += CRGB::White;
  }
}

void confetti() {
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds,Num_rainbow_pixels, 10);
  int pos = random16(Num_rainbow_pixels);
  leds[pos] += CHSV( gHue + random8(Num_rainbow_pixels), 200, 255);
}

void sinelon() {
  // a colored dot sweeping back and forth, with fadingtrails
  fadeToBlackBy( leds, Num_rainbow_pixels, 20);
  int pos = beatsin16( 13, 0, Num_rainbow_pixels-1 );
  leds[pos] += CHSV( gHue, 255, 192);
}

void bpm() {
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 120;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for( int i = 0; i < Num_rainbow_pixels; i++) { //9948
    leds[i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
  }
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, Num_rainbow_pixels, 20);
  byte dothue = 0;
  for( int i = 0; i < 8; i++) {
    leds[beatsin16( i+7, 0, Num_rainbow_pixels-1 )] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}

void autonomous() {
  for(int i = 0; i < Num_rainbow_pixels; i++) {
    leds[i] = red;
  }
}

void teleoperation() {
  for(int i = 0; i < Num_rainbow_pixels; i++) {
    leds[i] = blue;
  }
}

void success() {
  uint8_t flash_duration = 1;
  
  if(millis() - pixel_clock > flash_duration*1000) {
    for(int i = 0; i < Num_rainbow_pixels; i++) {
      if(leds[i] != green) {
        leds[i] = green;
      } else {
        leds[i] = CHSV(0, 0, 0);
      }
    }
    
    pixel_clock = millis();
  }
}

void run_leds() {
  if (millis() - pixel_clock > pixel_timer) {
   
    static uint8_t led = 0;
    static uint8_t dir = 1;
    leds[led] = CHSV(gHue, 240, 170);
    for (int i = 0; i < Num_rainbow_pixels; i++) {
      leds[i].nscale8(237 );
    }
    if (led == Num_rainbow_pixels - 1) {
      dir = -1;
    } else if (led == 0) {
      dir = 1;
    }
    led += dir;
    gHue++;
    pixel_clock = millis();
  }
}

CRGB status_scale(int scale, int val) {
  if(val < scale/2)
    return CRGB(254,70*val*2/scale,0);
  else if(val > scale/2)
    return CRGB(255 - 255*(2*val-scale)/scale, 70+165*(2*val-scale)/scale, 0);
  else
    return yellow;  
}


/**
 * @Description: Updates the LEDs for NEOPIXEL Display via Register Data from Pothos
 **/
void set_leds() {

  EVERY_N_MILLISECONDS( 20 ) { gHue++; }

  
  bool comms_status = communication_good;
  uint8_t drive_state = 0;
  uint8_t waypoint_state = 0;
  uint8_t comms = 0;
  uint8_t lights = 0;
  
  //int * color = blue;
  
  //collect data from pothos registers
  drive_state = DATA::LED_DRIVE_STATE;
  waypoint_state = DATA::LED_WAYPOINT_STATE;
  comms = DATA::LED_COMMS_STATE;
  lights = DATA::LED_LIGHTS_STATE;


  //Calling LED Control functions
  switch(lights){
    case 1:
      rainbow();
      break;
    case 2:
      rainbowWithGlitter();
      break;
    case 3:
      confetti();
      break;
    case 4:
      sinelon();
      break;
    case 5:
      bpm();
      break;
    case 6:
      juggle();
      break;
    case 0:
      run_leds();
      break;
    case 7:
      autonomous();
    case 8:
      teleoperation();
    case 9:
      success();
  }

  if(!comms_status)
    for(int i=57;i<60;i++)
      leds[i] = purple;
  else{
    switch(waypoint_state){
      case 0:
        leds[57] = red;
        break;
      case 1:
        leds[57] = yellow;
        break;
      case 2:
        leds[57] = green;
        break;
      case 3:
        leds[57] = blue;
        break;
    }
    leds[58] = status_scale(32,comms);
    switch(drive_state){
      case 0:
        leds[59] = blue;
        break;
      case 1:
        leds[59] = pink;
        break;
      case 2:
        leds[59] = green;
        break;  
    }
  }
  
  FastLED.show();
}
