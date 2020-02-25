/******************************************
*Pothos V1
*Author: Anthony Grana
*1/10/2020
*OSURC Mars Rover 2019 - 2020
******************************************/


#include <data_store.h>

class pothos{
	private:
		int ID;
		HardwareSerial serial;
		int sync_counter;
		int enPin;
		void read();
		void write();
		void write_multiple();
		void sync();
	public:
		data_store data;
		pothos(int, int);
		void setPort(HardwareSerial* s){ serial = *s;}
		void update();
		bool synced;
};

pothos::pothos(int SlaveID, int en){
	ID = SlaveID;
	enPin = en;
	pinMode(en,OUTPUT);
	synced = true;
}

void pothos::update(){

	if(synced){
		if(serial.available()>4){
			if(serial.read() == 0x00){
				if(serial.read() == ID){
					#ifdef POTHOS_DEBUG
					Serial.println("slave ID Match");
					#endif
					int fc = serial.read();
					switch(fc){
						case(1):
							read();
							break;
						case(2):
							write();
							break;
						case(3):
							write_multiple();
							break;
					}
					serial.read();
					digitalWrite(enPin,HIGH);
					serial.write(0xff);
					serial.write(0xff);
					serial.write(0xff);
					serial.write(0xff);
					while(serial.availableForWrite() != 63) delayMicroseconds(1);
					delayMicroseconds(800);
					digitalWrite(enPin,LOW);
				}else{
					sync_counter = 0;
					sync();
				}
			}else {
				sync_counter = 0;
				sync();
			}
		}
	}else sync();
}

void pothos::read(){
	#ifdef POTHOS_DEBUG
	Serial.println("Reading!");
	#endif
	byte adrs[255];
	int length=0;
	adrs[length] = serial.read();
	while(serial.peek() != 0xff){
		length++;
		adrs[length] = serial.read();
	}
	serial.read();
	digitalWrite(enPin,HIGH);
	for(int i=0;i<=length;i++){
		#ifdef POTHOS_DEBUG
		Serial.print("Transmitting at adr ");
		Serial.print(adrs[i]);
		Serial.print(" : ");
		#endif
		int type = data.get_type(adrs[i]);
		
		switch(type){
			case(1):
				#ifdef POTHOS_DEBUG
				Serial.println(data.get_char_data(adrs[i]));
				#endif
				serial.write(data.get_char_data(adrs[i]));
				break;
			case(2):
				#ifdef POTHOS_DEBUG
				Serial.println(data.get_int_data(adrs[i]));
				#endif
				for(int j=1;j>=0;j = j-1)
					serial.write((byte)(0 | (data.get_int_data(adrs[i]) >> 8*(1-j))));
				break;
			case(3):
				#ifdef POTHOS_DEBUG
				Serial.println(data.get_long_data(adrs[i]));
				#endif
				for(int j=3;j>=0;j = j-1)
					serial.write((byte)(0 | ((data.get_long_data(adrs[i]) >> 8*(3-j)))));
				break;
			case(4):
				#ifdef POTHOS_DEBUG
				Serial.println(data.get_float_data(adrs[i]),9);
				#endif
				float datf = data.get_float_data(adrs[i]);
				byte* dat_raw = (byte*)(&datf);
				for(int j=0;j<4;j++)
					serial.write(dat_raw[j]);
				break;
		}
		serial.write(0x00);
	}
	serial.write(0xff);
	#ifdef POTHOS_DEBUG
	Serial.println("\n");
	#endif
}

void pothos::write(){
	int reg = serial.read();
	int len;
	int type = data.get_type(reg);
	#ifdef POTHOS_DEBUG
	Serial.println("Writing single!");
	Serial.print("data type: ");
	Serial.println(type);
	#endif
	if(type <= 2) len = type;                        //determine length of data
	else len = 4;
	#ifdef POTHOS_DEBUG
	Serial.print("register is : ");
	Serial.println(reg);
	Serial.print("length in bytes: ");
	Serial.println(len);
	if(len == 0) Serial.println("You are trying to access a register that has an unset type!!!!!!!!!!!!    ERROR!!!");
	#endif
	while(serial.available()<=len) delayMicroseconds(1);                  //get trapped in a loop until all of the data has arived 
	byte data_in[len];
	for(int i=0;i<len;i++){
		data_in[i] = serial.read();              // recieve data as a byte array
		#ifdef POTHOS_DEBUG
		Serial.print("Byte ");
		Serial.print(i);
		Serial.print(": ");
		Serial.println(data_in[i]);
		#endif
	}
	data.set_data(reg,data_in); 
	#ifdef POTHOS_DEBUG
	Serial.println("\n");
	#endif
}

void pothos::write_multiple(){
	#ifdef POTHOS_DEBUG
	Serial.println("Writing multiple!");
	#endif
	int reg = serial.read();
	int len, type;
	while(reg != 0xff){
		type = data.get_type(reg);
		#ifdef POTHOS_DEBUG
		Serial.print("Writing to reg: ");
		Serial.println(reg);
		#endif
		if(type < 3) len = type;
		else len = 4;
		#ifdef POTHOS_DEBUG
		Serial.print("length in bytes: ");
		Serial.println(len);
		if(len == 0) Serial.println("You are trying to access a register that has not been set up!!!!!!!!!!!!    ERROR!!!");
		#endif
		while(serial.available() <= len) delayMicroseconds(1);
		byte data_in[len];
		for(int i=0;i<len;i++){
			data_in[i] = serial.read();              // recieve data as a byte array
			#ifdef POTHOS_DEBUG
			Serial.print("Byte ");
			Serial.print(i);
			Serial.print(": ");
			Serial.println(data_in[i]);
			#endif
		}
		data.set_data(reg,data_in);
		reg = serial.read();
	}
	#ifdef POTHOS_DEBUG
	Serial.println("\n");
	#endif
}

void pothos::sync(){
	synced = false;
	if(sync_counter < 5){
		while(serial.available()){
			if(serial.read() == 0xff)
				sync_counter++;
			else sync_counter = 0;
		}
	}else{
		while(serial.available()){
			if(serial.peek() != 0xff){
				synced = true;
				break;
			}else serial.read();
		}
	}
}

//0xff 0xff 0xff 0xff 0xff