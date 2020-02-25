class data_store{
  private:
    char char_data[255];
    int int_data[255];
    long long_data[255];
    float float_data[255];
    int type[255];
  public:
    
    data_store(){
      for(int i=0;i<255;i++){
        char_data[i] = 0x00;
        int_data[i] = 0;
        long_data[i] = 0;
        float_data[i] = 0.0;
        type[i] = 0;
      }
    }

    void set_type(int reg, String dType){
      if(dType == "int")
        type[reg] = 2;
      else if(dType == "char")
        type[reg] = 1;
      else if(dType == "long")
        type[reg] = 3;
      else if(dType == "float")
        type[reg] = 4;
    }

    int get_type(int reg){
    	return type[reg];
    }
    
    void set_data(int reg, byte* data){
        switch(type[reg]){
		case(0):
			break;
		case(1):
			char_data[reg] = data[0];
			#ifdef POTHOS_DEBUG
			Serial.print("The data that was stored is: ");
			Serial.println(char_data[reg]);
			#endif
			break;
		case(2):
			int_data[reg] = 0;
			int_data[reg] = int_data[reg] | data[0];
			int_data[reg] = int_data[reg] << 8;
			int_data[reg] = int_data[reg] | data[1];
			#ifdef POTHOS_DEBUG
			Serial.print("The data that was stored is: ");
			Serial.println(int_data[reg]);
			#endif
			break;
		case(3):
			long_data[reg] = 0;
			long_data[reg] = long_data[reg] | data[0];
			for(int i=1;i<4;i++){
				long_data[reg] = long_data[reg] << 8;	
				long_data[reg] = long_data[reg] | data[i];
			}
			#ifdef POTHOS_DEBUG
			Serial.print("The data that was stored is: ");
			Serial.println(long_data[reg]);
			#endif
			break;
		case(4):
			float_data[reg] = 0;
			memcpy(&(float_data[reg]), data, sizeof(float));
			#ifdef POTHOS_DEBUG
			Serial.print("The data that was stored is: ");
			Serial.println(float_data[reg],7);
			#endif
			break;
	}
    }


    void set_data(int reg, int data){
	if(type[reg] == 2){
		int_data[reg] = data;
	}
    }

    void set_data(int reg, float data){
	if(type[reg] == 4){
	 float_data[reg] = data;
	}
    }

    void set_data(int reg, long data){
	if(type[reg] == 3){
		long_data[reg] = data;
	}
    }

    void set_data(int reg, char data){
	if(type[reg] == 1){
		char_data[reg] = data;
	}
    }

    int get_int_data(int reg){
      return int_data[reg];  
    }

    char get_char_data(int reg){
      return char_data[reg];  
    }

    long get_long_data(int reg){
      return long_data[reg];  
    }

    float get_float_data(int reg){
      return float_data[reg];  
    }
};
