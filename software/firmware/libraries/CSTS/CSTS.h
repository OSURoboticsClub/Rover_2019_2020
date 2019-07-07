#include <average.h>

const int size = 50;
int datac[size];					//for not using averages, comment out lines 3,4,17,18 and uncomment 31,32
int datat[size];

average average_current(datac,size);
average average_temp(datat,size);


/***********************************************
normal cals are voltage cal = 3.3/1024
cs_scale = 50
cs_offset = 2.2
temp_scale = 158
temp_offset = 81
*************************************************/ 


class CSTS{
	public:
		CSTS(int,float,int,int,float);
		void update(float&,float&);
	private:
		int cs_scale;
		float cs_offset;
		int temp_scale;
		int temp_offset;
		float voltage_cal;
};



void CSTS::update(float& CS,float& Temp){
	float avgc = average_current.update(analogRead(A4));
	float avgt = average_temp.update(analogRead(A9));
	float cs_voltage = avgc*(voltage_cal) + 0.002;
	float temp_voltage = avgt*(voltage_cal)+0.002;
	CS = cs_voltage*cs_scale-cs_offset;
	Temp = ((temp_voltage-.75)*100)+25;
}

CSTS::CSTS(int csScale,float csOffset,int tempScale,int tempOffset,float vcal){
	cs_scale = csScale;
	cs_offset = csOffset;
	temp_scale = tempScale;
	temp_offset = tempOffset;
	voltage_cal = vcal;
} 
