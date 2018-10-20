#define Rpin A5                 //Reciever pin is A1
int sensitivity = 50;           //the lower this number, the more sensitive.
int state;                      //=1 if can hear ping        =2 if cannot hear ping
float freq;
float ambientNoise;
unsigned long totalDataPoints;
int dataBuff[3];
int data[3];
unsigned long t1,t2,t3;
int dt1,dt2 =0;
float dtavg;
int tcnt =2;
bool upstate = false;
bool mode = false;                      //switches between two modes (t/f)
void setup() {
  pinMode(Rpin, INPUT);
  Serial.begin(9600);
  totalDataPoints = 0;
  state = 2;
}
void dataSet(){
  for(int i=0;i<3;i++){
    data[i]=analogRead(Rpin);
    delay(1);
  }
}
int changeCheck(){
    int newSignalState = 0;                // 0= no change      1= signal start       2= signal stop
    if((data[0])>(dataBuff[2]+sensitivity)||data[2] > data[0]+sensitivity)
      newSignalState = 1;
    if((data[2] < data[0]-sensitivity)||(data[0]<(dataBuff[2]-sensitivity)))
      newSignalState = 2;
    return newSignalState;

}
void loop() {

  if(Serial.available()){
    Serial.read();
    mode = !mode;
  }
  if(mode == true){                      //mode 1 is raw output from the antena.
    Serial.println(analogRead(Rpin));
    delay(1);
  }
  else{
    for(int i=0;i<3;i++)
      dataBuff[i] = data[i];
    dataSet();
    int change = changeCheck();
    if(change){
      state = change;
      if(change == 1){
        t1 = millis();
        dt1 = t1-t2;
      }else{
        t2 = millis();
        dt2 = t2-t1;
      }
      if(dt2>dt1-100&&dt1>dt2-100){
        dtavg = (dtavg*float(tcnt-2)/float(tcnt))+(((dt1+dt2)/2.0)*float(2.0/tcnt));
        tcnt += 2;
        freq = 500.0/dtavg;
      }
    }
    if(change ==1 || millis() > dtavg*2+t3){
      t3 = millis();
      upstate = true;
    }
    if(millis()<t3+dtavg){
      for(int i=0;i<3;i++){
        Serial.print(data[i]-ambientNoise);
        Serial.print(", ");
        Serial.println(freq);
      }
    }else{
      float avgdat,dtot =0;
      for(int i=0;i<3;i++)
        dtot+=data[i];
      avgdat = dtot/3.0;
      ambientNoise = ambientNoise*float(totalDataPoints)/float(totalDataPoints+3)+avgdat*(3.0/float(totalDataPoints+3));
    }
  }
}
