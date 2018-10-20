float freq = 1.5;
int pin = 13;
// convert freq to mills
float mills = (1/freq)*1000;

void setup(){
    pinMode(pin, OUTPUT);
}

void loop(){
    digitalWrite(pin,LOW);
    delay(mills / 2.0);
    digitalWrite(pin, HIGH);
    delay(mills / 2.0);
}
