const int pwm_11 = 5;
const int pwm_12 = 21;

#define COAST   0
#define REVERSE 1
#define FORWARD 2
#define BRAKE   3

void setup() {
  pinMode(pwm_11, OUTPUT);
  pinMode(pwm_12, OUTPUT);
  Serial.begin(9600);
  digitalWrite(pwm_11, LOW);
  digitalWrite(pwm_12, LOW);
}

void loop() {
  setMotorOutput(REVERSE);
  delay(1000);
  setMotorOutput(FORWARD);
  delay(1000);
}

//---Set Motor Output---//
/*
  ----------------
  Control Logic:
  ----------------
           IN1 | IN2 | DIRECT
      Coast: 0    0      0
    Reverse: 0    1      1
    Forward: 1    0      2
      Brake: 1    1      3
  ----------------
*/

void setMotorOutput(uint8_t direct)
{
  if (direct <= 4)
  {
    if (direct <= 1){
      digitalWrite(pwm_11, LOW);
      Serial.println("1.1 LOW");
    }
    else{
      digitalWrite(pwm_11, HIGH);
      Serial.println("1.1 HIGH");
    }

    if ((direct == 0) || (direct == 2)){
      digitalWrite(pwm_12, LOW);
      Serial.println("1.2 LOW");
    }
    else{
      digitalWrite(pwm_12, HIGH);
      Serial.println("1.2 HIGH");
    }
  }
}
