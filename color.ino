#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <Servo.h>
#include <NewPing.h>

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

const int ledR = 44;
const int ledG = 45;
const int ledB = 46;

#define SPEED 100
#define TURN_SPEED 85
#define Strafe_Speed 70

#define MID_SPEED 75
#define HIGH_SPEED 100
#define LOW_SPEED 60

#define LONG_DELAY_TIME 70
#define DELAY_TIME 50
#define SHORT_DELAY_TIME 30

#define speedPinR 9
#define RightMotorDirPin1 22
#define RightMotorDirPin2 24
#define speedPinL 10
#define LeftMotorDirPin1 26
#define LeftMotorDirPin2 28

#define speedPinRB 11
#define RightMotorDirPin1B 5
#define RightMotorDirPin2B 6
#define speedPinLB 12
#define LeftMotorDirPin1B 7
#define LeftMotorDirPin2B 8

#define MAX_DISTANCE 300

NewPing sonar1(52,53, MAX_DISTANCE);
NewPing sonar2(50,51, MAX_DISTANCE);
NewPing sonar3(48,47, MAX_DISTANCE);
NewPing sonar4(42,43, MAX_DISTANCE);
NewPing sonar5(38,39, MAX_DISTANCE);

#define sensor1   A4
#define sensor2   A3
#define sensor3   A2
#define sensor4   A1
#define sensor5   A0

const int FRONT_CLEAR_TH = 12;
const int FRONT_CAUTION_TH = 7;
const int SIDE_SAFE_TH = 6;
const int BACK_SAFE_TH = 8;

const float RIGHT_BOOST = 1.20;

Servo garra;
int pinServo = 13;
bool garraAbierta = true;

void FR_fwd(int speed){
  int sp = (int)(speed * RIGHT_BOOST); if(sp>255) sp=255;
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, HIGH);
  analogWrite(speedPinR, sp);
}
void FR_bck(int speed){
  int sp = (int)(speed * RIGHT_BOOST); if(sp>255) sp=255;
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2, LOW);
  analogWrite(speedPinR, sp);
}
void FL_fwd(int speed){ digitalWrite(LeftMotorDirPin1,LOW); digitalWrite(LeftMotorDirPin2,HIGH); analogWrite(speedPinL,speed); }
void FL_bck(int speed){ digitalWrite(LeftMotorDirPin1,HIGH); digitalWrite(LeftMotorDirPin2,LOW); analogWrite(speedPinL,speed); }
void RR_fwd(int speed){
  int sp = (int)(speed * RIGHT_BOOST); if(sp>255) sp=255;
  digitalWrite(RightMotorDirPin1B, LOW);
  digitalWrite(RightMotorDirPin2B, HIGH);
  analogWrite(speedPinRB, sp);
}
void RR_bck(int speed){
  int sp = (int)(speed * RIGHT_BOOST); if(sp>255) sp=255;
  digitalWrite(RightMotorDirPin1B, HIGH);
  digitalWrite(RightMotorDirPin2B, LOW);
  analogWrite(speedPinRB, sp);
}
void RL_fwd(int speed){
  int boostedSpeed = speed * 1.1;
  if(boostedSpeed > 255) boostedSpeed = 255;
  digitalWrite(LeftMotorDirPin1B,LOW);
  digitalWrite(LeftMotorDirPin2B,HIGH);
  analogWrite(speedPinLB, boostedSpeed);
}
void RL_bck(int speed){
  int boostedSpeed = speed * 1.1;
  if(boostedSpeed > 255) boostedSpeed = 255;
  digitalWrite(LeftMotorDirPin1B,HIGH);
  digitalWrite(LeftMotorDirPin2B,LOW);
  analogWrite(speedPinLB, boostedSpeed);
}
void stop_Stop(){ analogWrite(speedPinLB,0); analogWrite(speedPinRB,0); analogWrite(speedPinL,0); analogWrite(speedPinR,0); }
void go_advance(int speed){ RL_fwd(speed); RR_fwd(speed); FR_fwd(speed); FL_fwd(speed); }
void go_back(int speed){ RL_bck(speed); RR_bck(speed); FR_bck(speed); FL_bck(speed); }
void left_turn(int speed){ RL_bck(speed); RR_fwd(speed); FR_fwd(speed); FL_bck(0); }
void right_turn(int speed){ RL_fwd(speed); RR_bck(0); FR_bck(0); FL_fwd(speed); }
void strafe_right(int s){ RL_bck(s); RR_fwd(s); FR_bck(s); FL_fwd(s); }
void strafe_left(int s){ RL_fwd(s); RR_bck(s); FR_fwd(s); FL_bck(s); }

void forward(int speed_left,int speed_right){
  RL_fwd(speed_left);
  RR_fwd(speed_right);
  FR_fwd(speed_right);
  FL_fwd(speed_left);
}
void reverse_(int speed){
  RL_bck(speed);
  RR_bck(speed);
  FR_bck(speed);
  FL_bck(speed);
}
void sharpRightTurn(int speed_left,int speed_right){
  RL_fwd(speed_left);
  RR_bck(speed_right);
  FR_bck(speed_right);
  FL_fwd(speed_left);
}
void sharpLeftTurn(int speed_left,int speed_right){
  RL_bck(speed_left);
  RR_fwd(speed_right);
  FR_fwd(speed_right);
  FL_bck(speed_left);
}
void stop_bot(){
  analogWrite(speedPinLB,0);
  analogWrite(speedPinRB,0);
  analogWrite(speedPinL,0);
  analogWrite(speedPinR,0);
  digitalWrite(RightMotorDirPin1B, LOW);
  digitalWrite(RightMotorDirPin2B,LOW);
  digitalWrite(LeftMotorDirPin1B, LOW);
  digitalWrite(LeftMotorDirPin2B,LOW);
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2,LOW);
  delay(30);
}

void init_GPIO(){
  pinMode(RightMotorDirPin1, OUTPUT);
  pinMode(RightMotorDirPin2, OUTPUT);
  pinMode(speedPinR, OUTPUT);
  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT);
  pinMode(speedPinL, OUTPUT);
  pinMode(RightMotorDirPin1B, OUTPUT);
  pinMode(RightMotorDirPin2B, OUTPUT);
  pinMode(speedPinRB, OUTPUT);
  pinMode(LeftMotorDirPin1B, OUTPUT);
  pinMode(LeftMotorDirPin2B, OUTPUT);
  pinMode(speedPinLB, OUTPUT);
  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  pinMode(ledB, OUTPUT);
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(sensor5, INPUT);
  stop_Stop();
}

unsigned int read_cm_avg(NewPing &s){
  unsigned long sum = 0;
  int n = 3;
  for(int i=0;i<n;i++){ sum += s.ping_cm(); delay(5); }
  return (unsigned int)(sum / n);
}

void setLED_blanco(){ analogWrite(ledR,255); analogWrite(ledG,255); analogWrite(ledB,255); }

String checarColor(){
  uint16_t r,g,b,c;
  float red,green,blue;
  tcs.getRawData(&r,&g,&b,&c);
  if(c==0) c=1;
  red   = (float)r / c * 256;
  green = (float)g / c * 256;
  blue  = (float)b / c * 256;
  analogWrite(ledR,0); analogWrite(ledG,0); analogWrite(ledB,0);
  if(blue > green && blue>red){ analogWrite(ledB,255); return "Azul"; }
  else if((red+green+blue) < 320){ return "Negro"; }
  else if((red > 155 && red <180) && (green > 105 && green < 135 )&& (blue > 65 && blue < 95)){ analogWrite(ledR,255); analogWrite(ledG,255); return "Amarillo"; }
  else if((red > 175 && red <200) && (green > 100 && green<120)&& (blue>100 && blue<125)){ analogWrite(ledR,255); analogWrite(ledB,255); return "Rosa"; }
  else if (green>red && green>blue ){ return "Verde"; }
  else if(red>green && red>blue){ return "Rojo"; }
  else { return "Null"; }
}

void mostrarColorRGB(String color){
  if(color == "Rojo"){ analogWrite(ledR,0); analogWrite(ledG,255); analogWrite(ledB,255); }
  else if(color == "Azul"){ analogWrite(ledR,255); analogWrite(ledG,0); analogWrite(ledB,255); }
  else if(color == "Verde"){ analogWrite(ledR,255); analogWrite(ledG,255); analogWrite(ledB,0); }
  else if(color == "Rosa"){ analogWrite(ledR,80); analogWrite(ledG,60); analogWrite(ledB,255); }
  else if(color == "Amarillo"){ analogWrite(ledR,0); analogWrite(ledG,255); analogWrite(ledB,80); }
  else if(color == "Negro"){
    for(int i=0;i<3;i++){
      analogWrite(ledR,0); analogWrite(ledG,255); analogWrite(ledB,255); delay(150);
      analogWrite(ledR,255); analogWrite(ledG,255); analogWrite(ledB,0); delay(150);
    }
    analogWrite(ledR,255); analogWrite(ledG,255); analogWrite(ledB,255);
  } else { setLED_blanco(); }
}

bool isBlackIR(){
  int s0 = digitalRead(sensor1);
  int s1 = digitalRead(sensor2);
  int s2 = digitalRead(sensor3);
  int s3 = digitalRead(sensor4);
  int s4 = digitalRead(sensor5);
  int zeros = (s0==0) + (s1==0) + (s2==0) + (s3==0) + (s4==0);
  return zeros >= 3;
}

bool mustAvoidBlacks(){
  String c = checarColor();
  if (c == "Negro") return true;
  if (isBlackIR()) return true;
  return false;
}

const int TURN_90_MS = 450;
const int TILE_MOVE_MS = 400;

void turnRight90(){
  stop_bot();
  RL_fwd(LOW_SPEED);
  RR_bck(LOW_SPEED);
  FR_bck(LOW_SPEED);
  FL_fwd(LOW_SPEED);
  delay(TURN_90_MS);
  stop_bot();
  delay(60);
}
void turnLeft90(){
  stop_bot();
  RL_bck(LOW_SPEED);
  RR_fwd(LOW_SPEED);
  FR_fwd(LOW_SPEED);
  FL_bck(LOW_SPEED);
  delay(TURN_90_MS);
  stop_bot();
  delay(60);
}
void moveOneTileForward(){
  go_advance(LOW_SPEED);
  delay(TILE_MOVE_MS);
  stop_bot();
  delay(500);
}

void afterRed(){
   turnLeft90();
   String curColor = checarColor();
   while(curColor =! "Verde"){
   go_advance(SPEED);
   delay(1300);
   strafe_left(SPEED);
   delay(1500);
   strafe_right(SPEED);
   delay(1200);
   }
   if (curColor=="Verde"){
    go_back(SPEED);
    delay(350);
    stop_bot();
    delay(9000);

   }
}

void rightHandStep(){
  unsigned int dR  = read_cm_avg(sonar1);
  unsigned int dFR = read_cm_avg(sonar2);
  unsigned int dFL = read_cm_avg(sonar3);
  unsigned int dL  = read_cm_avg(sonar4);
  unsigned int dB  = read_cm_avg(sonar5);
  String curColor = checarColor();
  if(curColor == "Rojo"){
    stop_bot();
    mostrarColorRGB(curColor);
    afterRed();
    return;
  }
  if(mustAvoidBlacks()){
    reverse_(LOW_SPEED);
    delay(250);
    stop_bot();
    if(dR > dL){ turnRight90(); moveOneTileForward(); }
    else { turnLeft90(); moveOneTileForward(); }
    return;
  }
  if(dR > SIDE_SAFE_TH){
    turnRight90();
    moveOneTileForward();
    return;
  }
  if(dFR > FRONT_CLEAR_TH && dFL > FRONT_CLEAR_TH){
    moveOneTileForward();
    return;
  }
  if(dFR > FRONT_CLEAR_TH && dFL <= FRONT_CLEAR_TH){
    moveOneTileForward();
    return;
  }
  if(dFL > FRONT_CLEAR_TH && dFR <= FRONT_CLEAR_TH){
    moveOneTileForward();
    return;
  }
  if(dL > SIDE_SAFE_TH){
    turnLeft90();
    moveOneTileForward();
    return;
  }
  if(dB > BACK_SAFE_TH){
    go_back(LOW_SPEED);
    delay(TILE_MOVE_MS/2);
    stop_bot();
    delay(60);
    turnRight90();
    moveOneTileForward();
    return;
  }
  turnRight90();
  turnRight90();
  moveOneTileForward();
}

void explore(){
  rightHandStep();
}

void abrirGarra(){ garra.write(55); garraAbierta = true; }
void cerrarGarra(){ garra.write(145); garraAbierta = false; }

void setup(){
  Serial.begin(9600);
  init_GPIO();
  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  pinMode(ledB, OUTPUT);
  garra.attach(pinServo);
  if(!tcs.begin()){
    while(1);
  }
}

void loop(){
  explore();
  delay(80);
}
