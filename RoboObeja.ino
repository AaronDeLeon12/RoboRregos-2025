#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <Servo.h> 
#include <NewPing.h>
#include <MPU6050.h>

// ----- SENSOR RGB -----
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// ----- LEDS -----
const int ledR = 44;
const int ledG = 45;
const int ledB = 46;

// ----- VELOCIDAD Y MOTORES -----
#define SPEED 100
#define TURN_SPEED 100
#define Strafe_Speed 80

#define MID_SPEED 100    
#define HIGH_SPEED 120    
#define LOW_SPEED 80    
#define LONG_DELAY_TIME 70 
#define DELAY_TIME 40 
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

NewPing sonar1(52,53, MAX_DISTANCE);//der
NewPing sonar2(50, 51, MAX_DISTANCE);//frontal der
NewPing sonar3(48, 47, MAX_DISTANCE);//frontal izq 
NewPing sonar4(42, 43, MAX_DISTANCE);//izq
NewPing sonar5(38,39,MAX_DISTANCE);

//Sensores IR
#define sensor1   A4 // Left most sensor
#define sensor2   A3 // 2nd Left   sensor
#define sensor3   A2 // center sensor
#define sensor4   A1 // 2nd right sensor// Right most sensor
#define sensor5   A0 // Right most sensor

const int FRONT_THRESHOLD = 15;    
const int SIDE_CLEAR = 30;         
const int VERY_CLOSE = 2;         
const int MOVE_SPEED = LOW_SPEED; 
const int TURN_SPEED_SLOW = LOW_SPEED; 
const unsigned long TURN_TIMEOUT = 1600; 
const int READ_REPEATS = 3; 

const uint8_t MPU = 0x68;     // dirección I2C correcta
float gyroZ_offset = 0.0;     // offset de giro Z calculado en calibración
float yaw_deg = 0.0;          // ángulo integrado (grados)
unsigned long t_prev = 0;     // timestamp para integrar (micros)

Servo garra;
int pinServo = 13;
bool garraAbierta = true;

bool rampa=false;
bool rampaTope=false;
// -------- FUNCIONES DE MOTORES --------
void FR_fwd(int speed){digitalWrite(RightMotorDirPin1, LOW);digitalWrite(RightMotorDirPin2,HIGH); analogWrite(speedPinR,speed);}

void FR_bck(int speed){
int boosted_Speed = speed*1.3; // limitar máximo PWM
digitalWrite(RightMotorDirPin1,HIGH);
digitalWrite(RightMotorDirPin2,LOW); 
analogWrite(speedPinR,boosted_Speed);}

void FL_fwd(int speed){digitalWrite(LeftMotorDirPin1,LOW); digitalWrite(LeftMotorDirPin2,HIGH); analogWrite(speedPinL,speed);}
void FL_bck(int speed){digitalWrite(LeftMotorDirPin1,HIGH); digitalWrite(LeftMotorDirPin2,LOW); analogWrite(speedPinL,speed);}
void RR_fwd(int speed){digitalWrite(RightMotorDirPin1B, LOW); digitalWrite(RightMotorDirPin2B,HIGH); analogWrite(speedPinRB,speed*1.3);}
void RR_bck(int speed){digitalWrite(RightMotorDirPin1B, HIGH); digitalWrite(RightMotorDirPin2B,LOW); analogWrite(speedPinRB,speed);}


//ajuste de potencias
void RL_fwd(int speed){
  int boostedSpeed = speed * 1.1;       // +10% de potencia
  if(boostedSpeed > 255) boostedSpeed = 255;  // limitar máximo PWM
  digitalWrite(LeftMotorDirPin1B,LOW);
  digitalWrite(LeftMotorDirPin2B,HIGH);
  analogWrite(speedPinLB, boostedSpeed);
}

void RL_bck(int speed){
  int boostedSpeed = speed * 1.1;       // +10% de potencia
  if(boostedSpeed > 255) boostedSpeed = 255;  // limitar máximo PWM
  digitalWrite(LeftMotorDirPin1B,HIGH);
  digitalWrite(LeftMotorDirPin2B,LOW);
  analogWrite(speedPinLB, boostedSpeed);
}
void stop_Stop(){analogWrite(speedPinLB,0); analogWrite(speedPinRB,0); analogWrite(speedPinL,0); analogWrite(speedPinR,0);}

void go_advance(int speed){RL_fwd(speed); RR_fwd(speed); FR_fwd(speed); FL_fwd(speed);}
void go_back(int speed){RL_bck(speed); RR_bck(speed); FR_bck(speed); FL_bck(speed);}
void left_turn(int speed){RL_bck(speed); RR_fwd(speed); FR_fwd(speed); FL_bck(0);}
void right_turn(int speed){RL_fwd(speed); RR_bck(0); FR_bck(0); FL_fwd(speed);}

void strafe_right(int strafe_Speed){RL_bck(strafe_Speed); RR_fwd(strafe_Speed); FR_bck(strafe_Speed); FL_fwd(strafe_Speed);}
void strafe_left(int strafe_Speed){RL_fwd(strafe_Speed); RR_bck(strafe_Speed); FR_fwd(strafe_Speed); FL_bck(strafe_Speed);}

//sigue lineas funciones de movimiento----------
void forward(int speed_left,int speed_right)
{
   RL_fwd(speed_left);
   RR_fwd(speed_right);
   FR_fwd(speed_right);
   FL_fwd(speed_left); 
}
void reverse(int speed)
{
   RL_bck(speed);
   RR_bck(speed);
   FR_bck(speed);
   FL_bck(speed); 
}
void right_shift(int speed_fl_fwd,int speed_rl_bck ,int speed_rr_fwd,int speed_fr_bck) 
{
  FL_fwd(speed_fl_fwd); 
  RL_bck(speed_rl_bck); 
  RR_fwd(speed_rr_fwd);
  FR_bck(speed_fr_bck);
}
void left_shift(int speed_fl_bck,int speed_rl_fwd ,int speed_rr_bck,int speed_fr_fwd)
{
   FL_bck(speed_fl_bck);
   RL_fwd(speed_rl_fwd);
   RR_bck(speed_rr_bck);
   FR_fwd(speed_fr_fwd);
}

void right(int speed)
{
   RL_fwd(speed);
   RR_bck(0);
   FR_bck(0);
   FL_fwd(speed); 
}
void left(int speed)
{
   RL_fwd(0);
   RR_bck(speed);
   FR_bck(speed);
   FL_fwd(0); 
}
void right_back(int speed)
{
   RL_bck(speed);
   RR_fwd(0);
   FR_fwd(0);
   FL_bck(speed); 
}
void sharpRightTurn(int speed_left,int speed_right)
{
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
 
void stop_bot()    //Stop
{
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
  delay(40);
}

// -------- INICIALIZACIÓN DE PINES --------
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
  stop_Stop();
}

//debugger de ultras
void checkUltraSounds(){
  unsigned int distRight = sonar1.ping_cm();
  Serial.print("Derecha (cm): "); 
  Serial.println(distRight);
  unsigned int distFL = sonar2.ping_cm();
  Serial.print("Frontal izquierda (cm): "); 
  Serial.println(distFL);
  unsigned int distFR = sonar3.ping_cm();
  Serial.print("Frontal derecha (cm) : "); 
  Serial.println(distFR);
  unsigned int distLeft = sonar4.ping_cm();
  Serial.print("Izquierda (cm): "); 
  Serial.println(distLeft);
  unsigned int distBack = sonar5.ping_cm();
  Serial.print("Atras (cm): ");
  Serial.println(distBack);
  delay(1000);
}

// -------- FUNCIÓN CHECAR COLOR --------
String checarColor(){
  uint16_t r,g,b,c;
  float red,green,blue;

  tcs.getRawData(&r,&g,&b,&c);
  if(c==0) c=1; // evitar división entre 0

  red   = (float)r / c * 256;
  green = (float)g / c * 256;
  blue  = (float)b / c * 256;

  Serial.print("R: "); Serial.print(red);
  Serial.print(" G: "); Serial.print(green);
  Serial.print(" B: "); Serial.println(blue);

  // Apagar LEDs antes de encender el correspondiente
  analogWrite(ledR,0);
  analogWrite(ledG,0);
  analogWrite(ledB,0);

  // Condiciones de color
  if(blue > green && blue>red){
    Serial.println("Color Azul");
    return "Azul";
  }
    else if((red+green+blue) <320){
    Serial.println("Color Negro");
    return "Negro";
  }
  else if((red > 155 && red <180) && (green > 105 && green < 135 )&& (blue > 65 && blue < 95)){
    Serial.println("Color Amarillo");
    return "Amarillo";
  } 
  else if((red > 175 && red <200) && (green > 100 && green<120)&& (blue>100 && blue<125)){
    Serial.println("Color Rosa");
    return "Rosa";
  } 
  else if (green>red && green>blue ){
    Serial.println("Color Verde");
    return "Verde";
  }
  else if(red>green && red>blue){
    Serial.println("Color Rojo");
    return "Rojo";
  }
  else{ 
    Serial.println("No identifica color");
    return "Null"; //  quieto
  }
}
//--------Lo que sea que haremos para mostrar el color--------------------------------------------------------------------------------
void mostrarColorRGB(String color) {
  if(color == "Rojo") {
    analogWrite(ledR, 0);
    analogWrite(ledG, 255);
    analogWrite(ledB, 255);
  }
  else if(color == "Azul") {
    analogWrite(ledR, 255);
    analogWrite(ledG, 0);
    analogWrite(ledB, 255);
  }
  else if(color == "Verde") {
    analogWrite(ledR, 255);
    analogWrite(ledG, 255);
    analogWrite(ledB, 0);
  }
  else if(color == "Rosa") { // Rojo + Verde
    analogWrite(ledR, 80);   // un poco de rojo
    analogWrite(ledG, 60);   // un poco de verde
    analogWrite(ledB, 255);  // azul apagado
  }
  else if(color == "Amarillo") { // Rojo + Azul = Magenta real
    analogWrite(ledR, 0);    // rojo encendido
    analogWrite(ledG, 255);  // verde completamente apagado
    analogWrite(ledB, 80);   // azul encendido, pero no al máximo (da tono fucsia)
  }
  else if(color == "Negro") { 
    while(color == "Negro") {
      // Rojo
      analogWrite(ledR, 0);
      analogWrite(ledG, 255);
      analogWrite(ledB, 255);
      delay(200);
      // Azul
      analogWrite(ledR, 255);
      analogWrite(ledG, 255);
      analogWrite(ledB, 0);
      delay(200);
      // Revisa el color de nuevo
      color = checarColor();
    }
    // Apagar al salir
    analogWrite(ledR, 255);
    analogWrite(ledG, 255);
    analogWrite(ledB, 255);
  }
  else { // Apagado
    analogWrite(ledR, 255);
    analogWrite(ledG, 255);
    analogWrite(ledB, 255);
  }
}



//------S I G U E   L I N E A S -------
void sigueLineas()
{
  String colorB = checarColor();
  String senstr="";
  int s0 = digitalRead(sensor1);
  int s1 = digitalRead(sensor2);
  int s2 = digitalRead(sensor3);
  int s3 = digitalRead(sensor4);
  int s4 = digitalRead(sensor5);
  int sensorvalue=32;
  sensorvalue +=s0*16+s1*8+s2*4+s3*2+s4;
  senstr= String(sensorvalue,BIN);
  senstr=senstr.substring(1,6);
  
  Serial.print(senstr);
  
  if (senstr=="10000" || senstr=="01000" || senstr=="11000")
   {
     Serial.println(" Shift Left");
      sharpLeftTurn(LOW_SPEED,MID_SPEED);
    //  left_shift(HIGH_SPEED,HIGH_SPEED,HIGH_SPEED,HIGH_SPEED);
      delay(DELAY_TIME);
      stop_bot();     
   }
   
  if ( senstr=="11100" || senstr=="10100" )
  {
     Serial.println("Slight Shift Left");
      forward(0,HIGH_SPEED);
      delay(DELAY_TIME);
      stop_bot(); 
  }
  if ( senstr=="01100" ||  senstr=="11110"  || senstr=="10010"  || senstr=="10110"  || senstr=="11010")
  {
     Serial.println("Slight Left");
      forward(LOW_SPEED,MID_SPEED);
      delay(DELAY_TIME);
  }
 if (senstr=="01110" || senstr=="01010" || senstr=="00100"  || senstr=="10001"  || senstr=="10101"  || senstr=="10011" || senstr=="11101" || senstr=="10111" || senstr=="11011"  || senstr=="11001")
  {
     Serial.println("Forward");
      forward(MID_SPEED,MID_SPEED);
      delay(DELAY_TIME);
       stop_bot(); 
  }
 if ( senstr=="00110" || senstr=="01111" || senstr=="01001" || senstr=="01011" || senstr=="01101")
  {
        Serial.println("Slit Right");
      forward(MID_SPEED,LOW_SPEED);
      delay(DELAY_TIME);
       stop_bot(); 
  }
 if (senstr=="00111" || senstr=="00101" )
  {    Serial.println("Slight Shift to Right ");
       forward(HIGH_SPEED,0);
      delay(DELAY_TIME);
      stop_bot(); 
  }
 if (senstr=="00001" || senstr=="00010" || senstr=="00011")
 {
   Serial.println("Shift to Right");
   sharpRightTurn(MID_SPEED,LOW_SPEED);
    //  right_shift(HIGH_SPEED,HIGH_SPEED,HIGH_SPEED,HIGH_SPEED);
      delay(DELAY_TIME);
      stop_bot();   
        
 }
}


// -------- SETUP --------
void setup(){
  Serial.begin(9600);
  init_GPIO();
  
  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  pinMode(ledB, OUTPUT);

  garra.attach(pinServo);
 // (ver versión nueva abajo)

}

//-----G A R R A-----
void abrirGarra(){
  garra.write(55);
  Serial.println("Garra Abierta");
  garraAbierta = true; 
}

void cerrarGarra(){
  garra.write(145);
  Serial.println("Garra Cerrada");
  garraAbierta = false; 
}
//-------------------------------------------------------------------------------------------------------E X P L O R E (GOATEST)------------------------------------------
void explorePistaC0() {
  unsigned int distRight = sonar1.ping_cm();
  unsigned int distFL = sonar2.ping_cm();
  unsigned int distFR = sonar3.ping_cm();
  unsigned int distLeft = sonar4.ping_cm();
  unsigned int distBack = sonar5.ping_cm();

  String colorActual = checarColor();
  mostrarColorRGB(colorActual);
  delay(500);

  static bool rampa = false;
  static bool rampaTope = false;
  static bool reversa = false;

  // --- NAVEGACIÓN HASTA LA RAMPA (ROJO) ---
  if (!rampa) {
    if (colorActual == "Rojo") {
      stop_bot();
      delay(300);
      rampa = true;
    } else {
      // Navegación tipo laberinto con regla de mano derecha
      if (distFR < 10 && distFL < 10) {
        go_advance(70);
        delay(400);
      } else if (distRight < 3) {
        strafe_left(60);
        delay(150);
      } else if (distLeft < 3) {
        strafe_right(60);
        delay(150);
      } else if (distFR < 5 && distFL > 6) {
        strafe_left(60);
        delay(200);
      } else if (distFL < 5 && distFR > 6) {
        strafe_right(60);
        delay(200);
      } else if (distFR < 8 && distFL < 8) {
        sharpRightTurn(70, 70);
        delay(500);
      } else {
        go_advance(80);
        delay(400);
      }

      if (colorActual == "Negro") {
        reverse(70);
        delay(400);
        strafe_left(60);
        delay(250);
      }
    } // ← ESTA LLAVE FALTABA (cierra el else del bloque !rampa)
  }

  // --- SUBIDA DE LA RAMPA ---
  if (rampa && !rampaTope && !reversa) {
    go_advance(90);
    delay(800);
    strafe_left(75);
    delay(250);
    stop_bot();
    delay(100);

    colorActual = checarColor();
    mostrarColorRGB(colorActual);

    if (colorActual == "Verde") {
      stop_bot();
      delay(800);
      rampaTope = true;
      reversa = true;
    }
  }

  // --- REVERSA DE LA RAMPA ---
  if (reversa) {
    reverse(90);
    delay(800);
    strafe_right(75);
    delay(250);
    stop_bot();
    delay(100);

    colorActual = checarColor();
    mostrarColorRGB(colorActual);

    if (colorActual == "Rojo") {
      stop_bot();
      delay(1000);
      reversa = false;
      rampa = false;
      rampaTope = false;
    }
  }

  stop_bot();
  delay(200);
}


void explorePistaC1(){
  unsigned int distRight = sonar1.ping_cm();
  unsigned int distFL = sonar2.ping_cm();
  unsigned int distFR = sonar3.ping_cm();
  unsigned int distLeft = sonar4.ping_cm();
  unsigned int distBack = sonar5.ping_cm();

  String colorActual = checarColor();
  mostrarColorRGB(colorActual);
  delay(500);

  static bool rampa = false;
  static bool rampaTope = false;
  static bool reversa = false;

  // --- NAVEGACIÓN HASTA LA RAMPA (ROJO) ---
  if (!rampa) {
    if (colorActual == "Rojo") {
      stop_bot();
      delay(300);
      rampa = true;
    } else {
      if (distFR < 10 && distFL < 10) {
        go_advance(70);
        delay(300);
      } else if (distRight < 3) {
        strafe_left(60);
        delay(150);
      } else if (distLeft < 3) {
        strafe_right(60);
        delay(150);
      } else if (distFR < 5 && distFL > 6) {
        strafe_left(60);
        delay(200);
      } else if (distFL < 5 && distFR > 6) {
        strafe_right(60);
        delay(200);
      } else if (distFR < 8 && distFL < 8) {
        sharpRightTurn(70, 70);
        delay(500);
      } else {
        reverse(80);
        delay(400);
      }

      if (colorActual == "Negro") {
        reverse(70);
        delay(400);
        strafe_left(70);
        delay(300);
      }
    }
  }

  // --- SUBIDA DE LA RAMPA ---
  if (rampa && !rampaTope && !reversa) {
    go_advance(95);
    delay(1000);
    strafe_left(75);
    delay(250);
    stop_bot();
    delay(100);

    colorActual = checarColor();
    mostrarColorRGB(colorActual);

    if (colorActual == "Verde") {
      stop_bot();
      delay(800);
      rampaTope = true;
      reversa = true;  // Activa la reversa cuando llega al verde
    }
  }

  // --- REVERSA DE LA RAMPA ---
  if (reversa) {
    reverse(90);
    delay(900);
    strafe_left(75);
    delay(250);
    stop_bot();
    delay(100);

    colorActual = checarColor();
    mostrarColorRGB(colorActual);

  stop_bot();
  delay(200);
}
}
void explorePistaA(){
  unsigned int distRight = sonar1.ping_cm();
  unsigned int distFL = sonar2.ping_cm();
  unsigned int distFR = sonar3.ping_cm();
  unsigned int distLeft = sonar4.ping_cm();
  unsigned int distBack = sonar5.ping_cm();

  String colorActual = checarColor();

      if (distFR > 10 && distFL > 10) {
        go_advance(90);
        delay(350);
        stop_bot();
        delay(1000);
      } else if (distRight < 1) {
        strafe_left(60);
        delay(150);
        stop_bot();
        delay(1000);

      }else if (distRight > 10){
        right_turn(80);
        delay(800);
        stop_bot();
        delay(1000);
      }
      else if (distLeft > 10){
        left_turn(80);
        delay(800);
        stop_bot();
        delay(1000);
      }
      else if (distLeft < 1) {
        strafe_right(60);
        delay(150);
        stop_bot();
        delay(1000);
      } else if (distFR > distFL) {
        strafe_left(60);
        delay(200);
        stop_bot();
        delay(100);
      } else if (distFL < distFR) {
        strafe_right(60);
        delay(200);
        stop_bot();
        delay(100);
      } else if(distFR < 1 && distFL<1) {
        reverse(80);
        delay(200);
        stop_bot();
        delay(100);
      }
}
void pistAA(){
          delay(3000);
          go_advance(80);
          delay(1300);
          strafe_left(70);
          delay(500);
          stop_bot();
          delay(6000); //para el juez
          go_advance(70);
          delay(300);
          right_turn(75);
          cerrarGarra();
          delay(900);
          stop_bot();
          go_advance(70);
          delay(1350);
          stop_bot();
          delay(1000);
          strafe_left(70);
          delay(500);
          strafe_right(70);
          delay(300);
          go_advance(70);
          delay(1000);
          strafe_left(80);
          delay(300);
          stop_bot();
          abrirGarra();
          delay(5000);//mas tiempo
          cerrarGarra();
          delay(2000);
          go_advance(70);
          delay(600);
          left_turn(70);
          delay(900);
          go_advance(70);
          abrirGarra();
          delay(1000);
          stop_bot();
          delay(10000);
}
//-----PISTA A------
void pistaA(){
  String colorA = checarColor();

  explorePistaA();
  delay(300);

  if (colorA == "Azul"){
    stop_Stop();
    delay(1000);
    abrirGarra();
    delay(3000);
    cerrarGarra();
    delay(5000);
    go_advance(80);
    delay(400);
  }

  String colorActual = checarColor();
  if (colorActual=="Amarillo"){
    go_advance(70);
    delay(450);
    cerrarGarra();
    stop_bot();
    delay(300);
  }
  if (colorActual=="Azul"){
    stop_bot();
    delay(500);
    abrirGarra();
    delay(1000);
  }

  stop_bot();
  delay(300);
  mostrarColorRGB(colorActual);
  delay(1000);
}

// -------- LOOP PRINCIPAL (GOAT)--------

void loop() {
  //checkUltraSounds();
  //explorePistaC1();
  //delay(1000);
  //delay(100);
  //explore();
  //mostrarColorRGB(color);
  //go_advance(SPEED);
  //pistaA();
  pistAA();
  //cerrarGarra();
  //delay(1000);
  //abrirGarra();
  //delay(10000);
}

///PISTA C LABERINTO

