#include <SoftwareSerial.h>
#define SOFT_RX 0
#define SOFT_TX 1
SoftwareSerial hcSerial(SOFT_RX, SOFT_TX);
//Special thanks to VSK
#define DEBUG_ROBOT 1                                   //For making code run without usb so that no error for serial communication
#if DEBUG_ROBOT
    #define DUMPS(s)  Serial.println(s)
#else
    #define DUMPS(s)
#endif
const int irR = 11;
const int irMR = 12;
const int irMRR = A0;
const int irMMR = A1;
const int irMML = A2;
const int irMLL = A3;
const int irML = A4;
const int irL = A5;

const int white = 0;
const int black = 1;
int sR,sMR,sMRR,sMMR,sMML,sMLL,sML,sL,c=1;
unsigned long time;
unsigned long Mt1,Motortime,Lt1,Looptime;
float P=0, D=0,I=0,Kp=24,Kd=15, PD_value=0;
float error=0, previous_error=0;

void read_sensor_values(void);

void calculate_pid(void);

void motor_control(void);

void report(void);

////////////////////////////////////////////////////////
char count;
const int MOTOR_RIGHT = 1;                             //Declaring ports with constants
const int MOTOR_LEFT = 2;

const int MR_CONTROL_PIN_A = 2;
const int MR_CONTROL_PIN_B = 3;
const int MR_PWM_PIN       = 9;

const int ML_CONTROL_PIN_A = 4;
const int ML_CONTROL_PIN_B = 5;
const int ML_PWM_PIN       = 10;
////////////////////////////////////////////////////////
void setup() {                                          //Setup
  pinMode(irR,INPUT);
  pinMode(irMR,INPUT);
  pinMode(irMRR,INPUT);
  pinMode(irMMR,INPUT);
  pinMode(irMML,INPUT); 
  pinMode(irMLL,INPUT);
  pinMode(irML,INPUT);
  pinMode(irL,INPUT); 
  /////////////////////////////////////////////////////
  pinMode(MR_CONTROL_PIN_A, OUTPUT);
  pinMode(MR_CONTROL_PIN_B, OUTPUT);
  pinMode(MR_PWM_PIN, OUTPUT);

  pinMode(ML_CONTROL_PIN_A, OUTPUT);
  pinMode(ML_CONTROL_PIN_B, OUTPUT);
  pinMode(ML_PWM_PIN, OUTPUT);
  Serial.begin(9600);
  hcSerial.begin(38400);
  Lt1 = millis();
}
////////////////////////////////////////////////////////
void loop() {                                           //Functions are called in in loop for one motor 
 void sensor_values();
    DUMPS(c);
    sensor_values();
 
    DUMPS(" ERROR:");
    DUMPS(error);

    calculate_pid();

    DUMPS(" P:");
    DUMPS(P);
    
    DUMPS(" D:");
    DUMPS(D);
    
    motor_control();
   c = c + 1;
}
///////////////////////////////////////////////////////
void forward(int motorNo,int motorSpeed)               //For making motor to be run for forward direction with speed
{
  if(motorSpeed <0 || motorSpeed > 255)
  {
    DUMPS("\r\n ERROR: Invalid motor speed: ");
    DUMPS(motorSpeed);
    DUMPS("\r\n [Suggestion: Valid motorSpeed range is 0 to 255]");    
  }
switch(motorNo)
{
  case MOTOR_RIGHT:
    analogWrite(MR_PWM_PIN,motorSpeed);
    digitalWrite(MR_CONTROL_PIN_A,HIGH);
    digitalWrite(MR_CONTROL_PIN_B,LOW);
    break;
    
  case MOTOR_LEFT:
    analogWrite(ML_PWM_PIN,motorSpeed);
    digitalWrite(ML_CONTROL_PIN_A,HIGH);
    digitalWrite(ML_CONTROL_PIN_B,LOW);
    break;

  default:
    DUMPS("\r\n ERROR: Invalid motor no: ");
    DUMPS(motorNo);
}
}
////////////////////////////////////////////////////////
void reverse(int motorNo,int motorSpeed)                //For making motor to be run for reverse direction with speed
{
  DUMPS("\r\n REVERSE: ");
  DUMPS(motorNo);
  if(motorSpeed <0 || motorSpeed > 255)
  {
    DUMPS("\r\n ERROR: Invalid motor speed: ");
    DUMPS(motorSpeed);
    DUMPS("\r\n [Suggestion: Valid motorSpeed range is 0 to 255]");    
  }
switch(motorNo)
{
  case MOTOR_RIGHT:
    analogWrite(MR_PWM_PIN,motorSpeed);
    digitalWrite(MR_CONTROL_PIN_A,LOW);
    digitalWrite(MR_CONTROL_PIN_B,HIGH);
    break;
    
  case MOTOR_LEFT:
    analogWrite(ML_PWM_PIN,motorSpeed);
    digitalWrite(ML_CONTROL_PIN_A,LOW);
    digitalWrite(ML_CONTROL_PIN_B,HIGH);
    break;

  default:
    DUMPS("\r\n ERROR: Invalid motor no: ");
    DUMPS(motorNo);
}
}
////////////////////////////////////////////////////////
void stopMotor(int motorNo)                             //For stopping motor
{
  DUMPS("\r\n STOP: ");
  DUMPS(motorNo);
switch(motorNo)
{
  case MOTOR_RIGHT:
    digitalWrite(MR_CONTROL_PIN_A,LOW);
    digitalWrite(MR_CONTROL_PIN_B,LOW);
    break;
    
  case MOTOR_LEFT:
    digitalWrite(ML_CONTROL_PIN_A,LOW);
    digitalWrite(ML_CONTROL_PIN_B,LOW);
    break;

  default:
    DUMPS("\r\n ERROR: Invalid motor no: ");
    DUMPS(motorNo);
}
}
////////////////////////////////////////////////////////
void calculate_pid()

{  

    P = error;

    D = error - previous_error;


    PD_value = (Kp*P) + (Kd*D);
    
    previous_error = error;

}
/////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
void sensor_values()
{
 sR = digitalRead(irR);
 sMR= digitalRead(irMR);
 sMRR=digitalRead(irMRR);
 sMMR=digitalRead(irMMR);
 sMML=digitalRead(irMML);
 sMLL=digitalRead(irMLL);
 sML= digitalRead(irML);
 sL = digitalRead(irL);

 if(sML==1&&sMLL==1&&sMML==0&&sMMR==0&&sMRR==1&&sMR==1)
 {
  error=0;
  count = 'S';
  DUMPS(count);
 }
 
 if(sML==1&&sMLL==1&&sMML==1&&sMMR==0&&sMRR==0&&sMR==1)
 { 
  error=-1;
 }
 ///
 if(sML==1&&sMLL==1&&sMML==1&&sMMR==1&&sMRR==0&&sMR==0)
 { 
  error=-2;
 }
 /*if(sML==1&&sMLL==1&&sMML==1&&sMMR==0&&sMRR==0&&sMR==0)
 { 
  error=-3;
 }
 
 if(sML==0&&sMLL==0&&sMML==0&&sMMR==1&&sMRR==1&&sMR==1&&sR==1)
 {
  error=3;
 }
 */
 if(sML==0&&sMLL==0&&sMML==1&&sMMR==1&&sMRR==1&&sMR==1)
 { 
  error=2;
 }
 ///
 if(sML==1&&sMLL==0&&sMML==0&&sMMR==1&&sMRR==1&&sMR==1)
 { 
  error=1;
 }
 if(sL==0&&sML==0&&sMLL==0&&sMML==0&&sMMR==0&&sMRR==0&&sMR==1&&sR==1||sL==0&&sML==0&&sMLL==0&&sMML==0&&sMMR==0&&sMRR==1&&sMR==1&&sR==1) /* left turn*/
 {
  delay(100);
  count='L';
  DUMPS(count);
 }
 if(sL==1&&sML==1&&sMLL==1&&sMML==1&&sMMR==1&&sMRR==1&&sMR==1&&sR==1&&count=='L')
  {
  reverse(MOTOR_RIGHT,150); 
  forward(MOTOR_LEFT,150);
  delay(360);
  }
if(sL==1&&sML==1&&sMLL==0&&sMML==0&&sMMR==0&&sMRR==0&&sMR==0&&sR==0||sL==1&&sML==1&&sMLL==1&&sMML==0&&sMMR==0&&sMRR==0&&sMR==0&&sR==0)/* rigth turn */ 
 { 
  delay(100);
  count='R';
  DUMPS(count);
 }
 if(sL==1&&sML==1&&sMLL==1&&sMML==1&&sMMR==1&&sMRR==1&&sMR==1&&sR==1&&count=='R')
  {
  forward(MOTOR_RIGHT,150); 
  reverse(MOTOR_LEFT,150);
  delay(360);
  }
 if(sR==0&&sMR==0&&sMRR==0&&sMMR==0&&sMML==0&&sMLL==0&&sML==0&&sL==0)
 {
  delay(100);
  count='T'; 
  DUMPS(count);
 }
 if(sR==0&&sMR==0&&sMRR==0&&sMMR==0&&sMML==0&&sMLL==0&&sML==0&&sL==0)
 {
  count = 'S'; 
  DUMPS(count); 
  stopMotor(MOTOR_LEFT);
  stopMotor(MOTOR_RIGHT);
  report();
 }      
}
//////////////////////////////////////////////////////////
void motor_control()

{

    // Calculating the effective motor speed:
      forward(MOTOR_RIGHT,130-PD_value);
      forward(MOTOR_LEFT,130+PD_value);


}
float velocity,distance,sec_per_cm;
void report()
{ 
  distance=202;
  Motortime=millis()-Mt1;
  DUMPS("Time: ");
  DUMPS(Motortime);
  DUMPS("milisecond");  
  ///////////////////////////
  velocity=((distance*1000)/Motortime);
  DUMPS("\n\rspeed: ");
  DUMPS(velocity);
  DUMPS("cm/sec");
  ///////////////////////////
  sec_per_cm=(1/velocity);
  DUMPS("\n\rsec_per_cm");
  DUMPS(sec_per_cm);
  DUMPS("sec/cm");
  while(1); 
 }
