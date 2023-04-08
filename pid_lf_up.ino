void forward();

#define s1 22
#define s2 23
#define s3 24
#define s4 25
#define s5 26

int S1 = 0;
int S2 = 0;
int S3 = 0;
int S4 = 0;
int S5 = 0;

int initial_motor_speed = 50; //can change the initial speed that you prefer

float kp = 2.5;
float ki = 0;
float kd = 1;
float pid_v = 0;
float p = 0, i = 0, d = 0;
float error = 0, prev_error = 0, prev_i = 0;

float R_M_Speed = 0, L_M_Speed = 0;

//Pin declaration for SHIELD-2AMOTOR
int ENA = 2;
int IN1 = 48;
int ENB = 3;
int IN3 = 50;



void setup()
{
  Serial.begin(9600); //Enable Serial Communications
  pinMode(s1, INPUT);
  pinMode(s2, INPUT);
  pinMode(s3, INPUT);
  pinMode(s4, INPUT);
  pinMode(s5, INPUT);

  //Motor Driver Pin Setup
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  digitalWrite(IN1, HIGH); //according to your connection from motor to the motor driver
  digitalWrite(IN3, HIGH); //according to your connection from motor to the motor driver

}

void loop()
{

  S1 = digitalRead(22);
  S2 = digitalRead(23);
  S3 = digitalRead(24);
  S4 = digitalRead(25);
  S5 = digitalRead(26);

 if (S1 == 0 & S2 == 0 & S3 == 1 & S4 == 0 & S5 == 0) {
    error = 0;
  }
   if (S1 == 0 & S2 == 0 & S3 == 0 & S4 == 1 & S5 == 0) {
    error = 1;
  }
   if (S1 == 0 & S2 == 0 & S3 == 1 & S4 == 1 & S5 == 0) {
    error = 2;
  }
   if (S1 == 0 & S2 == 0 & S3 == 0 & S4 == 0 & S5 == 1) {
    error = 3;
  }
   if (S1 == 0 & S2 == 0 & S3 == 0 & S4 == 1 & S5 == 1) {
    error = 4;
  }
   if (S1 == 0 & S2 == 0 & S3 == 1 & S4 == 1 & S5 == 1) {
    error = 5;
  }
   if (S1 == 0 & S2 == 1 & S3 == 1 & S4 == 1 & S5 == 1) {
    error = 6;
  }
   if (S1 == 1 & S2 == 1 & S3 == 1 & S4 == 1 & S5 == 1) {
    error = 7;
  }
   if (S1 == 0 & S2 == 1 & S3 == 0 & S4 == 0 & S5 == 0) {
    error = -1;
  }
   if (S1 == 0 & S2 == 1 & S3 == 1 & S4 == 0 & S5 == 0) {
    error = -2;
  }
   if (S1 == 1 & S2 == 0 & S3 == 0 & S4 == 0 & S5 == 0) {
    error = -3;
  }
   if (S1 == 1 & S2 == 1 & S3 == 0 & S4 == 0 & S5 == 0) {
    error = -4;
  }
   if (S1 == 1 & S2 == 1 & S3 == 1 & S4 == 0 & S5 == 0) {
    error = -5;
  }
   if (S1 == 1 & S2 == 1 & S3 == 1 & S4 == 1 & S5 == 0) {
    error = -6;
  }
   if (S1 == 0 & S2 == 0 & S3 == 0 & S4 == 0 & S5 == 0) {
    error = -9;
  }

  p = error;
  i = i + prev_error;
  d = error - (prev_error);
  pid_v = (kp * p) + (ki * i) + (kd * d);
  prev_i = i;
  prev_error = error;

  R_M_Speed = initial_motor_speed - pid_v;
  L_M_Speed = initial_motor_speed + pid_v;

  
  if (R_M_Speed < 0)
  {
    R_M_Speed = 0;
  }
  if (R_M_Speed > 255)
  {
    R_M_Speed = 255;
  }

  if (L_M_Speed < 0)
  {
    L_M_Speed = 0;
  }
  if (L_M_Speed > 255)
  {
    L_M_Speed = 255;
  }

  forward();

}

void forward()
{
  analogWrite(ENA, R_M_Speed); //Left Motor Speed
  analogWrite(ENB, L_M_Speed); //Right Motor Speed
  delay(1);
}
