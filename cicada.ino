#include <NewPing.h>
#include <Wire.h>

#define BRAKEVCC 0
#define CW   1
#define CCW  2
#define BRAKEGND 3
#define CS_THRESHOLD 100
#define PWM_MOTOR_MAX  255
#define PWM_MOTOR_HALF 127
/*  VNH2SP30 pin definitions
 xxx[0] controls '1' outputs
 xxx[1] controls '2' outputs */
int inApin[2] = {7, 4};  // INA: Clockwise input
int inBpin[2] = {8, 9}; // INB: Counter-clockwise input
int pwmpin[2] = {5, 6}; // PWM input
int cspin[2] = {2, 3}; // CS: Current sense ANALOG input
int enpin[2] = {0, 1}; // EN: Status of switches output (Analog pin)
int statpin = 13;

#define CUTTING_RELAY 14
#define RIGHT_TRIG 15
#define RIGHT_ECHO 16
#define CENTER_TRIG 18
#define CENTER_ECHO 19
#define LEFT_TRIG 20
#define LEFT_ECHO 21
#define MAX_DISTANCE 120

#define ENABLE_CUTTING 1


NewPing sonarRight(RIGHT_TRIG, RIGHT_ECHO, MAX_DISTANCE);
NewPing sonarCenter(CENTER_TRIG, CENTER_ECHO, MAX_DISTANCE);
NewPing sonarLeft(LEFT_TRIG, LEFT_ECHO, MAX_DISTANCE);

//Buffer
int distanceRight = -1;
int distanceCenter = -1;
int distanceLeft = -1;


void setupMotor() {
  pinMode(statpin, OUTPUT);

  // Initialize digital pins as outputs
  for (int i = 0; i < 2; i++)
  {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }
  // Initialize braked
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
}

void setup() {
  Serial.begin(115200);
  
  setupMotor();
  if ( ENABLE_CUTTING == 1) {
    pinMode(CUTTING_RELAY, OUTPUT);
    delay(3000);
    cuttingOn();
    delay(3000);
  }
}

/* motorGo() will set a motor going in a specific direction
 the motor will continue going in that direction, at that speed
 until told to do otherwise.

 motor: this should be either 0 or 1, will selet which of the two
 motors to be controlled

 direct: Should be between 0 and 3, with the following result
 0: Brake to VCC
 1: Clockwise
 2: CounterClockwise
 3: Brake to GND

 pwm: should be a value between ? and 1023, higher the number, the faster
 it'll go
 */
void motorLeft(uint8_t direct, uint8_t pwm)
{
  if (direct == CW) {
    digitalWrite(inApin[0], HIGH);
    digitalWrite(inBpin[0], LOW);
  }  else {
    digitalWrite(inApin[0], LOW);
    digitalWrite(inBpin[0], HIGH);
  }
  analogWrite(pwmpin[0], pwm);
}

void motorRight(uint8_t direct, uint8_t pwm)
{
  if (direct == CW) {
    digitalWrite(inApin[1], HIGH);
    digitalWrite(inBpin[1], LOW);
  } else {
    digitalWrite(inApin[1], LOW);
    digitalWrite(inBpin[1], HIGH);
  }
  analogWrite(pwmpin[1], pwm);
}

void motorOff(int motor)
{
  // Initialize braked
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  analogWrite(pwmpin[motor], 0);
}

void motorAllOff() {
  motorOff(0);
  motorOff(1);
}

void robotToLeft(int speed) {
  motorLeft(CCW, speed);
  motorRight(CCW, speed);
}

void robotToRight(int speed) {
  motorLeft(CW, speed);
  motorRight(CW, speed);
}

void cuttingOn() {
  digitalWrite(CUTTING_RELAY, HIGH);
}

void cuttingOff() {
  digitalWrite(CUTTING_RELAY, LOW);
}

void motorForward(int speed) {
  motorLeft(CCW, (uint8_t)speed);
  motorRight(CW, (uint8_t)speed);
}

void motorBack(int speed) {
  motorLeft(CW, (uint8_t)speed);
  motorRight(CCW, (uint8_t)speed);
}

int currentSpeed = PWM_MOTOR_MAX;

void loop() {
  motorForward(currentSpeed);

  delay(50);
  distanceRight = sonarRight.ping_cm();
  delay(50);
  distanceCenter = sonarCenter.ping_cm();
  delay(50);
  distanceLeft = sonarLeft.ping_cm();

  Serial.print("A: ");
  Serial.print(distanceRight);
  Serial.print(" B: ");
  Serial.print(distanceCenter);
  Serial.print(" C: ");
  Serial.print(distanceLeft);
  Serial.print("\n");
  
    //Potential
    if (( distanceLeft > 16 && distanceLeft < 40) || ( distanceCenter > 16 && distanceCenter < 40) || ( distanceRight > 16 && distanceRight < 40))
    {
      currentSpeed = PWM_MOTOR_HALF;
    }

    //Very short distance
    if (( distanceLeft > 0 && distanceLeft < 15) || ( distanceCenter > 0 && distanceCenter < 15) || ( distanceRight > 0 && distanceRight < 15))
    {
      motorAllOff();
      delay(1000);
      motorBack(PWM_MOTOR_HALF);
      delay(1000);
      motorAllOff();

      if ( distanceRight > distanceCenter)
        robotToRight(PWM_MOTOR_HALF);
      else
        robotToLeft(PWM_MOTOR_HALF);
      delay(500);
      while(distanceLeft < 40 && distanceCenter < 40 && distanceRight < 40 || distanceLeft == 0 || distanceCenter == 0 || distanceRight == 0) {
        delay(50);
        distanceRight = sonarRight.ping_cm();
        delay(50);
        distanceCenter = sonarCenter.ping_cm();
        delay(50);
        distanceLeft = sonarLeft.ping_cm();
    }
      motorAllOff();
    }

    if (( distanceRight > 30) || ( distanceCenter > 30))
    {

      currentSpeed = PWM_MOTOR_MAX;
    }

    if ((analogRead(cspin[0]) < CS_THRESHOLD) && (analogRead(cspin[1]) < CS_THRESHOLD))
      digitalWrite(statpin, HIGH);
      
}




