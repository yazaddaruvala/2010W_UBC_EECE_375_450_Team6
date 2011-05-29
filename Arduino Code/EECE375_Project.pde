#include <MotorControler.h>
#include <Servo.h>
const int RIGHT_POWER =  11;
const int RIGHT_SELECT =  7;
const int RIGHT_ENCODER =  0; //PIN 2
const int LEFT_POWER =  6;
const int LEFT_SELECT = 12;
const int LEFT_ENCODER = 1; //PIN 3
Motor rightMotor = Motor(RIGHT_POWER, RIGHT_SELECT, 175);
Motor leftMotor = Motor(LEFT_POWER, LEFT_SELECT, 188);
MotorControler controller( &rightMotor, &leftMotor );

//If someone can fill these out I think they are all we need to
//convert "ticks" to distance/degrees
#define OUTTER_ROBOT_DIAMETER 2180
#define WHEEL_DIAMETER 6.3
#define TICKS_PER_WHEEL_ROTATION 18
#define DISTANCE_PER_TICK WHEEL_DIAMETER/TICKS_PER_WHEEL_ROTATION
#define TICKS_PER_ROBOT_ROTATION OUTTER_ROBOT_DIAMETER/DISTANCE_PER_TICK
#define TICKS_PER_DEGREE TICKS_PER_ROBOT_ROTATION/360
#define TICKS_PER_DISTANCE 1/DISTANCE_PER_TICK

const int FRONT_LEFT_SENSOR = A3;
const int FRONT_RIGHT_SENSOR = A5;

const int BACK_RIGHT_SENSOR = A1;
const int BACK_LEFT_SENSOR = A0;

Servo myservo;
Servo myservo2;

void setup(){
  Serial.begin(57600);
  pinMode(FRONT_RIGHT_SENSOR, INPUT);
  pinMode(FRONT_LEFT_SENSOR, INPUT);
  attachInterrupt( RIGHT_ENCODER, right_Encoder, RISING );
  attachInterrupt( LEFT_ENCODER, left_Encoder,  RISING);
  myservo.attach(13);
  myservo.writeMicroseconds(2200);
  myservo2.attach(4);
  myservo2.write(180);
}

void loop(){
  //Get inputs if possible
  handleInput();

  //Correct Wheel Speed
  if( controller.checkState() != 0 ){
    controller.allignWheels();

    switch(controller.checkState()){
     case 1:
      //Check Sensors
      checkFrontSensors();
      break;
    case 2:
      break;
     case 3: //Right
       if (analogRead(BACK_LEFT_SENSOR) > 680){
         controller.stopRun();
         controller.goForward(10);
       }
       break;
    case 4:
       if (analogRead(BACK_RIGHT_SENSOR) > 680){
         controller.stopRun();
         controller.goForward(10);
       }
       break;
    default:
      break;
    }
  }
}

void right_Encoder( void ){
  controller.updateRightEncoder();
}

void left_Encoder( void ){
  controller.updateLeftEncoder();
}

void checkFrontSensors( void ){
  int RightValue = analogRead(FRONT_RIGHT_SENSOR);
  int LeftValue = analogRead(FRONT_LEFT_SENSOR);
  
  if ( (RightValue > 420) && (LeftValue > 420) ){
    controller.stopRun();
    controller.goBackward( 10 );
  }
  else if (RightValue > 420)
  {
    //controller.slightLeft();
    leftMotor.pauseRun();
  }
  else if (LeftValue > 420)
  {
    //controller.slightRight();
    rightMotor.pauseRun();
  } else {
      leftMotor.resumeRun();
      rightMotor.resumeRun();
  }
}

int number = 0, Speed = 200; char inByte, val[3];
//accuracy to about 4 degrees
// 11 is about 45*
// 25 is about 90*
// 40 is about 135*
// 57 is about 180*
void handleInput( void ){
  if (Serial.available() >= 4)
  {
    inByte = Serial.read();
    val[0] = Serial.read();
    val[1] = Serial.read();
    val[2] = Serial.read();
    number = atoi(val);
//    if (number == 0)
//      return;

      
   // Serial.println(inByte);
   // Serial.println(number);
    
    switch( inByte ){
      case 'a':
        //number = number*.3333333;
        //rightEncoder = number*0.91;
        //leftEncoder = number;
        controller.turnLeft( number ); // *TICKS_PER_DEGREE
        break;
      case 'd':
        //number = number*.333333;
        //rightEncoder = number*0.91;
        //leftEncoder = number;
        controller.turnRight( number ); //*TICKS_PER_DEGREE
        break;
      case 's':
      //MINIMUM 8cm
      
        //rightEncoder = number*0.96*1.65;
        //leftEncoder = number*1.65;
        controller.goBackward( number ); //*TICKS_PER_DISTANCE
        break;
      case 'w':
      //MINIMUM 8cm
        //rightEncoder = number*0.96*1.65;
        //leftEncoder = number*1.65;
        controller.goForward( number  ); //*TICKS_PER_DISTANCE
        break;
      case 'e':
        controller.stopRun();
        break;
      case 'c':
        //CAPTURE BALLS
        
        break;
      case 'q':
        //RELEASE BALLS
        if (number == 0){
            myservo.writeMicroseconds(900);
        }
        if (number == 255) {
            myservo.writeMicroseconds(2200);
        }
        break;
      default:
        break;
    }
  }
}
