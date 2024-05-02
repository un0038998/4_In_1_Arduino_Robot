#include <Servo.h>
#include <NewPing.h>

#define OBSTACLE_AVOIDING_MODE                    1
#define LINE_FOLLOWER_MODE                        2
#define LINE_FOLLOWER_WITH_OBSTACLE_AVOIDING_MODE 3
#define OBJECT_FOLLOWING_MODE                     4
#define ROBOT_CONTROL_MODE OBSTACLE_AVOIDING_MODE

#define SERVO_PIN 3
#define ULTRASONIC_SENSOR_TRIG 11
#define ULTRASONIC_SENSOR_ECHO 12

#define BOTTOM_IR_SENSOR_RIGHT A0
#define BOTTOM_IR_SENSOR_LEFT A1
#define TOP_IR_SENSOR_RIGHT A3
#define TOP_IR_SENSOR_LEFT A2


//Right motor
int enableRightMotor=5;
int rightMotorPin1=7;
int rightMotorPin2=8;

//Left motor
int enableLeftMotor=6;
int leftMotorPin1=9;
int leftMotorPin2=10;

NewPing mySensor(ULTRASONIC_SENSOR_TRIG, ULTRASONIC_SENSOR_ECHO, 400);
Servo myServo;

void setup()
{
  // put your setup code here, to run once:
  pinMode(enableRightMotor,OUTPUT);
  pinMode(rightMotorPin1,OUTPUT);
  pinMode(rightMotorPin2,OUTPUT);
  
  pinMode(enableLeftMotor,OUTPUT);
  pinMode(leftMotorPin1,OUTPUT);
  pinMode(leftMotorPin2,OUTPUT);

  myServo.attach(SERVO_PIN);
  myServo.write(90);

  pinMode(BOTTOM_IR_SENSOR_RIGHT, INPUT);
  pinMode(BOTTOM_IR_SENSOR_LEFT, INPUT);

  pinMode(TOP_IR_SENSOR_RIGHT, INPUT);
  pinMode(TOP_IR_SENSOR_LEFT, INPUT);
    
  rotateMotor(0,0);   
}

void obstacleAvoidingControl()
{
  static int MAX_REGULAR_MOTOR_SPEED  = 150;
  static int MAX_MOTOR_TURN_SPEED  = 200;
  static int DISTANCE_TO_CHECK  = 30;
  
  int distance = mySensor.ping_cm();

  //If distance is within 30 cm then adjust motor direction as below
  if (distance > 0 && distance < DISTANCE_TO_CHECK)
  {
    //Stop motors
    rotateMotor(0, 0);
    delay(500);  
       
    //Reverse motors
    rotateMotor(-MAX_MOTOR_TURN_SPEED, -MAX_MOTOR_TURN_SPEED);        
    delay(200);
    
    //Stop motors
    rotateMotor(0, 0);
    delay(500);
    
    //Rotate servo to left    
    myServo.write(180);
    delay(500);

    //Read left side distance using ultrasonic sensor
    int distanceLeft = mySensor.ping_cm();    

    //Rotate servo to right
    myServo.write(0);    
    delay(500);    

    //Read right side distance using ultrasonic sensor   
    int distanceRight = mySensor.ping_cm();

    //Bring servo to center
    myServo.write(90); 
    delay(500);        
    
    if (distanceLeft == 0 )
    {
      rotateMotor(MAX_MOTOR_TURN_SPEED, -MAX_MOTOR_TURN_SPEED);
      delay(500);
    }
    else if (distanceRight == 0 )
    {
      rotateMotor(-MAX_MOTOR_TURN_SPEED, MAX_MOTOR_TURN_SPEED);
      delay(500);
    }
    else if (distanceLeft >= distanceRight)
    {
      rotateMotor(MAX_MOTOR_TURN_SPEED, -MAX_MOTOR_TURN_SPEED);
      delay(500);
    }
    else
    {
      rotateMotor(-MAX_MOTOR_TURN_SPEED, MAX_MOTOR_TURN_SPEED);
      delay(500);      
    }
    rotateMotor(0, 0);    
    delay(200);     
  }
  else
  {
    rotateMotor(MAX_REGULAR_MOTOR_SPEED, MAX_REGULAR_MOTOR_SPEED);
  }  
}

void lineFollowerControl()
{
  static int MOTOR_SPEED = 100;
  static int TURNING_MOTOR_SPEED = 200;
  
  int rightIRSensorValue = digitalRead(BOTTOM_IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(BOTTOM_IR_SENSOR_LEFT);

  //If none of the sensors detects black line, then go straight
  if (rightIRSensorValue == LOW && leftIRSensorValue == LOW)
  {
    rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
  }
  //If right sensor detects black line, then turn right
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW )
  {
      rotateMotor(-TURNING_MOTOR_SPEED, TURNING_MOTOR_SPEED); 
  }
  //If left sensor detects black line, then turn left  
  else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH )
  {
      rotateMotor(TURNING_MOTOR_SPEED, -TURNING_MOTOR_SPEED); 
  } 
  //If both the sensors detect black line, then stop 
  else 
  {
    rotateMotor(0, 0);
  }
}

void lineFollowerWithObstacleAvoidingControl()
{
  static int MOTOR_SPEED = 100;
  static int TURNING_MOTOR_SPEED = 200;
  static int DISTANCE_TO_CHECK  = 20;
    
  int rightIRSensorValue = digitalRead(BOTTOM_IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(BOTTOM_IR_SENSOR_LEFT);

  //If none of the sensors detects black line, then go straight
  if (rightIRSensorValue == LOW && leftIRSensorValue == LOW)
  {
    int distance = mySensor.ping_cm();
    //If object detected that is if distance is within 30 cm then adjust motor direction as below
    if (distance > 0 && distance < DISTANCE_TO_CHECK)
    {
      adjustRobotToAvoidObjectWhileFollowingLine();
    }
    else
    {
      rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
    }
  }
  //If right sensor detects black line, then turn right
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW )
  {
      rotateMotor(-TURNING_MOTOR_SPEED, TURNING_MOTOR_SPEED); 
  }
  //If left sensor detects black line, then turn left  
  else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH )
  {
      rotateMotor(TURNING_MOTOR_SPEED, -TURNING_MOTOR_SPEED); 
  } 
  //If both the sensors detect black line, then stop 
  else 
  {
    rotateMotor(0, 0);
  }  
}


void objectFollowingControl()
{
  static int MAX_FORWARD_MOTOR_SPEED  = 150;
  static int MAX_MOTOR_TURN_SPEED_ADJUSTMENT  = 100;
  static int MIN_DISTANCE  = 10;
  static int MAX_DISTANCE  = 30;
  
  int distance = mySensor.ping_cm();
  int rightIRSensorValue = digitalRead(TOP_IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(TOP_IR_SENSOR_LEFT);

  //NOTE: If IR sensor detects the hand then its value will be LOW else the value will be HIGH
  
  //If right sensor detects hand, then turn right. We increase left motor speed and decrease the right motor speed to turn towards right
  if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH )
  {
      rotateMotor(MAX_FORWARD_MOTOR_SPEED - MAX_MOTOR_TURN_SPEED_ADJUSTMENT, MAX_FORWARD_MOTOR_SPEED + MAX_MOTOR_TURN_SPEED_ADJUSTMENT ); 
  }
  //If left sensor detects hand, then turn left. We increase right motor speed and decrease the left motor speed to turn towards left
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW )
  {
      rotateMotor(MAX_FORWARD_MOTOR_SPEED + MAX_MOTOR_TURN_SPEED_ADJUSTMENT, MAX_FORWARD_MOTOR_SPEED - MAX_MOTOR_TURN_SPEED_ADJUSTMENT); 
  }
  //If distance is between min and max then go straight
  else if (distance >= MIN_DISTANCE && distance <= MAX_DISTANCE)
  {
    rotateMotor(MAX_FORWARD_MOTOR_SPEED, MAX_FORWARD_MOTOR_SPEED);
  }
  //stop the motors
  else 
  {
    rotateMotor(0, 0);
  }  
}

void adjustRobotToAvoidObjectWhileFollowingLine()
{
  static int MAX_MOTOR_TURN_SPEED  = 200;

  //Stop motors
  rotateMotor(0, 0);
  delay(500);
    
  //Rotate servo to left    
  myServo.write(180);
  delay(500);

  //Read left side distance using ultrasonic sensor
  int distanceLeft = mySensor.ping_cm();    

  //Rotate servo to right
  myServo.write(0);    
  delay(500);    

  //Read right side distance using ultrasonic sensor   
  int distanceRight = mySensor.ping_cm();

  //Bring servo to center
  myServo.write(90); 
  delay(500);        

  bool robotMovedToLeftDirection = false;
  if (distanceLeft == 0 )
  {
    robotMovedToLeftDirection = true;
    rotateMotor(MAX_MOTOR_TURN_SPEED, -MAX_MOTOR_TURN_SPEED);
    delay(500);
  }
  else if (distanceRight == 0 )
  {
    rotateMotor(-MAX_MOTOR_TURN_SPEED, MAX_MOTOR_TURN_SPEED);
    delay(500);
  }
  else if (distanceLeft >= distanceRight)
  {
    robotMovedToLeftDirection = true;
    rotateMotor(MAX_MOTOR_TURN_SPEED, -MAX_MOTOR_TURN_SPEED);
    delay(500);
  }
  else
  {
    rotateMotor(-MAX_MOTOR_TURN_SPEED, MAX_MOTOR_TURN_SPEED);
    delay(500);      
  }
  
  rotateMotor(MAX_MOTOR_TURN_SPEED, MAX_MOTOR_TURN_SPEED);    
  delay(500); 

  if (robotMovedToLeftDirection)
  {
    rotateMotor(-MAX_MOTOR_TURN_SPEED, MAX_MOTOR_TURN_SPEED);
    delay(800);  
  }
  else
  {
    rotateMotor(MAX_MOTOR_TURN_SPEED, -MAX_MOTOR_TURN_SPEED);
    delay(800);  
  }
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,HIGH);    
  }
  else if (rightMotorSpeed >= 0)
  {
    digitalWrite(rightMotorPin1,HIGH);
    digitalWrite(rightMotorPin2,LOW);      
  }

  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,HIGH);    
  }
  else if (leftMotorSpeed >= 0)
  {
    digitalWrite(leftMotorPin1,HIGH);
    digitalWrite(leftMotorPin2,LOW);      
  }

  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));    
}

void loop()
{
  switch (ROBOT_CONTROL_MODE)
  {
    case OBSTACLE_AVOIDING_MODE:
      obstacleAvoidingControl();
      break;

    case LINE_FOLLOWER_MODE:
      lineFollowerControl();
      break;

    case LINE_FOLLOWER_WITH_OBSTACLE_AVOIDING_MODE:
      lineFollowerWithObstacleAvoidingControl();
      break;

    case OBJECT_FOLLOWING_MODE:
      objectFollowingControl();
      break;

    default:
      break;  
  }
 
}
