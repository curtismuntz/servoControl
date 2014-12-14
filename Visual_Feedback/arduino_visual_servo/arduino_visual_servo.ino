/************************************************************
Project   : visualServo.ino
Created By: Curtis Muntz
Date      : 14 July 2014
Usage     : 

************************************************************
Concept   : 1) visual servo-ing

Requires  : sudo apt-get install ros-indigo-rosserial-arduino
          : rosrun rosserial_arduino make_libraries.py /home/muntzcurtis/sketchbook/libraries/

ROSNODE   : rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0

LISTENS TO: objectLocation
          : ex) rostopic pub objectLocation std_msgs/UInt16 240

TODO      : Test this code


referenced urls:
  * my old servo code, modified: https://github.com/curtismuntz/servoControl.git
  * rosserial_arduino tutorial: http://wiki.ros.org/rosserial_arduino/Tutorials/Servo%20Controller
*************************************************************/
 
#include "Servo.h" // probably not used if HBridge motor is being used.
#include <ros.h>
#include <std_msgs/UInt16.h>

void objectLocation_cb( const std_msgs::UInt16& cmd_msg);


Servo servo; // probably not used, if HBridge motor is being used.
ros::NodeHandle nh;
ros::Subscriber<std_msgs::UInt16> sub("objectLocation", objectLocation_cb);

#define LED_PIN 13
#define M_PI 3.14159265359
#define GEAR_RATIO 0.04       // 250:1
#define PWM_A 3               // Speed control 
#define MOTORA_IN1 9          // motor A connected between A01 and A02
#define MOTORA_IN2 8          // not used
#define STANDBY     10        // standby pin


bool blinkState=false;
bool debug = false; // set to true for testing/debugging.
int motorVolume =0; // <-- sash's constant


// timing parameters
int last_update;
int cycle_time;
long last_cycle = 0;
int timerVal = 50; //unused 


// PID function variables
float setPoint  = 320;
float integral  = 0.00;
float error_old = 0.00;

// PID constants . these should be computed in the calibrate function.
double Kp = 2;
double Ki = 0.5;
double Kd = 0;

int pixel = 0;




void objectLocation_cb( const std_msgs::UInt16& cmd_msg)
{
  pixel = cmd_msg.data;
}


/**************************************************************
FUNCTION: setup()
  
**************************************************************/
void setup()
{
  Serial.begin(9600);    //  initialize serial communication. needed for println
  nh.initNode();
  nh.subscribe(sub);

  //H-Bridge Standby pin
  pinMode(STANDBY, OUTPUT);

  //H-Bridge Motor 1 pins
  pinMode(PWM_A, OUTPUT);
  pinMode(MOTORA_IN1, OUTPUT);
  pinMode(MOTORA_IN2, OUTPUT);


  //Arduino blinky light thing
  pinMode(LED_PIN, OUTPUT);  // configure LED pin

 
  Serial.println("Initialization complete.....");
  Serial.print("Kp = ");
  Serial.println(Kp);
  Serial.print("Ki = ");
  Serial.println(Ki);
  Serial.print("Kd = ");
  Serial.println(Kd);

  // set first cycle time, the rest will be defined by the timer funtion
  cycle_time = timerVal * .001;

}


/***************************************************************
FUNCTION compute()
  computes the PID, storing controller signal in global variable yOutput
      previous_error = 0
      integral = 0 
      start:
      error = setpoint - measured_value
      integral = integral + error*dt
      derivative = (error - previous_error)/dt
      output = Kp*error + Ki*integral + Kd*derivative
      previous_error = error
      wait(dt)
      goto start
***************************************************************/
float compute(int pixelLocation)
{
  int error = setPoint - pixelLocation;
  //integral
  float integral = integral + (error * (cycle_time*.001));
  //derivative
  float derivative = (error - error_old)/(cycle_time*.001);
  float output = (Kp*error) + (Ki*integral) + (Kd*derivative);

  error_old = error;

  return output;

}


/***************************************************************
FUNCTION move(int motor, int, int)
 * Sets motor speed and direction for a given motor number
 * Modified from the sparkfun H-bridge example code
***************************************************************/
void move(int motor, int speedz, int direction)
{
  digitalWrite(STANDBY, HIGH); //disable standby
  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if(direction == 1){
    inPin1 = HIGH;
    inPin2 = LOW;
  }

  if(motor == 1){
    digitalWrite(MOTORA_IN1, inPin1);
    digitalWrite(MOTORA_IN2, inPin2);
    analogWrite(PWM_A, speedz);
  }
}


/***************************************************************
FUNCTION stop()
    stops motor by enabling standby pin on the Hbridge
***************************************************************/
void stop()
{
  digitalWrite(STANDBY, LOW); 
}

/***************************************************************
FUNCTION timing()
  calculates total cycle time for accurate sample time calculations
***************************************************************/
void timing()
{
  // ***** if forced timing is required uncomment the while code.
  
  // while((millis() - last_cycle) < timerVal){
  //  delay(1);
  // }

  cycle_time = millis() - last_cycle;
  last_cycle = millis();
}



/**************************************************************
FUNCTION: outputCalc
returns the constrained pwm value
**************************************************************/
int outputCalc(float measurement)
{
  int pwmval;
  return pwmval = constrain(abs((int)measurement),0,255);
}


/**************************************************************
FUNCTION: outputDir
returns the direction for the motor to turn
**************************************************************/
int outputDir(float measurement)
{
    int directionz;
    if (measurement < 0)
    {
      directionz=1;
    }
    else
    {
      directionz = 0;
    }
    return directionz;
}

/***************************************************************
FUNCTION read

***************************************************************/
int read()
{
  return pixel;
}


/***************************************************************
Function: MAIN LOOP
  performs data gathering, computing, and executing.
***************************************************************/
void loop()
{
  //initial set point should be half way between image start and image end (middle of image)
  int setPoint = 240;
  int location = 200;

// Control loop to keep object in the center of the image frame.
  while(true)
    {

      int location = read();
      //Serial.print("Location: ");
      //Serial.println(location);
      //compute PID, execute servo PWM write
      float measurement = compute(location);

      int go = outputCalc(measurement);
      int dir= outputDir(measurement);

      if (go < 2)
      {
        go = 0;
      }
      move(1, go, dir); // move motor 1, go=speed, dir=direction

      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
      timing();
      

      nh.spinOnce();
      delay(1);
    }
}
