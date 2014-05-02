/**
 * @file servoControl.ino
 * This program is designed to control a custom built servo motor using PID control theory.
 *
 * @author  Curtis Muntz <cmuntz@outlook.com>
 * @author  David Larribas <dlarribas@gmail.com>
 * @version 0.0.1
 *
 *
 * @section DESCRIPTION
 * This program is woo like woo.
 * 
 */
#include <Wire.h>


// include the Adafruit libraries if using seven seg display
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

Adafruit_7segment matrix = Adafruit_7segment();


#define PIN_ENCODER_A 0       // DIGITAL PIN 0
#define PIN_ENCODER_B 2       // DIGITAL PIN 2
#define PINx  PIND            // ask david for the pin letter... otherwise i'll set it to C for no reason... -curtis
#define GEAR_RATIO 0.9375     // 250:1 gear ratio at 24 pulses per rotation gives 1 pulse is 0.9375 degrees output rotation
#define UNGEAR_RATIO 15       // 24 pulses per rotation on encoder yeilds 15 degree angle resolution
#define PWM_A 3               // Speed control 
#define MOTORA_IN1 9          // motor A connected between A01 and A02
#define MOTORA_IN2 8          //
#define STANDBY     10        // standby pin

static uint8_t enc_prev_pos = 0;
static uint8_t enc_flags    = 0;

int i=0;
int x=0;

float prevDeg = 0;


/**
 * Sets up the pin modes and encoder values
 */

void setup()
{
  Serial.begin(9600);
  // set pins as input with internal pull-up resistors enabled
  pinMode(PIN_ENCODER_A, INPUT);
  pinMode(PIN_ENCODER_B, INPUT);
  digitalWrite(PIN_ENCODER_A, HIGH);
  digitalWrite(PIN_ENCODER_B, HIGH);

  pinMode(STANDBY, OUTPUT);

  pinMode(PWM_A, OUTPUT);
  pinMode(MOTORA_IN1, OUTPUT);
  pinMode(MOTORA_IN2, OUTPUT);
  
  // get an initial reading on the encoder pins
  if (digitalRead(PIN_ENCODER_A) == LOW) {
    enc_prev_pos |= (1 << 0);
  }
  if (digitalRead(PIN_ENCODER_B) == LOW) {
    enc_prev_pos |= (1 << 1);
  }
  
  Serial.println("Initialization complete.....");
}

/**
 * gets encoder values 
 * @return Returns current encoder value  
 */
int getEncoder()
{  
  int8_t enc_action = 0; // 1 or -1 if moved, sign is direction
  int i;

  // note: for better performance, the code will now use
  // direct port access techniques
  // http://www.arduino.cc/en/Reference/PortManipulation
  uint8_t enc_cur_pos = 0;
  // read in the encoder state first
  if (bit_is_clear(PINx, PIN_ENCODER_A)) {
    enc_cur_pos |= (1 << 0);
  }
  if (bit_is_clear(PINx, PIN_ENCODER_B)) {
    enc_cur_pos |= (1 << 1);
  }

  // if any rotation at all
  if (enc_cur_pos != enc_prev_pos)
  {
    if (enc_prev_pos == 0x00)
    {
      // this is the first edge
      if (enc_cur_pos == 0x01) {
        enc_flags |= (1 << 0);
      }
      else if (enc_cur_pos == 0x02) {
        enc_flags |= (1 << 1);
      }
    }

    if (enc_cur_pos == 0x03)
    {
      // this is when the encoder is in the middle of a "step"
      enc_flags |= (1 << 4);
    }
    else if (enc_cur_pos == 0x00)
    {
      // this is the final edge
      if (enc_prev_pos == 0x02) {
        enc_flags |= (1 << 2);
      }
      else if (enc_prev_pos == 0x01) {
        enc_flags |= (1 << 3);
      }

      // check the first and last edge
      // or maybe one edge is missing, if missing then require the middle state
      // this will reject bounces and false movements
      if (bit_is_set(enc_flags, 0) && (bit_is_set(enc_flags, 2) || bit_is_set(enc_flags, 4))) {
        enc_action = 1;
      }
      else if (bit_is_set(enc_flags, 2) && (bit_is_set(enc_flags, 0) || bit_is_set(enc_flags, 4))) {
        enc_action = 1;
      }
      else if (bit_is_set(enc_flags, 1) && (bit_is_set(enc_flags, 3) || bit_is_set(enc_flags, 4))) {
        enc_action = -1;
      }
      else if (bit_is_set(enc_flags, 3) && (bit_is_set(enc_flags, 1) || bit_is_set(enc_flags, 4))) {
        enc_action = -1;
      }

      enc_flags = 0; // reset for next time
    }
  }

  enc_prev_pos = enc_cur_pos;

  if (enc_action > 0) {
    return i=i+1;
  }
  else if (enc_action < 0) {
    return i=i-1;
  }
  else {
    return i=0;
  }
}

/**
 * Calculates the output angle based on global variable set gear ratio
 *
 * @return Returns the calculated degree (i* GEAR_RATIO)
 */
float outputAngleCalc()
{
  float degree;
  degree = i * GEAR_RATIO;
  return degree;
}
/**
 * Sets motor speed
 */
void move(int motor, int speed, int direction)
{
//Move specific motor at speed and direction
//motor: 0 for B 1 for A
//speed: 0 is off, and 255 is full speed
//direction: 0 clockwise, 1 counter-clockwise

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
    analogWrite(PWM_A, speed);
  }
}

/**
    stops motor
*/
void stop(){
//enable standby  
  digitalWrite(STANDBY, LOW); 
}


void position(float requestedAngle)
{
  //convert requested position to angle
  float error;
  float approxAngle = (requestedAngle/GEAR_RATIO)+error;
  int pulses = floor(approxAngle);
  float newError = approxAngle - pulses;
  error = newError;
}


void loop()
{
  float curDeg;

  getEncoder();
  curDeg = outputAngleCalc();
  if (curDeg != prevDeg)
  {
    Serial.println(curDeg,4);
    prevDeg = curDeg;
  }

  move(1, 255, 1); //motor 1, full speed, left

  delay(5000); //go for 1 second
  stop(); //stop
  delay(500); //hold for 250ms until move again

  move(1, 127, 0); //motor 1, half speed, right

  delay(5000);
  stop();
  delay(500);


}