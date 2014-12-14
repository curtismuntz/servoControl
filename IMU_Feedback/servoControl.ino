/*
  Production Servo code.

  This is the most up to date code.

  sources:
    * jeff rowberg's I2Cdev.h and MPU6050.h header files borrowed from his github
    * MIT filters.pdf
    * MPU6050 PS and PM manuals/datasheets

  NOTE: this file requires the I2Cdev, MPU6050, Adafruit_LEDBackpack, 
  and Adafruit_GFX.h libraries either installed to the arduino libraries 
  directory, or in the path of the servoControl.ino file.

  TODO: come up with better gimbal name.


  WIRING NOTES:
    MPU VCC pin ---3.3v
    MPU GND pin ---GND
    MPU SCL pin ---A5
    MPU SDA pin ---A4
    MPU ADO pin ---GND
*/
 
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"


#define LED_PIN 13

#define LEAST_SENSITIVE_GYRO MPU6050_GYRO_FS_2000
#define LEAST_SENSITIVE_ACCEL MPU6050_ACCEL_FS_16
#define M_PI 3.14159265359

#define GEAR_RATIO 0.04       // 250:1
#define PWM_A 3               // Speed control 
#define MOTORA_IN1 9          // motor A connected between A01 and A02
#define MOTORA_IN2 8          // not used
#define STANDBY     10        // standby pin



Adafruit_7segment matrix = Adafruit_7segment();
MPU6050 mpu; 

bool blinkState=false;
bool debug = false; // set to true for testing/debugging.
int motorVolume =0; // <-- sash's constant

// these variables hold sensor data
int16_t ay, ax, az;
int16_t gy, gx, gz;

//these are angles written to servos
float yAngle = 0.00;
float yOutput = 0.00;

//calibrator values (set by initial function)
int yGyro_calibrate;
int yAccel_calibrate;

//gyro variables
float yGyro_raw;
float yGyro_rate = 0.00;
float yGyro_angle;
float yGyro_tst;
float yGyro_scale = 0.13;

//accelerometer variables
float yAccel_raw;
float yAccel_math;
float yAccel_angle; 
int yAccel_scale;

//complementary filter scaling
double gyro_weight = 0.95; //high pass
double accel_weight = 0.05; //low pass

// timing parameters
int last_update;
int cycle_time;
long last_cycle = 0;
int timerVal = 50; //unused 


// PID function variables
float desired = 0.00;

float yErrorOld = 0;
float yDer = 0;
float yInt = 0;
float yError = 0;

// PID constants . these should be computed in the calibrate function.
double Kp = 0;
double Ki = 0;
double Kd = 0;


/**************************************************************
FUNCTION: setup()
  initializes i2c bus
  initializes seven seg matrix
  assigns pinmodes for motor drive
  set sensitivities and offsets
  calls calibration function for more precise baselines
**************************************************************/
void setup()
{
  Wire.begin();      // join I2C bus  
  Serial.begin(9600);    //  initialize serial communication

  pinMode(STANDBY, OUTPUT);

  pinMode(PWM_A, OUTPUT);
  pinMode(MOTORA_IN1, OUTPUT);
  pinMode(MOTORA_IN2, OUTPUT);

  matrix.begin(0x70);
  matrix.println(0);
  matrix.writeDisplay();


  if(debug == true){
    Serial.println("Initializing I2C devices...");
    Serial.println("Testing device connections...");
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    Serial.println("\n\n\nUpdating internal sensor offsets...\n");
    timerVal=120;

  }

  pinMode(LED_PIN, OUTPUT);  // configure LED pin
  offsetter();
  calibrate();


  //initialize some PID variables
  float xErrorOld = 0.0;
  float yErrorOld = 0.0;
  float xInt = 0.0;
  float yInt = 0.0;

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
FUNCTION: offsetter
  sets the precalculated offsets on the MEMS chip for more accurate
  gyro and accelerometer readings
***************************************************************/
void offsetter()
{
  mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
  mpu.setFullScaleGyroRange(LEAST_SENSITIVE_GYRO);
  mpu.setFullScaleAccelRange(LEAST_SENSITIVE_ACCEL);
  mpu.setXGyroOffset(40);
  mpu.setYGyroOffset(20);
  mpu.setSleepEnabled(false);
  mpu.setXAccelOffset(-1250);
  mpu.setYAccelOffset(275);
}

/***************************************************************
FUNCTION: calibrate
  gets 100 samples of gyro and accel data to further decrease 
  offset sensitivity

  //TODO: MAKE THIS FUNCTION AMAZING
***************************************************************/
void calibrate()
{
  int xGyro_avg, yGyro_avg;
  int xAccel_avg, yAccel_avg;
  long sample_size = 1000;
  // setup loop to read gyro 10 times
  for (int i = 0; i < sample_size; i++)
  {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    // read gyro and each time, add the value to the running total
    yGyro_avg = yGyro_avg + gy;
    yAccel_avg = yAccel_avg + ax; 
  }
  // with a sum of 10 readings, divide by 10 to get the average
  yGyro_calibrate = yGyro_avg / sample_size;
  yAccel_calibrate = yAccel_avg / sample_size;
  Serial.print("\tyGyro_calibrate:");
  Serial.print(yGyro_calibrate);
  Serial.print("\tyAccel_calibrate:");
  Serial.println(yAccel_calibrate);

   Kp=6.5991;
   Ki=0.67604;
   Kd=0.023425;

}


/***************************************************************
FUNCTION: sampleGyro()
  samples gyro data, storing data in global variables
***************************************************************/
void sampleGyro()
{
  mpu.getRotation(&gx, &gy, &gz);

  yGyro_raw = (gy - yGyro_calibrate) * -1;
  yGyro_rate = (yGyro_raw * yGyro_scale) * (cycle_time*.001);
  yGyro_tst = yGyro_tst + yGyro_rate;
  yGyro_angle = yAngle + yGyro_rate;

}


/***************************************************************
FUNCTION sampleAccel()
  samples accelerometer data, storing data in global variables
  TODO: update to use arctan2f instead of divide by thirty hack
***************************************************************/
void sampleAccel()
{

  mpu.getAcceleration(&ax, &ay, &az);
  // method one: arctan!
    //yAccel_raw = ax - yAccel_calibrate;
    //yAccel_math = atan2f(ax, (sqrt((ay^2+az^2))));
    //yAccel_angle = (float)(yAccel_math * yAccel_scale);
  //method two:
    //yAccel_raw = (float)(map((ax), -2000, 2000, -90, 90)*1 -yAccel_calibrate);
    //yAccel_angle = yAccel_raw;
  //method three
      //yAccel_raw = ax - yAccel_calibrate;
      //yAccel_angle = (float) yAccel_raw;
  //method4 experimenting
    yAccel_raw = ax/30;// - yAccel_calibrate;
    yAccel_angle = (float) yAccel_raw;
}

/***************************************************************
FUNCTION filter()
  computes the highpass/lowpass complementary filter for output angle
***************************************************************/
void filter()
{
  yAngle = (float)((gyro_weight * yGyro_angle) + (accel_weight * yAccel_angle));
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
void compute()
{
  yError = desired - yAngle;
  //integral
  yInt = yInt + (yError * (cycle_time*.001));
  //derivative
  yDer = (yError - yErrorOld)/(cycle_time*.001);
  yOutput = (Kp*yError) + (Ki*yInt) + (Kd*yDer);

  yErrorOld = yError;

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
FUNCTION move(int motor, int, int)
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
  // ***** if forced timeimg is required uncomment the while code.
  
  // while((millis() - last_cycle) < timerVal){
  //  delay(1);
  // }

  cycle_time = millis() - last_cycle;
  last_cycle = millis();
}


/**************************************************************
FUNCTION: printCSV
  function prints data to serial monitor in hopes to save to
  csv file for use in plotting. note the xGyro_tst and yGyro_tst
  are NOT the "gyro angle values". they are there so that drift 
  can be seen
**************************************************************/

void printCSV()
{
  Serial.print(yGyro_tst);
  Serial.print(", ");
  Serial.print(yAccel_angle);
  Serial.print(", ");
  Serial.print(yAngle);
  Serial.print(", ");
  Serial.println(yOutput);
}

/**************************************************************
FUNCTION: graphME

specifically formated serial outputs for realtime plots using python
**************************************************************/
void graphME(char axis, char option)
{
 
  if(axis == 'y')
  {
    if(option == 'o')
    {
      //reminder: this plots to outputPlotter.py
      Serial.print(yAngle);
      Serial.print(" ");
      Serial.println(yOutput);
    }
    if(option == 'v')
    {
      //reminder: this plots to plotter.py
      Serial.print(yGyro_tst);
      Serial.print(" ");
      Serial.print(yAccel_angle);
      Serial.print(" ");
      Serial.println(yAngle);
    }
    if(option == 't') //'t' is tune
    {
      //reminder: this plots to PIDplot.py
      Serial.print(yAngle);
      Serial.print(" ");
      Serial.print(yOutput);
      Serial.print(" ");
      Serial.print(Kp);
      Serial.print(" ");
      Serial.print(Ki);
      Serial.print(" ");
      Serial.println(Kd);

    }

  }

}

/**************************************************************
FUNCTION: matrixPrint()

prints feedback sensor reading to Seven Segment Display 
**************************************************************/

void matrixPrint()
{
  int toDisp = (int) yAngle;
  matrix.println(toDisp);
  matrix.writeDisplay();
}


/**************************************************************
FUNCTION: outputCalc
returns the constrained pwm value
**************************************************************/
int outputCalc()
{
  int pwmval;
  return pwmval = constrain(abs((int)yOutput),0,255);
}


/**************************************************************
FUNCTION: dir
returns the direction for the motor to turn
**************************************************************/
int dir()
{
    int directionz
    if (yOutput < 0)
    {
      directionz=1;
    }
    else
    {
      directionz = 0;
    }
    return directionz
}

/***************************************************************
Function: MAIN LOOP
  performs data gathering, computing, and executing. uncomment
  printDEBUG or printCSV for debugging/extracting data.

  graphME(axis, option) is a function to assist in real time 
  graphing.


***************************************************************/
void loop()
{
  // sample data and filter into angle
  sampleGyro();
  sampleAccel();
  filter();
  int dir = 0;
  Serial.println("Type an angle (-30 to 30) into the box above,");
  Serial.println("then click [send] or press [return]");
  Serial.println();  // Print a blank line


// Input an angle via serial monitor to turn the motor to said angle
while(true)
  {
    sampleGyro();
    sampleAccel();
    filter();
    while (Serial.available() > 0)
    {
      // last change: -1* Serial.parseInt();
      desired = 1* Serial.parseInt();
    }

    //compute PID, execute servo PWM write
    compute();

    int go;
    go = outputCalc();
    dir= outputDir();

    if (go < 2)
    {
      go = 0;
    }
    move(1, go, dir);


    // Graphing/Plotting/Data Acquisition
    //graphME('y','v');
    
    matrixPrint();
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    timing();
  }
}