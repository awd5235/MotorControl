/*=================== MOTOR TEST ==================

  ------------------ About Servos -----------------
  Standard hobbyist servos achieve approximately 180 degrees of motion unless specifically stated.
  They typically expect to see a pulse every 20ms (50Hz period) for historical reasons.
  However modern servos are not defined by the PWM duty cycle, but instead the width of the pulse
  As long as the refresh rate is somewhere between 40Hz-200Hz, the servo should be able to detect, say, a 1ms pulse the same.
  (See Wikipedia: Servo Control)
    1ms pulse -> 0 degrees
    1.5ms pulse -> 90 degrees
    2ms pulse -> 180 degrees
  For now assume both servos follow standard hobbyist PWM control signals.
  -------------------------------------------------


  ------------ About Motor Shield -----------------
  "SER1" header -> PWM1B -> Controlled by Uno Pin 10
  "SERVO_2" header -> PWM1A -> Controlled Uno Pin 9
  -------------------------------------------------


  ------- Non-blocking Polling Algorithm ----------
  Delay() is a blocking process which doesn't allow for any other processing while it waits.
  Instead, poll output every x ms and in between, do whatever other processing you need.
  1. Get the current time in ms
  2. Subtract the time it was last polled to get the elapsed time
  3. Is the elapsed time >= the polling interval?
  4. If so, then poll output
  5. Otherwise, skip it and do something else.
  
  -------------------------------------------------
 
====================================================*/

#include <Servo.h>
//#include <Stepper.h>
//#include <AFMotor.h>
//#include <AccelStepper.h>

// Declare Hardware Peripherals
Servo small_servo;  // Declare the smaller servo 
Servo tall_servo;   // Declare the larger servo
const int POT = 5;  // Map the potentiometer to Uno Analog pin 5
const int SW1 = 2;  // Switch connected to digital pin 2
const int SW2 = 3;  // Switch connected to digital pin 3

// Declare Intermediate Constants
const long pollingInterval = 500;   // The polling interval in ms

// Declare Intermediate Variables
int potPos = 0;                     // Stores the position of the potentiometer
bool sw1State = LOW;                // Stores the state of SW1
bool sw2State = LOW;                // Stores the state of SW2
unsigned long timeOfLastPoll = 0;   // Time in ms, since the input was polled
unsigned long elapsedTime = 0;      // How much time has passed since the output was last polled?
int state = 0;                      // Stores the state of the finite state machine

void setup() {
  // Map MCU PWM to the corresponding motor
  small_servo.attach(10, 1000, 2000);  // Attach to pin 10, set min angle to 0 (1ms), max angle to 180 (2ms)
  tall_servo.attach(9, 1000, 2000);    // Attach to pin 9, set min angle to 0 (1ms), max angle to 180 (2ms)

  // Initialize motors to 0 degrees as a nice neat starting point
  small_servo.write(0);    // Alternatively "small_servo.writeMicroseconds(1000);"
  tall_servo.write(0);     // Alternatively "tall_servo.writeMicroseconds(1000);" 

  // Initialize switch pins
  pinMode(SW1, INPUT_PULLUP);     // Switch 1 uses built in pullups. Note active low logic: Open = 1, Closed = 0.
  pinMode(SW2, INPUT_PULLUP);     // Switch 2 uses built in pullups. Note active low logic: Open = 1, Closed = 0.

  //Serial.begin(9600); // For testing purposes
}


void loop(){
  
  sw1State = !digitalRead(SW1);               // Read Switch 1 and invert the logic to active high so that Open = 0, Closed = 1
  sw2State = !digitalRead(SW2);               // Read Switch 2 and invert the logic to active high so that Open = 0, Closed = 1
  unsigned long currentTime = millis();       // Get the current time since the program started in ms
  elapsedTime = currentTime - timeOfLastPoll; // Calculate elapsed time
  
  /*switch (state)
  {
    case 0:
    
      break;
      
    case 1:
    
      break;
      
    case 2:
    
      break;
      
    case 3:
    
      break;
  }

  if(elapsedTime >= pollingInterval)  // Is it time to poll the output?
    {
      timeOfLastPoll = currentTime;             //    Save the time of last poll for later reference
      potPos = analogRead(POT);                 //    read the value of the potentiometer (value between 0 and 1023)
      potPos = map(potPos, 0, 1023, 0, 180);    //    scale it to use it with the servo (value between 0 and 180)
      small_servo.write(potPos);                //    sets the servo position according to the scaled value
      tall_servo.write(potPos);                 //    sets the servo position according to the scaled value    
    }

  */
}
