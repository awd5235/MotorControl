/*=========================================== MOTOR TEST ===========================================
  Using three switches, select up to three motors to control with a potentiometer and display 
  the current position (in percent 0-99) on a 7 segment display. Sound a buzzer when motors are 
  too close to max position, indicating that they might be strained.
  
  Use "switch...case" statements to implement system as a finite state maschine.
  Consider Switch 1 to be the LSB and Switch 3 to be the MSB
  Switches = 0x0000, case 0: no motors are selected, potentiometer has no effect, screen displays "00"
  Switches = 0x0001, case 1: Small servo is activated, deactivate other motors, move to position of potentiometer and display on 7 segment
  Switches = 0x0010, case 2: Tall servo is activated, deactivate other motors, move to position of potentiometer and display on 7 segment
  Switches = 0x0011, case 3: Both Small and Tall servos are activated, deactivate other motors, move to position of potentiometer and display on 7 segment



  ------------------------------------------ About PORTD -------------------------------------------
  Port manipulation allows for the simultaneous manipulation of digital pins. It is faster, but less readable and less transferrable.
  Port D controls pins 0-7 with LSB corresponding to pin 0 and MSB corresponding to pin 7.
  
  Controlled by 3 registers:
  -DDRD:  (R/W) Data direction register D which specifies whether the pins are INPUT (default, hi-z) or OUTPUT (low-z). Always set this reg first!
          Write 0 to set as input, write 1 to set as output
          The low level equivalent of pinMode()
  -PORTD: (R/W) Uses internal pull-ups to set the logic state of pins HIGH or LOW. 
          When configured OUTPUT, think of it as setting state HIGH/LOW. When configured INPUT think of it as setting the pull-up.
          Write 0 to set LOW/disable pull-up, write 1 to set HIGH/enable pull-up.
          The low level equivalent of digitalWrite(), see also bitWrite()
  -PIND:  (R) Reads the state of input pins
          The low level equivalent of digitalRead()
  
  NOTE: PORT D Digital pins 0 and 1 are used for serial comm and programming Arduino. Avoid using these!!!
        For digital pins 8-13 see PORT B, bits 6 and 7 map to the crystal and are unusable. 
        For analog pins 0-5 see PORT C, bits 6 and 7 are only accessible on Arduino mini.
  --------------------------------------------------------------------------------------------------


  
  ---------------------------------------- About Servos --------------------------------------------
  Standard hobbyist servos achieve approximately 180 degrees of motion unless specifically stated.
  They typically expect to see a pulse every 20ms (50Hz period) for historical reasons.
  However modern servos are not defined by the PWM duty cycle, but instead the width of the pulse
  As long as the refresh rate is somewhere between 40Hz-200Hz, the servo should be able to detect, say, a 1ms pulse the same.
  (See Wikipedia: Servo Control)
    1ms pulse -> 0 degrees
    1.5ms pulse -> 90 degrees
    2ms pulse -> 180 degrees
  --------------------------------------------------------------------------------------------------



  ------------------------------------- About Motor Shield V1.2-------------------------------------
  "SER1" header -> PWM1B -> Controlled by Uno Pin 10
  "SERVO_2" header -> PWM1A -> Controlled Uno Pin 9
  --------------------------------------------------------------------------------------------------



  -------------------------------- Non-blocking Polling Algorithm ----------------------------------
  Delay() is a blocking process which doesn't allow for any other processing while it waits.
  Instead, poll output every x ms and in between, do whatever other processing you need.
  1. Get the current time in ms
  2. Subtract the time it was last polled to get the elapsed time
  3. Is the elapsed time >= the polling interval?
  4. If so, then poll output
  5. Otherwise, skip it and do something else.
  --------------------------------------------------------------------------------------------------
 
====================================================================================================*/

#include <Servo.h>
//#include <Stepper.h>
//#include <AFMotor.h>
//#include <AccelStepper.h>

// Hardware Peripherals
Servo small_servo;  // Declare the smaller servo 
Servo tall_servo;   // Declare the larger servo
const int POT = 5;  // Map the potentiometer to Uno Analog pin 5

// Declare Intermediate Constants
const long pollingInterval = 500;   // The polling interval in ms

// Declare Intermediate Variables
int potPos = 0;                     // Stores the position of the potentiometer
unsigned long currentTime = 0;      // Stores the current time in ms
unsigned long timeOfLastPoll = 0;   // Stores the time since the input was polled in ms
unsigned long elapsedTime = 0;      // Stores how much time has passed since the output was last polled
byte FSMstate = 0;                  // Stores the state of the finite state machine, only need a byte so no need for a two byte int


void setup() {

  // Servo Setup
  small_servo.attach(10, 1000, 2000);  // Attach to pin 10 to small servo, set min angle to 0 (1ms), max angle to 180 (2ms)
  small_servo.write(0);                // Alternatively "small_servo.writeMicroseconds(1000);" to initialize at 0 degrees
  tall_servo.attach(9, 1000, 2000);    // Attach to pin 9, set min angle to 0 (1ms), max angle to 180 (2ms)
  tall_servo.write(0);                 // Alternatively "tall_servo.writeMicroseconds(1000);" to initialize at 0 degrees

  // Switch setup
  DDRD &= B00000011;                   // Set digital pins 2-7 as inputs, leave pins 0 and 1 alone since these are used for programming the arduino
  PORTD |= B00001100;                  // Set pull-ups on digital pins 2 and 3 leaving the rest of the pins alone.
  // NOTE active low logic. Switch open: pins 2 and 3 are set. Switch closed: pins 2 and 3 cleared.

  //Serial.begin(9600);                  // For testing purposes
}


void loop(){

  // Change state based on switch inputs
  //  1. Read Port D Pins (PIND)
  //  2. Invert them for active high logic (~)
  //  3. Single out digital pins 2 and 3 which are connected to the hardware switches and ignore others as they contain nothing important. (& B00001100)
  //  4. Shift right twice to place the bits we care about in the LSB position (>> 2)
  //  5. FSMstate now holds an integer value between 0 and 3
  FSMstate = (~PIND & B00001100) >> 2;
  
  // Switch to different cases based on the value of the current state (FSMstate)
  // FSMstate = 0, case 0: no motors are selected, potentiometer has no effect, screen displays "00"
  // FSMstate = 1, case 1: Small servo is activated, deactivate other motors, move to position of potentiometer and display on 7 segment
  // FSMstate = 2, case 2: Tall servo is activated, deactivate other motors, move to position of potentiometer and display on 7 segment
  // FSMstate = 3, case 3: Both Small and Tall servos are activated, deactivate other motors, move to position of potentiometer and display on 7 segment
  switch (FSMstate)
  {
    case 0:  // No motors, no need to calculate elapsed time or read potentiometer position
      
      break;
      
    case 1:  // Small Servo only
      currentTime = millis();                       // Get the current time since the program started in ms
      elapsedTime = currentTime - timeOfLastPoll;   // Calculate elapsed time
      if(elapsedTime >= pollingInterval)            // Is it time to poll the output?
      {                                             // If so:
        timeOfLastPoll = currentTime;               //    1. Save the time of last poll for later reference
        potPos = analogRead(POT);                   //    2. read the value of the potentiometer (value between 0 and 1023)
        potPos = map(potPos, 0, 1023, 0, 180);      //    3. scale it to use it with the servo (value between 0 and 180)
        small_servo.write(potPos);                  //    4. sets the small servo position according to the scaled value
      }      
      break;
      
    case 2:  // Tall Servo only
      currentTime = millis();                       // Get the current time since the program started in ms
      elapsedTime = currentTime - timeOfLastPoll;   // Calculate elapsed time
      if(elapsedTime >= pollingInterval)            // Is it time to poll the output?
      {                                             // If so:
        timeOfLastPoll = currentTime;               //    1. Save the time of last poll for later reference
        potPos = analogRead(POT);                   //    2. read the value of the potentiometer (value between 0 and 1023)
        potPos = map(potPos, 0, 1023, 0, 180);      //    3. scale it to use it with the servo (value between 0 and 180)
        tall_servo.write(potPos);                   //    4. sets the tall servo position according to the scaled value    
      }     
      break;
      
    case 3:  // Both servos
      currentTime = millis();                       // Get the current time since the program started in ms
      elapsedTime = currentTime - timeOfLastPoll;   // Calculate elapsed time
      if(elapsedTime >= pollingInterval)            // Is it time to poll the output?
      {                                             // If so:
        timeOfLastPoll = currentTime;               //    1. Save the time of last poll for later reference
        potPos = analogRead(POT);                   //    2. read the value of the potentiometer (value between 0 and 1023)
        potPos = map(potPos, 0, 1023, 0, 180);      //    3. scale it to use it with the servo (value between 0 and 180)
        small_servo.write(potPos);                  //    4. sets the small servo position according to the scaled value
        tall_servo.write(potPos);                   //    5. sets the tall servo position according to the scaled value    
      }
      break;
  }
}
