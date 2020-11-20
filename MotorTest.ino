/*=========================================== MOTOR TEST ===========================================
  Using three switches, select up to three motors to control with a potentiometer and display 
  the current position (in percent 0-99) on a 7 segment display. Sound a buzzer when motors are 
  too close to max position, indicating that they might be strained.
  
  Use "switch...case" statements to implement system as a finite state maschine.
  Consider Switch 1 to be the LSB and Switch 2 to be the MSB
  Switches = 0x0000, case 0: no motors are selected, potentiometer has no effect, screen displays "00"
  Switches = 0x0001, case 1: Small servo is activated, deactivate other motors, move to position of potentiometer and display
  Switches = 0x0010, case 2: Tall servo is activated, deactivate other motors, move to position of potentiometer and display
  Switches = 0x0011, case 3: Both Small and Tall servos are activated, deactivate other motors, move to position of potentiometer and display



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

  
  ------------------------------------------- About LCD --------------------------------------------
  2x16 Character LCD.
  
  For this project configure LCD in 4-bit mode using Port D
    Digital Pin 2 = Register Select  : 0 = command, 1 = data
    Digital Pin 3 = Enable           : Command/Data sent on trailing edge. Keep low, Set, then clear to write. 
    Digital Pin 4 = D4               : LSB
    Digital Pin 5 = D5               :
    Digital Pin 6 = D6               :
    Digital Pin 7 = D7               : MSB
  
  LCD COMMANDS
    "Function Set"        : 0x001WLDXX, where W = width (4 vs 8 bit), L = lines ( 1 vs 2), and D = dots (5x7 vs 5x10). X = don't care
    "Display Control"     : 0x00001DCB, where D = Display (off vs on), C = cursor (off vs on), B = blink (off vs on)
    "Display clear"       : 0x00000001, Clears the display and returns the cursor to address 0. No levers associated with this command
    "Cursor Home"         : 0x0000001X, where X = don't care. Returns cursor home and shifted display to original position
    "Entry mode"          : 0x000001CS, where C = cursor move (decrement vs increment), S = shift display (off vs on) 
    "Cursor Control"      : 0x0001MDXX, where M = move (cursor move vs display move), D = direction (left vs right)
  --------------------------------------------------------------------------------------------------
  
  ------------------------------------------- TO DO ------------------------------------------------
  2. Port manipulation of POT (BONUS: replace pot with light sensor)
  3. Port manipulation of Servos
  4. Add alarm for 180 degrees
  5. Implement using interrupts
  6. Add stepper?
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
String line1;
String line2;

void setup() {

  //Serial.begin(9600);                 // For testing purposes

  // Servo Setup
  small_servo.attach(10, 1000, 2000);   // Attach to pin 10 to small servo, set min angle to 0 (1ms), max angle to 180 (2ms)
  small_servo.write(0);                 // Alternatively "small_servo.writeMicroseconds(1000);" to initialize at 0 degrees
  tall_servo.attach(9, 1000, 2000);     // Attach to pin 9, set min angle to 0 (1ms), max angle to 180 (2ms)
  tall_servo.write(0);                  // Alternatively "tall_servo.writeMicroseconds(1000);" to initialize at 0 degrees

  // Switch setup
  DDRB &= B11100111;                    // Set digital pins 11 and 12 as inputs for the hardware switches, leave others unchanged as they are set elsewhere
  PORTB |= B00011000;                   // Set pull-ups on digital pins 11 and 12 leaving the rest of the pins unchanged.
  // NOTE active low logic. Switch open: pins 11 and 12 are set. Switch closed: pins 11 and 12 cleared.

  // LCD Setup
  DDRD |= B11111100;                    // Initialize digital pins 7-2 as outputs for the LCD screen, leave 1,0 unchanged as they are used for serial/programming
  _delay_ms(100);                           // Wait >40ms for Led Vcc to rise to the correct voltage
  LCDinit4bit();                        // Initialize LCD in 4-bit mode
}


void loop(){
  // Change state based on switch inputs
  //  1. Read Port B Pins (PINB)
  //  2. Invert them for active high logic (~)
  //  3. Single out digital pins 11 and 12 which are connected to the hardware switches and ignore others as they are not important now. (& B00011000)
  //  4. Shift right three times to place the bits we care about in the LSB position (>> 3)
  //  5. FSMstate now holds an integer value between 0 and 3
  FSMstate = (~PINB & B00011000) >> 3;
  
  // Switch to different cases based on the value of the current state (FSMstate)
  // FSMstate = 0, case 0: no motors are selected, potentiometer has no effect, screen displays "--"
  // FSMstate = 1, case 1: Small servo is activated, deactivate other motors, move to position of potentiometer and display angle
  // FSMstate = 2, case 2: Tall servo is activated, deactivate other motors, move to position of potentiometer and display angle
  // FSMstate = 3, case 3: Both Small and Tall servos are activated, deactivate other motors, move to position of potentiometer and display angle
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

/*
void LCDwriteString(char message[])
{
  LCDwriteCMD(0x01);          // "Display clear" command (0x01) : Clears the display and returns the cursor to address 0
  LCDwriteDAT(message[0]);
}
*/

void LCDprint(String line)
{
  for(int i=0; line[i] != '\0'; i++)
  {
    LCDwriteDAT(line[i]);
  }
 
 LCDwriteCMD(0x02); 
}

// LCD COMMANDS
//    "Function Set"        : 0x001WLDXX, where W = width (4 vs 8 bit), L = lines ( 1 vs 2), and D = dots (5x7 vs 5x10). X = don't care
//    "Display Control"     : 0x00001DCB, where D = Display (off vs on), C = cursor (off vs on), B = blink (off vs on)
//    "Display clear"       : 0x00000001, Clears the display and returns the cursor to address 0. No levers associated with this command
//    "Cursor Home"         : 0x0000001X, where X = don't care. Returns cursor home and shifted display to original position
//    "Entry mode"          : 0x000001CS, where C = cursor move (decrement vs increment), S = shift display (off vs on) 
//    "Cursor Control"      : 0x0001MDXX, where M = move (cursor move vs display move), D = direction (left vs right)
    
void LCDinit4bit()      // Initialize the LCD screen in 4-bit mode
{
  LCDreset();           // 1. Reset LCD into 4-bit mode
  LCDwriteCMD(0x28);    // 2. Configure "Function Set" Register (0x28)  : 4 bit mode, 2 lines, and 5x7 dots
  LCDwriteCMD(0x08);    // 3. Turn off all "Display Control" (0x08)     : Display, cursor, and blink all off
  LCDwriteCMD(0x01);    // 4. "Display clear" command (0x01)            : Clears the display and returns the cursor to address 0
  LCDwriteCMD(0x06);    // 5. Configure "Entry Mode" Register (0x06)    : Sets auto increment cursor and disables display shift
  LCDwriteCMD(0x0F);    // 6. Configure "Display Control" Register      : Enable screen, cursor, and blink
}

void LCDreset()             // Reset LCD into 4-bit mode. Only needs to be called once at start up. 
{ 
  // Data sheet calls this initialization by instruction. Send 0x3 three times, then 0x2 to reset into 4-bit mode.
  PORTD &= B00000011;       // 01. Clear D7-D4, En, RS (digital pins 7-2, respectively) to 0 as a neutral starting point, leave pins 1,0 unchanged
  PORTD |= B00111000;       // 02. Write upper nibble 0x3, En=1
  PORTD &= B11110111;       // 03. En=0. Command is sent on trailing edge of enable
  _delay_ms(5);             // 04. Wait >4.1ms for command to process
  PORTD |= B00111000;       // 05. Write upper nibble 0x3, En=1
  PORTD &= B11110111;       // 06. En=0. Command is sent on trailing edge of enable
  _delay_us(150);           // 07. Wait >100us for command to process
  PORTD |= B00111000;       // 08. Write upper nibble 0x3, En = 1
  PORTD &= B11110111;       // 09. En=0. Command is sent on trailing edge of enable
  _delay_us(150);           // 10. Wait >100us for command to process
  PORTD &= B00000011;       // 11. Clear PORTD to overwrite next instruction in buffer
  PORTD |= B00101000;       // 12. Write upper nibble 0x2, En = 1
  PORTD &= B11110111;       // 13. En=0. Command is sent on trailing edge of enable
  _delay_us(150);           // 14. Wait >100us for command to process
  // At this point LCD is reset and listening for 4-bit commands.
}

void LCDwriteCMD(byte CMD)                // Write a command byte to the LCD one nibble at a time using 4 bit mode.
{
  byte upperNibble = CMD & 0xF0;          // 01. Mask upper nibble of CMD, clear lower nibble so result takes the form: 0xUUUU0000
  byte lowerNibble = (CMD & 0x0F) << 4;   // 02. Mask lower nibble of CMD, clear upper nibble then shift left 4 so result takes the form 0xLLLL0000
  PORTD &= B00000011;                     // 03. Clear LCD buffer, enable, and register select
  PORTD |= upperNibble;                   // 04. Write upperNibble to LCD buffer
  PORTD |= B00001000;                     // 05. En=1, RS = 0
  PORTD &= B11110111;                     // 06. En=0. Command is sent on trailing edge of enable
  _delay_us(1600);                        // 07. Wait >40us for command to process
  PORTD &= B00000011;                     // 08. Clear LCD buffer, enable, and register select
  PORTD |= lowerNibble;                   // 09. Write lowerNibble to LCD buffer
  PORTD |= B00001000;                     // 10. En=1, RS = 0
  PORTD &= B11110111;                     // 11. En=0. Command is sent on trailing edge of enable
  _delay_us(1600);                        // 12. Wait >40us for command to process
}


void LCDwriteDAT(byte DAT)                // Write a data byte to the LCD one nibble at a time using 4 bit mode.
{
  byte upperNibble = DAT & 0xF0;          // 01. Mask upper nibble of DAT, clear lower nibble so result takes the form: 0xUUUU0000
  byte lowerNibble = (DAT & 0x0F) << 4;   // 02. Mask lower nibble of DAT, clear upper nibble then shift left 4 so result takes the form 0xLLLL0000
  PORTD &= B00000011;                     // 03. Clear LCD buffer, enable, and register select 
  PORTD |= upperNibble;                   // 04. Write upperNibble to LCD buffer
  PORTD |= B00001100;                     // 05. En=1, RS = 1
  PORTD &= B11110111;                     // 06. En=0. Command is sent on trailing edge of enable
  _delay_us(1600);                        // 07. Wait >40us for command to process
  PORTD &= B00000011;                     // 08. Clear LCD buffer, enable, and register select
  PORTD |= lowerNibble;                   // 09. Write lowerNibble to LCD buffer
  PORTD |= B00001100;                     // 10. En=1, RS = 1
  PORTD &= B11110111;                     // 11. En=0. Command is sent on trailing edge of enable
  _delay_us(1600);                        // 12. Wait >40us for command to process
}
