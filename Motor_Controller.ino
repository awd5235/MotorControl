/*==================================== Motor Controller ==========================================
  Using two DIP switches, select up to two servo motors to control with a potentiometer and display 
  the currently active motor(s) along with the current angle in degrees (0-180) on the LCD display. 

  The system is implemented as a state machine with 4 possible states:
     State 0:
        Switch 0 = 0
        Switch 1 = 0
        No motors Enabled
     State 1:
        Switch 0 = 1
        Switch 1 = 0
        Servo1 is Enabled
     State 2:
        Switch 0 = 0
        Switch 1 = 1
        Servo2 is Enabled
     State 3:
        Switch 0 = 1
        Switch 1 = 1
        Servo1 and Servo2 are enabled
  
  The algorithm uses a non-blocking poll to read the switches and change states accordingly
    1. Get the current state
    2. Has the state changed since last time?
      If so:
        a. Clear the LCD
        b. Print the state label to the first line of the LCD
        c. Update the previous state variable for next iteration
      If not:
        a. Continue normally
    3. Get potentiometer position (in degrees)
    4. Move the servo(s) to that position
    5. Print the current position in degrees to the second line of the LCD
  
  Hardware Peripherals:
    1. A 10kΩ potentiometer is connected to Arduino Uno Analog pin 5 to control the position
    2. 8 position DIP switch is used to trigger state changes (only using the first two, the last 6 are unused)
       a. Digital pin 02 = Swtich 0
       b. Digital pin 03 = Switch 1
    3. Generic 2x16 LCD screen is used to display state change and angle
       a. Digital Pin 04 = D4               : LCD buffer LSB
       b. Digital Pin 05 = D5               :
       c. Digital Pin 06 = D6               :
       d. Digital Pin 07 = D7               : LCD buffer MSB
       e. Digital Pin 11 = Register Select  : 0 = command, 1 = data
       f. Digital Pin 12 = Enable           : Command/Data sent on trailing edge. Keep low, Set, then clear to write.
    4. Adafruit motor shield V1.2
       a. Digital Pin 09 = "SERVO_2" header
       b. Digital Pin 10 = "SER1" header
  
  Port manipulation was used for faster execution. Each port is controlled by 3 registers
    1. DDRX:  (R/W) Data direction register X which specifies whether the pins are INPUT (default, hi-z) or OUTPUT (low-z). Always set this reg first!
              0 = input, 1 = output
              The low level equivalent of pinMode()
    2. PORTX: (R/W) Uses internal pull-ups to set the logic state of pins HIGH or LOW.
              When configured OUTPUT, think of it as setting state HIGH/LOW. 
              When configured INPUT think of it as setting the pull-up.
              Write 0 to set LOW/disable pull-up, write 1 to set HIGH/enable pull-up.
              The low level equivalent of digitalWrite(), see also bitWrite()
    3. PINX:  (R) Reads the state of input pins
              The low level equivalent of digitalRead()
    Port D bits 2 and 3 are used to read switches 0 and 1 respectively
    Port D bits 4 through 7 are used to send 4-bit data to the LCD buffer
    Port B bits 3 and 4 are used for LCD control bits Register Select and Enable respectively 

  Standard hobbyist servos accept a short pulse and change position according to the width of that pulse
  They typically have about 180 degrees of motion (theoretically)
    1ms pulse   -> 0 degrees
    1.5ms pulse -> 90 degrees
    2ms pulse   -> 180 degrees

  LCD Commands (RS = 0)
    "Function Set"        : 0x001WLDXX, where W = width (4 vs 8 bit), L = lines ( 1 vs 2), and D = dots (5x7 vs 5x10). X = don't care
    "Display Control"     : 0x00001DCB, where D = Display (off vs on), C = cursor (off vs on), B = blink (off vs on)
    "Display clear"       : 0x00000001, Clears the display and returns the cursor to address 0. No levers associated with this command
    "Cursor Home"         : 0x0000001X, where X = don't care. Returns cursor home and shifted display to original position
    "Entry mode"          : 0x000001CS, where C = cursor move (decrement vs increment), S = shift display (off vs on)
    "Cursor Control"      : 0x0001MDXX, where M = move (cursor move vs display move), D = direction (left vs right)
    "DDRAM Address Set"   : 0x1AAAAAAA, where A = 7-bit DDRAM address. Line 1 starts at address 0x00, Line 2 starts at 0x40
==================================================================================================*/

#include <Servo.h>

// Declare hardware peripherals
  const byte POT = 5;              // Map the potentiometer to Uno Analog pin 5
  Servo servo1;                    // Declare smaller servo
  Servo servo2;                    // Declare larger servo

// Declare global variables
  int potPos = 0;                  // Stores the current position of the potentiometer (0-1023)
  byte currentState = 0;           // Stores the current state (0-3). No need for a full int, byte takes up less space
  byte previousState = 5;          // Stores the previously recorded state. Initialize with out of bounds value to guaratee a state change on startup

void setup() {
// Servo Setup
  servo1.attach(10, 1000, 2000);   // Attach the smaller servo to pin 10, set min angle to 0 (1ms), max angle to 180 (2ms)
  servo1.write(90);                // Initialize to 90 degrees (1.5ms)
  servo2.attach(9, 1000, 2000);    // Attach the larger servo to pin 9, set min angle to 0 (1ms), max angle to 180 (2ms)
  servo2.write(90);                // Initialize to 90 degrees (1.5ms)

// Switch Setup
  DDRD  &= B11110011;              // Configure digital pins 2 and 3 as inputs for the hardware switches. leave others unchanged as they are used elsewhere
  PORTD |= B00001100;              // Set pull-ups on digital pins 2 and 3
// NOTE active low logic. Switch open: pins 2 and 3 are set. Switch closed: pins 2 and 3 cleared.

// LCD Setup
  lcd4BitInit();                   // Initialize LCD in 4-bit mode
}
//-----------------------------------------------------------------------------------------------------------------------------
void loop() {
  currentState = readSwitches();          // Get the current state
 
  if(currentState - previousState != 0){  // Has the state changed since last time?
    printStateLabel(currentState);        //   If so:  Print the state label to the first line of the LCD
    previousState = currentState;         //           Update the previous state variable for next iteration
  }                                       //   If not: skip this block and continue as normal                
  
  potPos = potPositionDegrees(POT);       // Get potentiometer position
  servoWrite(currentState, potPos);       // Move servo to that position
  printPos(String(potPos));               // Print the angle in degrees to the second line of the LCD
}
//-----------------------------------------------------------------------------------------------------------------------------
byte readSwitches(){
  //  1. Read Port D Pins (PIND)
  //  2. Invert them for active high logic (~)
  //  3. Single out digital pins 2 and 3 which are connected to the hardware switches and ignore others as they are not important now. (& B00001100)
  //  4. Shift right two times to place the bits we care about in the LSB position (>> 2)
  //  5. return now holds an integer value between 0 through 3
  return (~PIND & B00001100) >> 2;
}

void printStateLabel(byte state){   // Prints a different label for each state on line 1 of the LCD
  lcdWriteByte(0, 0x01);            // Clear LCD so it can be overwritten
  switch(state){
    case 0: // No motors
      lcdPrintLn("No Motor Enabled");
      break;
      
    case 1: // Servo1
      lcdPrintLn("Motor 1:");
      break;

    case 2: // Servo2
      lcdPrintLn("Motor 2:");
      break;

    case 3: // Servo1 & Servo2
      lcdPrintLn("Motor 1 & 2:");
      break;    
  }
}

int potPositionDegrees(const int potentiometer){  // Covert potentiometer position as read by the ADC to degrees
  int pos = analogRead(potentiometer);            //    Read the current potentiometer position from the ADC (0 to 1023)
  pos = 180 * (long) pos / 1023;                  //    Map that position to a number in the range between 0 and 180
  return pos;
}

// Given the state and angle in degrees, change the corresponding motor(s) to that angle
void servoWrite(byte state, int pos){   
  switch(state){
    case 0:  // No motors enabled
      break;
         
    case 1:  // Set servo1 position in degrees
      servo1.write(pos);
      break;

    case 2:  // Set servo2 position in degrees
      servo2.write(pos);
      break;

    case 3:  // Set servo1 and servo2 position in degrees
      servo1.write(pos);
      servo2.write(pos);
      break;    
  }
}

void printPos(String pos){  // Given a string with angle in degrees (up to 3 digits), print that angle to the LCD
  lcdWriteByte(0, 0xC0);    //    Move cursor to start of line 2
  lcdPrintLn("   ");        //    Clear line 2 by overwriting previously printed angle with 3 blank space characters
  lcdWriteByte(0, 0xC0);    //    Move cursor to start of line 2
  lcdPrintLn(pos);          //    Print the string containing angle to the LCD screen
}

void lcdPrintLn(String line){
  for (int i = 0; line[i] != '\0'; i++){  // For each element in the string up until '\0' (All strings end with '\0'):
    lcdWriteByte(1, line[i]);             //    Write the character data byte to the LCD
  }
}

void lcdWriteByte(bool RS, byte bite){    // Write either a command (Rs=0) or data (Rs=1) byte to the LCD one nibble at a time using 4 bit mode
  byte upperNibble = bite & 0xF0;         //    Mask upper nibble of bite, clear lower nibble so result takes the form: 0xUUUU0000
  byte lowerNibble = (bite & 0x0F) << 4;  //    Mask lower nibble of bite, clear upper nibble then shift left 4 so result takes the form 0xLLLL0000
  if (RS)                                 //    Evaluate the passed value of RS
    PORTB |= B00001000;                   //      If 1 : Set Register select bit to write data
  else
    PORTB &= B11110111;                   //      If 0 : Clear Register select bit to write command
  PORTD &= B00001111;                     //    Clear LCD buffer
  PORTD |= upperNibble;                   //    Write upperNibble to LCD buffer
  PORTB |= B00010000;                     //    En=1
  PORTB &= B11101111;                     //    En=0. Command is sent on trailing edge of enable
  _delay_us(1600);                        //    Wait >37us for command to process
  PORTD &= B00001111;                     //    Clear LCD buffer
  PORTD |= lowerNibble;                   //    Write lowerNibble to LCD buffer
  PORTB |= B00010000;                     //    En=1
  PORTB &= B11101111;                     //    En=0. Command is sent on trailing edge of enable
  _delay_us(1600);                        //    Wait >37us for command to process
}

void lcd4BitInit(){       // Initialize the LCD screen in 4-bit mode
  lcdReset();             //    Reset LCD into 4-bit mode
  lcdWriteByte(0, 0x28);  //    Configure "Function Set" Register (0x28)  : 4 bit mode, 2 lines, and 5x7 dots
  lcdWriteByte(0, 0x08);  //    Turn off all "Display Control" (0x08)     : Display, cursor, and blink all off
  lcdWriteByte(0, 0x01);  //    "Display clear" command (0x01)            : Clears the display and returns the cursor to address 0
  lcdWriteByte(0, 0x06);  //    Configure "Entry Mode" Register (0x06)    : Sets auto increment cursor and disables display shift
  lcdWriteByte(0, 0x0C);  //    Configure "Display Control" Register      : Re-Enable screen, no cursor or blink
}

void lcdReset(){          // Reset LCD into 4-bit mode. Only needs to be called once at start up.
  DDRD  |= B11110000;     //    Initialize digital pins 7-4 as outputs for the LCD buffer, leave others unchanged as they are used elsewhere
  PORTD &= B00001111;     //    Clear D7-D4 to 0 as a neutral starting point
  DDRB  |= B00011000;     //    Initialize digital pins 12 (En) and 11 (Rs) as outputs for the LCD control
  PORTB &= B11100111;     //    Clear En, Rs to 0 as a neutral starting point

  // Data sheet calls this initialization by instruction. Send 0x3 three times, then 0x2 to reset into 4-bit mode.
  _delay_ms(16);          //    Wait >15ms for Led Vcc to rise to the correct voltage
  PORTD |= B00110000;     //    Write upper nibble 0x3 to LCD buffer
  PORTB |= B00010000;     //    En=1
  PORTB &= B11101111;     //    En=0. Command is sent on trailing edge of enable
  _delay_ms(5);           //    Wait >4.1ms for command to process
  PORTB |= B00010000;     //    En=1 (NOTE: Upper nibble of PORTD is still 0x3, no need to rewrite, just send enable signal)
  PORTB &= B11101111;     //    En=0. Command is sent on trailing edge of enable
  _delay_us(105);         //    Wait >100us for command to process
  PORTB |= B00010000;     //    En=1 (NOTE: Upper nibble of PORTD is still 0x3, no need to rewrite, just send enable signal)
  PORTB &= B11101111;     //    En=0. Command is sent on trailing edge of enable
  _delay_us(38);          //    Wait >37us for command to process. See table 6 "Function Set"
  PORTD &= B00101111;     //    Write upper nibble 0x2 to LCD buffer (NOTE: it was already 0x3, just clear bit 4 instead of clearing the whole thing and rewriting)
  PORTB |= B00010000;     //    En=1
  PORTB &= B11101111;     //    En=0. Command is sent on trailing edge of enable
  _delay_us(38);          //    Wait >37us for command to process. See table 6 "Function Set"
  // At this point LCD is reset and listening for 4-bit commands.
}
