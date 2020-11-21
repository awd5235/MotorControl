/*=========================================== MOTOR TEST ===========================================
  Using two switches, select up to two motors to control with a potentiometer and display current position
  the current position (in percent 0-99) on a 7 segment display. Sound a buzzer when motors are
  too close to max position, indicating that they might be strained.

  Use "switch...case" statements to implement system as a finite state maschine.
  Consider Switch 1 to be the LSB and Switch 2 to be the MSB
  Switches = 0x0000, case 0: no motors are selected, potentiometer has no effect, screen displays "00"
  Switches = 0x0001, case 1: Small servo is activated, deactivate other motors, move to position of potentiometer and display
  Switches = 0x0010, case 2: Tall servo is activated, deactivate other motors, move to position of potentiometer and display
  Switches = 0x0011, case 3: Both Small and Tall servos are activated, deactivate other motors, move to position of potentiometer and display



  ------------------------------------------ About PORT X -------------------------------------------
  Port manipulation allows for the simultaneous manipulation of digital pins. It is faster, but less readable and less transferrable.
  There are three ports available PORT A (analog 0-5), PORT B (Digital 8-13), and PORTD (digital 0-7)
  The LSB of each port corresponds to lowest controlled pin and MSB corresponds to the highest controlled pin. (e.g. PORTD LSB = pin0, MSB = pin7)

  Controlled by 3 registers:
  -DDRX:  (R/W) Data direction register D which specifies whether the pins are INPUT (default, hi-z) or OUTPUT (low-z). Always set this reg first!
          0 = input, 1 = output
          The low level equivalent of pinMode()
  -PORTX: (R/W) Uses internal pull-ups to set the logic state of pins HIGH or LOW.
          When configured OUTPUT, think of it as setting state HIGH/LOW. When configured INPUT think of it as setting the pull-up.
          Write 0 to set LOW/disable pull-up, write 1 to set HIGH/enable pull-up.
          The low level equivalent of digitalWrite(), see also bitWrite()
  -PINX:  (R) Reads the state of input pins
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

  This project uses Motor Shield V1.2
    "SER1" header -> PWM1B -> Controlled by Uno Pin 10
    "SERVO_2" header -> PWM1A -> Controlled Uno Pin 9
  --------------------------------------------------------------------------------------------------

  ------------------------------------- About Interrupts -------------------------------------------
  Arduino Uno only has interrupts on pins 2 (INT.0) & 3 (INT0.1)
  millis(), micros(), and delay() do not work in ISR. delayMicroseconds() will work
  To share data between ISR and main declare global variables as volatile

  For this porject:
  Dipswitch 0 = pin 2 -> INT.0
  Dipswitch 1 = pin 3 -> INT.1


  Note: ISR is a defined keyword do not use
  --------------------------------------------------------------------------------------------------


  ------------------------------------------- About LCD --------------------------------------------
  2x16 Character LCD.

  For this project configure LCD in 4-bit mode using Port D
    Digital Pin 04 = D4               : LCD buffer LSB
    Digital Pin 05 = D5               :
    Digital Pin 06 = D6               :
    Digital Pin 07 = D7               : LCD buffer MSB
    Digital Pin 11 = Register Select  : 0 = command, 1 = data
    Digital Pin 12 = Enable           : Command/Data sent on trailing edge. Keep low, Set, then clear to write.

  LCD COMMANDS
    "Function Set"        : 0x001WLDXX, where W = width (4 vs 8 bit), L = lines ( 1 vs 2), and D = dots (5x7 vs 5x10). X = don't care
    "Display Control"     : 0x00001DCB, where D = Display (off vs on), C = cursor (off vs on), B = blink (off vs on)
    "Display clear"       : 0x00000001, Clears the display and returns the cursor to address 0. No levers associated with this command
    "Cursor Home"         : 0x0000001X, where X = don't care. Returns cursor home and shifted display to original position
    "Entry mode"          : 0x000001CS, where C = cursor move (decrement vs increment), S = shift display (off vs on)
    "Cursor Control"      : 0x0001MDXX, where M = move (cursor move vs display move), D = direction (left vs right)
  --------------------------------------------------------------------------------------------------

  ------------------------------------------- TO DO ------------------------------------------------
  1. Print state to LCD (best way to buffer/refresh/print quickly?)
  2. Interrupt driven FSM (Without reevaluating state every single loop)
  3. Port manipulation of Servos
  4. Add alarm for 180 degrees


  BONUS
    1. Replace pot with light sensor
    2. Roll your own LCD lib
    3. Roll your own servo lib
  --------------------------------------------------------------------------------------------------
  ====================================================================================================*/

#include <Servo.h>

// Hardware Peripherals
Servo small_servo;                      // Declare the smaller servo
Servo tall_servo;                       // Declare the larger servo
const int POT = 5;                      // Map the potentiometer to Uno Analog pin 5

// Declare Intermediate Variables
int potPos = 0;                         // Stores the position of the potentiometer
volatile byte FSMstate = 0;             // Stores the state of the finite state machine, only need a byte so no need for a two byte int
//String line1;
//String line2;

void setup() {
  // Servo Setup
  small_servo.attach(10, 1000, 2000);   // Attach to pin 10 to small servo, set min angle to 0 (1ms), max angle to 180 (2ms)
  small_servo.write(0);                 // Alternatively "small_servo.writeMicroseconds(1000);" to initialize at 0 degrees
  tall_servo.attach(9, 1000, 2000);     // Attach to pin 9, set min angle to 0 (1ms), max angle to 180 (2ms)
  tall_servo.write(0);                  // Alternatively "tall_servo.writeMicroseconds(1000);" to initialize at 0 degrees

  // Switch setup
  DDRD  &= B11110011;                   // Configure digital pins 2 and 3 as inputs for the hardware switches. leave others unchanged as they are used elsewhere
  PORTD |= B00001100;                   // Set pull-ups on digital pins 2 and 3
  // NOTE active low logic. Switch open: pins 2 and 3 are set. Switch closed: pins 2 and 3 cleared.

  // LCD Setup
  LCDinit4bit();                        // Initialize LCD in 4-bit mode

  attachInterrupt(digitalPinToInterrupt(2), interruptServiceRoutine, CHANGE);   // Attach interrupt to switch 1, trigger on any change, and call interruptServiceRoutine
  attachInterrupt(digitalPinToInterrupt(3), interruptServiceRoutine, CHANGE);   // Attach interrupt to switch 2, trigger on any change, and call interruptServiceRoutine

  //Serial.begin(9600);                 // For testing purposes
}


void loop() {
  // Switch to different cases based on the value of the current state (FSMstate)
  // FSMstate = 0, case 0: no motors are selected, potentiometer has no effect, screen displays "--"
  // FSMstate = 1, case 1: Small servo is activated, deactivate other motors, move to position of potentiometer and display angle
  // FSMstate = 2, case 2: Tall servo is activated, deactivate other motors, move to position of potentiometer and display angle
  // FSMstate = 3, case 3: Both Small and Tall servos are activated, deactivate other motors, move to position of potentiometer and display angle
  switch (FSMstate)
  {
    case 0:  // No motors
      break;

    case 1:  // Small Servo only
      potPos = analogRead(POT);                   // read the value of the potentiometer (value between 0 and 1023)
      potPos = map(potPos, 0, 1023, 0, 180);      // scale it to use it with the servo (value between 0 and 180)
      small_servo.write(potPos);                  // sets the small servo position according to the scaled value
      break;

    case 2:  // Tall Servo only
      potPos = analogRead(POT);                   // read the value of the potentiometer (value between 0 and 1023)
      potPos = map(potPos, 0, 1023, 0, 180);      // scale it to use it with the servo (value between 0 and 180)
      tall_servo.write(potPos);                   // sets the tall servo position according to the scaled value
      break;

    case 3:  // Both servos
      potPos = analogRead(POT);                   // read the value of the potentiometer (value between 0 and 1023)
      potPos = map(potPos, 0, 1023, 0, 180);      // scale it to use it with the servo (value between 0 and 180)
      small_servo.write(potPos);                  // sets the small servo position according to the scaled value
      tall_servo.write(potPos);                   // sets the tall servo position according to the scaled value
      break;
  }
}

void interruptServiceRoutine()
{
  // Change state based on switch inputs
  //  1. Read Port D Pins (PIND)
  //  2. Invert them for active high logic (~)
  //  3. Single out digital pins 2 and 3 which are connected to the hardware switches and ignore others as they are not important now. (& B00001100)
  //  4. Shift right two times to place the bits we care about in the LSB position (>> 2)
  //  5. FSMstate now holds an integer value between 0 and 3
  FSMstate = (~PIND & B00001100) >> 2;
}

void LCD4BitPrintLn(String line)
{
  for (int i = 0; line[i] != '\0'; i++) // For each element in the string up until '\0' (All strings end with '\0'):
    LCD4BitWriteByte(1, line[i]);       //    Write the character data byte to the LCD
  LCD4BitWriteByte(0, 0x02);            // Send the cursor home command so next write can start at the beginning of the line
}

void LCDinit4bit()            // Initialize the LCD screen in 4-bit mode
{
  LCDreset();                 // 1. Reset LCD into 4-bit mode
  LCD4BitWriteByte(0, 0x28);  // 2. Configure "Function Set" Register (0x28)  : 4 bit mode, 2 lines, and 5x7 dots
  LCD4BitWriteByte(0, 0x08);  // 3. Turn off all "Display Control" (0x08)     : Display, cursor, and blink all off
  LCD4BitWriteByte(0, 0x01);  // 4. "Display clear" command (0x01)            : Clears the display and returns the cursor to address 0
  LCD4BitWriteByte(0, 0x06);  // 5. Configure "Entry Mode" Register (0x06)    : Sets auto increment cursor and disables display shift
  LCD4BitWriteByte(0, 0x0F);  // 6. Configure "Display Control" Register      : Enable screen, cursor, and blink
}

void LCDreset()         // Reset LCD into 4-bit mode. Only needs to be called once at start up.
{
  // Port Set up
  DDRD  |= B11110000;   // 01. Initialize digital pins 7-4 as outputs for the LCD buffer, leave others unchanged as they are used elsewhere
  PORTD &= B00001111;   // 02. Clear D7-D4 to 0 as a neutral starting point
  DDRB  |= B00011000;   // 03. Initialize digital pins 12 (En) and 11 (Rs) as outputs for the LCD control
  PORTB &= B11100111;   // 04. Clear En, Rs to 0 as a neutral starting point

  // Data sheet calls this initialization by instruction. Send 0x3 three times, then 0x2 to reset into 4-bit mode.
  _delay_ms(100);       // 05. Wait >40ms for Led Vcc to rise to the correct voltage
  PORTD |= B00110000;   // 06. Write upper nibble 0x3 to LCD buffer
  PORTB |= B00010000;   // 07. En=1
  PORTB &= B11101111;   // 08. En=0. Command is sent on trailing edge of enable
  _delay_ms(5);         // 09. Wait >4.1ms for command to process
  PORTB |= B00010000;   // 10. En=1 (NOTE: Upper nibble of PORTD is still 0x3, no need to rewrite, just send enable signal)
  PORTB &= B11101111;   // 11. En=0. Command is sent on trailing edge of enable
  _delay_us(150);       // 12. Wait >100us for command to process
  PORTB |= B00010000;   // 13. En=1 (NOTE: Upper nibble of PORTD is still 0x3, no need to rewrite, just send enable signal)
  PORTB &= B11101111;   // 14. En=0. Command is sent on trailing edge of enable
  _delay_us(150);       // 15. Wait >100us for command to process
  PORTD &= B00101111;   // 16. Write upper nibble 0x2 to LCD buffer (NOTE: it was already 0x3, just clear bit 4 instead of clearing the whole thing and rewriting)
  PORTB |= B00010000;   // 17. En=1
  PORTB &= B11101111;   // 18. En=0. Command is sent on trailing edge of enable
  _delay_us(150);       // 19. Wait >100us for command to process
  // At this point LCD is reset and listening for 4-bit commands.
}

void LCD4BitWriteByte(bool RS, byte BITE) // Write either a command (Rs=0) or data (Rs=1) byte to the LCD one nibble at a time using 4 bit mode.
{
  byte upperNibble = BITE & 0xF0;         // 01. Mask upper nibble of BITE, clear lower nibble so result takes the form: 0xUUUU0000
  byte lowerNibble = (BITE & 0x0F) << 4;  // 02. Mask lower nibble of BITE, clear upper nibble then shift left 4 so result takes the form 0xLLLL0000
  if (RS)                                 // 03. Evaluate the passed value of RS
    PORTB |= B00001000;                   //     If 1 : Set Register select bit to write data
  else
    PORTB &= B11110111;                   //     If 0 : Clear Register select bit to write command
  PORTD &= B00001111;                     // 04. Clear LCD buffer
  PORTD |= upperNibble;                   // 05. Write upperNibble to LCD buffer
  PORTB |= B00010000;                     // 06. En=1
  PORTB &= B11101111;                     // 07. En=0. Command is sent on trailing edge of enable
  _delay_us(1600);                        // 08. Wait >40us for command to process
  PORTD &= B00001111;                     // 09. Clear LCD buffer
  PORTD |= lowerNibble;                   // 10. Write lowerNibble to LCD buffer
  PORTB |= B00010000;                     // 11. En=1
  PORTB &= B11101111;                     // 12. En=0. Command is sent on trailing edge of enable
  _delay_us(1600);                        // 13. Wait >40us for command to process
}
