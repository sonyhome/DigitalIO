////////////////////////////////////////////////////////////////////////////////
// DigitalIO demo
// A straightforward library to use rotary encoders on Arduinos
////////////////////////////////////////////////////////////////////////////////
// 3_RotaryEncoder example
//
// This demo shows how to read a rotary encoder using the DigitalIO library.
// You can use interrupt or non-interrupt mode just by changing the declaration
// of your encoder variable.
// With encoders, you will either need to use the interrupt mode or a fast loop
// body (<10ms), if you want to catch the rotations without a glitch.
// The digitalRotaryEncoder class inherits all the methods of digitalIo, which
// all only access to the switchPin pin, and also defines rotaryRead().
// The encoders supported have 2 pins (often labelled A,B or Clock,Data) as well
// as ground and Vcc, and can be rotary buttons, or encoders attached to motors.
////////////////////////////////////////////////////////////////////////////////
// Example pinout for a Keyes KY-040 rotary encoder board
//
//    Uno        Encoder
//  Pin 3 ------ Ck
//  Pin 4 ------ Dt
//  Pin 5 ------ Sw
//     5V ------ + / Vcc
//    Gnd ------ Gnd
//
////////////////////////////////////////////////////////////////////////////////

// These macros can be used to change the behavior of the DigitalIO library
//#define DIGITAL_IO_DEBUG 3
//#define DIGITAL_IO_DEBOUNCE_DELAY 200
#include <DigitalIO.hpp>

// The digitalRotaryEncoder class template takes the following parameters:
// Pins:   switchPin, dataPin, clockPin(*),
// Limits: minValue, maxValue, (hard limit: -32767 to 32767)
//         useInterrupt(*).
// (*) If you use the interrupt mode, the clock pin must be a pin on your board
//     that supports interrupts (pin 2 & 3 on Arduino Uno), see your manual.
// Below are 4 ways to instantiat ethe class. Pick one.
// Polling mode:
digitalRotaryEncoder<5,4,3,-16,16,false> encoder;
// Interrupt mode (more responsive):
//digitalRotaryEncoder<5,4,3,-16,16,true> encoder;
// AVR Polling mode (more compact):
//digitalRotaryEncoderAvr<'D',5,'D',4,'D',3,-16,16,false, 1> encoder;
// AVR Interrupt mode (more compact and responsive):
//digitalRotaryEncoderAvr<'D',5,'D',4,'D',3,-16,16,true, 1> encoder;

// The digitalIo class template takes 2 inputs: pin, defaulValue.
// The default value is the level at rest for your device. In this case LOW
// to turn off the LED.
digitalIo<13, LOW> led;
// AVR mode (more compact):
//digitalIoAvr<'B', 5, LOW> led;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Starting!");
  led.outputMode();
}

void loop() {
  const int32_t curr = encoder.rotaryRead();
  
  // If the button is pressed down display the value
  switch(encoder.flipped())
  {
    case 1:
      Serial.print("Button pressed, position: ");
      Serial.println(curr);
      led.turnOn();
      break;
    case -1:
      led.turnOff();
    case 0:
      break;
  }
  delay(10);
}
