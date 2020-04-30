////////////////////////////////////////////////////////////////////////////////
// DigitalIO demo
// A straightforward library to use rotary encoders on Arduinos
////////////////////////////////////////////////////////////////////////////////
// Rotary Encoder Switch example
//
// A rotary encoder looks like a metallic potentiometer, but when you turn it
// you feel the clicks of the different steps of the encoder. It works by sending
// offset square signals to 2 pins, which are used to detect the turn direction
// and the number of steps. Clockwise will increase and counter will decrease the
// value of the counter monitoring the encoder.
//
// This demo shows how to use a rotary encoder using the DigitalIO library's
// digitalEncoder class. It supports interrupt or non-interrupt monitoring just
// by changing a flag. It also supports min/max boundaries for the values.
//
// With encoders, you will either need o use the interrupt mode or a fast loop
// body (delay < 10ms), if you want to catch the rotations without a glitch.
// The digitalEncoder class inherits the digitalIo class methods, which apply
// to switchPin (signals when you press down), and defines rotaryRead() to read
// the encoder value.
////////////////////////////////////////////////////////////////////////////////
// The encoders supported have 2 pins (often labelled A,B or Clock,Data) as well
// as ground and Vcc, and can be rotary buttons, or encoders attached to motors.
//
// Example wiring of a Keyes KY-040 rotary encoder board with an Arduino Uno:
//
// Arduino Uno        Rotart Encoder board
//       Pin 3 ------ Ck
//       Pin 4 ------ Dt
//       Pin 5 ------ Sw
//          5V ------ + / Vcc
//         Gnd ------ Gnd
//
////////////////////////////////////////////////////////////////////////////////
// The digitalEncoder class template takes the following template parameters:
//
// Port:   portName(+),       only for digitalEncoderAvr
// Pins:   switchPin,         pin number or port pin for digitalEncoderAvr
//         dataPin,           pin number or port pin for digitalEncoderAvr
//         clockPin(*),       pin number or port pin for digitalEncoderAvr
// Limits: minValue,          hard limit: -32767
//         maxValue,          hard limit:  32767
//         useInterrupt(*),   optional (default false)
//         interruptNumber(*) optional interrupt number for clockPin
//
// (*) If you use the interrupt mode, the clock pin must be a pin on your board
//     that supports interrupts (pin 2 & 3 on Arduino Uno are attached to
//     interrupts 0 and 1), see your manual for other boards (try all pins from
//     the smallest number, using interrupt 0 to use trial and error).
//     If you use digitalEncoder you dont need to specify interruptNumber.
//     If you use digitalEncoderAvr you must specify the interruptNumber.
//
// Below are 4 ways to instantiate the Encoder. Pick one. The interrupt version
// is more resilient to fast encoder movement. The Avr vesion compiles smaller
// and code runs faster, but only for boards built with an AVR microcontroller.
////////////////////////////////////////////////////////////////////////////////

// Debug macro displays what the library detects (U/D and value changes)
//#define DIGITAL_IO_DEBUG 2
//#define DIGITAL_IO_DEBOUNCE_DELAY 200
#include <DigitalIO.hpp>


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
  // Please uncomment only one of the digitalEncoder declarations at a time:
  // Polling mode (2648 bytes of program storage, 227 bytes of RAM)
  digitalEncoder<5,4,3,-16,16> encoder;
  // Interrupt mode  (2868 bytes of program storage, 240 bytes of RAM)
  // digitalEncoder<5,4,3,-16,16,true> encoder;
  // AVR Polling mode  (2496 bytes of program storage, 227 bytes of RAM)
  //digitalEncoderAvr<D,5,4,3,-16,16> encoder;
  // AVR Interrupt mode  (2766 bytes of program storage, 240 bytes of RAM)
  //static digitalEncoderAvr<D,5,4,3,-16,16,true, 1> encoder;

  const int32_t curr = encoder.readEncoder();
  
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
