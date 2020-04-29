////////////////////////////////////////////////////////////////////////////////
// A Digital IO Library for Arduino
// Copyright (c) 2015-2020 Dan Truong
////////////////////////////////////////////////////////////////////////////////
// 1_Button_and_builtin_LED
//
// This simple demo declares push button and an LED. The LED turns on if you
// press the push-button.
//
// The code is set up for an Arduino Uno. Other boards may have different IO pin
// configurations. Some may not even have a built-in LED. For the UNO, Pin 7 is
// the AVR port D7. Pin 13 is the AVR port B5. Google Arduino <you board> to find
// a diagram of your board's pinout mapping.
//
// This library offers multiple ways to declare your pin variables. It makes
// trade offs between portability vs performance and compactness of the code.
// Comment/uncomment each of the ways to see the impact at compile time on code
// size. I'm putting detailed comments to explain what's what.
//
// Code size improvements on Arduino Uno up to 17%.
//
//      2422  digitalRead()/digitalWrite()
//  1%  2406  Arduino Pin
//  7%  2254  Arduino Pin + DIGITAL_IO_AVR_OPTIMIZED
// 15%  2066  AVR Port + DIGITAL_IO_AVR_DEFAULT
// 17%  2014  AVR Port
//
////////////////////////////////////////////////////////////////////////////////
// Wiring
////////////////////////////////////////////////////////////////////////////////
// The push-button should not conduct at rest and conduct when pressed. Wire it
// between pin 7 and the ground. The program will enable the pull-up resistor so
// that with the push-button at rest, the pin is pulled to 5V and reads HIGH.
// When you press the push-button, it grounds pin 7 and reads LOW.
// Wiring for digitalIo<7, HIGH>:
//
//           -- Switch
// pin 7 ---o  o---.
//                 |
//                ---
//                /// Gnd
//
// We use the built in LED on pin 13 so there's nothing more to wire.
// If you want to use your own LED on a different pin, wire it to an IO pin in
// series with a 200 Ohm resistor, and to the ground. When you write HIGH to the
// pin, it goes high
// Wiring for digitalIo<7, LOW> as output (control a low power LED):
// When you output HIGH, it turns on the LED
//
//          200 Ohm
// pin 7 ---====---.
//                 |
//                 V  23mA LED
//                 -
//                 |
//                ---
//                /// Gnd
////////////////////////////////////////////////////////////////////////////////

// Uncomment this definition to have the library print to the Serial console
// a bunch of debug information to show you what's going on under the hood.
// This must be defined before including the library to take effect.
//#define DIGITAL_IO_DEBUG 1


// Uncomment one of the led and button variables declaration pairs to see their
// effect on the size of the compiled code, and portability:

////////////////////////////////////////////////////////////////////////////////
// BEST COMPATIBILITY:
// This is the most portable version of the code. It should work for AVR and
// non AVR Arduino boards (ARM, Intel, etc.).
// This declares the button is attached to pin 7, and is in its default rest
// state when it is HIGH. When it is LOW, it means the button is on.
// It declares an LED is attached to pin 13, and that we turn it off when we set
// the level to LOW, and turn it on when we set thelevel to HIGH.
// Uno: 2406 bytes program storage, 190 bytes dynamic memory
#include <DigitalIO.hpp>
digitalIo<7, HIGH> button;
digitalIo<13, LOW> led;

////////////////////////////////////////////////////////////////////////////////
// This is as portable, but if the board is an AVR board, it applies some code
// optimizations. There's still an Arduino Pin to AVR Port conversion so it's
// not quite as compact as it could.
// Uno: 2254 bytes program storage, 190 bytes dynamic memory
//#define DIGITAL_IO_AVR_PINMODE DIGITAL_IO_AVR_OPTIMIZED
//#include <DigitalIO.hpp>
//digitalIo<7, HIGH> button;
//digitalIo<13, LOW> led;

////////////////////////////////////////////////////////////////////////////////
// BEST PERFORMANCE:
// This only compiles for AVR boards, and generates optimized code that should 
// be as compact and fast as possible, as it avoids pin to port conversions. It
// should be more compact than if you were to code this yourself directly.
// Uno: 2014 bytes program storage, 190 bytes dynamic memory
//#include <DigitalIO.hpp>
//digitalIoAvr<D,7, HIGH> button;
//digitalIoAvr<B,5, LOW> led; // pin 13

////////////////////////////////////////////////////////////////////////////////
// This only compiles for AVR boards, and generates code that's more portable.
// If your board doesn't work with the optimal code version you could try this.
// Uno: 2066 bytes program storage, 190 bytes dynamic memory
//#define DIGITAL_IO_AVR_PORTMODE DIGITAL_IO_AVR_DEFAULT
//#include <DigitalIO.hpp>
//digitalIoAvr<D,7, HIGH> button;
//digitalIoAvr<B,5, LOW> led; // pin 13

////////////////////////////////////////////////////////////////////////////////
// native_loop Arduino calls to digitalRead() and digitalWrite()
// To test this, comment all button and led variables related code and uncomment
// the pinMode() and native_loop() code.
// Uno: 2422 bytes program storage, 188 bytes dynamic memory

////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);
  
  // Activate the pull-up resistor for the button
  button.inputPullupMode();
  // Set the port as output for the LED to turn it on/off
  led.outputMode();

  // If you uncomment native_loop() below, you should uncomment this: 
  // pinMode(7, INPUT_PULLUP);
  // pinMode(13, OUTPUT);
}

////////////////////////////////////////////////////////////////////////////////
void loop() {
  // Here's equivalent loop bodies that perform the same task in different ways.
  // Uncomment only one of them and try it, then read the code and description.

  // Uses digitalIO class and traditional read/write interface
  //read_write_loop();
  
  // Uses digitalIO class and convenient routines for easy to read code
  //isOn_isOff_loop();

  // Uses digitalIO class flipped() debounce function to read the button
  // (costs 108 bytes of program memory)
  //flipped_loop();

  // Uses digitalIO class triggered() debounce function to read spikes, and toggle()
  // (costs 170 bytes of program memory)
  triggered_loop();
  
  // Doesn't use digitalIO but Arduino built-in digital IO methods
  // native_loop();
}

////////////////////////////////////////////////////////////////////////////////
// The library also has more classic read()/write() methods.
// The code performs exactly the same task, but you'll notice it is harder to
// glance at it and understand what's happening.
////////////////////////////////////////////////////////////////////////////////
void read_write_loop() {
  // When you press the button, the pin is grounded so it reads LOW.
  if (button.read() == LOW) {
    // When you write HIGH, the pin voltage goes to 5V and turns on.
    led.write(HIGH);
  } else {
    led.write(LOW);
  }
  Serial.print(button.read());
  Serial.println(led.read());
  delay(500);
}

////////////////////////////////////////////////////////////////////////////////
// Code example with the convenience isOn()/isOff(), turnOn()/turnOff() methods.
// This takes advantage of the default state declarations we made when creating
// the variables, to simplify code reading.
////////////////////////////////////////////////////////////////////////////////
void isOn_isOff_loop() {
  if (button.isOn()) {
    led.turnOn();
  } else {
    led.turnOff();
  }
  Serial.print(button.isOn());
  Serial.println(led.isOn());
  delay(500);
}

////////////////////////////////////////////////////////////////////////////////
// Code example with the debounced flipped() method.
// This method returns 0 when button state has not changed, 1 if it was pressed
// and -1 if it was released. These 3 states allow us to not write to the LED
// port if the button has not changed state.
// Furthermore, the debounce of the button means we can use a mechanical button
// that causes noise on the line without having the code glitching.
////////////////////////////////////////////////////////////////////////////////
void flipped_loop() {
  // notice: int8_t is signed. Unsigned, a release would be represented by 255.
  const int8_t state = button.flipped();
  if (state == 1) {
    led.turnOn();
  } else if (state == -1) {
    led.turnOff();
  }
  Serial.print(button.isOn());
  Serial.println(led.isOn());
  delay(500);
}

////////////////////////////////////////////////////////////////////////////////
// Code example with the debounced triggered() method.
// The triggered method is for detecting brief spikes (i.e. knock sensor) so
// this loop function works differently. If a transition is detected on the
// button, it toggles the LED. This means if you press the button slowly, it
// will behave the same. However if you tap the button briefly it will toggle
// the LED. That's because triggered() doesn't track the HIGH or LOW level, but
// transitions on the pin (a spike or level change).
// triggered has debounce code to handle noisy spikes so the same spike doesn't
// trigger multiple times the code if it's noisy.
////////////////////////////////////////////////////////////////////////////////
void triggered_loop() {
  if (button.triggered()) {
    led.toggle();
    Serial.print(button.isOn());
    Serial.println(led.isOn());
  }
  delay(5);
}

////////////////////////////////////////////////////////////////////////////////
// Baseline loop without using the DigitalIO library.
////////////////////////////////////////////////////////////////////////////////
void native_loop()
{
  if (digitalRead(7) == LOW) {
    digitalWrite(13, HIGH);
  } else {
    digitalWrite(13, LOW);    
  }
  Serial.print(digitalRead(7));
  Serial.println(digitalRead(13));
  delay(500);
}
