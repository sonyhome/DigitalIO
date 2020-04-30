////////////////////////////////////////////////////////////////////////////////
// A Digital IO Library for Arduino
// Copyright (c) 2015-2020 Dan Truong
////////////////////////////////////////////////////////////////////////////////
// Intro_Button_and_builtin_LED
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
// size.
////////////////////////////////////////////////////////////////////////////////
// Wiring
////////////////////////////////////////////////////////////////////////////////
// Put a push-button between pin 7 and the ground. If you don't have a button,
// you can just use a wire on pin 7 that you can plug in and out of the ground.
//
//           -- Switch
// pin 7 ---o  o---.
//                 |
//                ---
//                /// Gnd
//
// We use the built-in LED on pin 13 so there's nothing more to wire.
////////////////////////////////////////////////////////////////////////////////
#include <DigitalIO.hpp>

// This works for all Arduino boards
digitalIo<7, HIGH> button;
digitalIo<13, LOW> led;
// Comment the above two declaration and uncomment the following two for an
// optimized code alternative (AVR boards only): 808 bytes instead of 1134 bytes.
//digitalIoAvr<D,7, HIGH> button;
//digitalIoAvr<B,5, LOW> led;

void setup() {
  // Activate the pull-up resistor for the button
  button.inputPullupMode();
  // Set the port as output for the LED to turn it on/off
  led.outputMode();
}

void loop() {
  // If button is pressed then turn on the LED.
  led.write(button.isOn());
  delay(10);
}
