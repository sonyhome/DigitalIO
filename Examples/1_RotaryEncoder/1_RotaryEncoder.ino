////////////////////////////////////////////////////////////////////////////////
// DigitalIO demo
// A straightforward library to use rotary encoders on Arduinos
////////////////////////////////////////////////////////////////////////////////
// Rotary encoder switch example for Arduino Uno
//
// This demo shows how to read an encoder switch using the DigitalIO library.
// You can use interrupt or non-interrput mode
//
////////////////////////////////////////////////////////////////////////////////
// Example pinout: Keyes KY-040 rotary encoder board
//
//    Uno        Encoder
//  Pin 3 ------ Ck
//  Pin 4 ------ Dt
//  Pin 5 ------ Sw
//     5V ------ +
//    Gnd ------ Gnd
//
////////////////////////////////////////////////////////////////////////////////

// These macros can be used to change the behavior of the DigitalIO library
#define DIGITAL_IO_DEBUG 3
//#define DIGITAL_IO_DEBOUNCE_DELAY 200

#include <DigitalIO.hpp>

// encoderData is an internal value you should never access 
volatile int32_t encoderData = 0;

// Polling mode:
digitalRotaryEncoder<5,4,3,-16,16,false, encoderData> encoder;
// Interrupt mode:
//digitalRotaryEncoder<5,4,3,-16,16,true, encoderData> encoder;
// AVR Polling mode:
//digitalRotaryEncoderAvr<'D',5,'D',4,'D',3,-16,16,false, 1, encoderData> encoder;
// AVR Interrupt mode:
//digitalRotaryEncoderAvr<'D',5,'D',4,'D',3,-16,16,true, 1, encoderData> encoder;

digitalIo<13, LOW> led;
//digitalIoAvr<'B', 5, LOW> led;

void setup() {
  Serial.begin(9600);
  delay(500);
  Serial.println("Starting!");
  led.outputMode();
}

bool nl = false;

int32_t prev = 0;

void loop() {
  const int32_t curr = encoder.rotaryRead();
  
  // If the switch is pressed down display the value
  switch(encoder.changed())
  {
    case 1:
      Serial.print("Button is On, position: ");
      Serial.println(curr);
      led.turnOn();
      break;
    case -1:
    case 0:
      break;
  }
  delay(10);
}
