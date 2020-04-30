////////////////////////////////////////////////////////////////////////////////
// DigitalIO demo
// A straightforward library to use rotary encoders on Arduinos
////////////////////////////////////////////////////////////////////////////////
// Ultrasonic Sensor - Sonar example
//
// This demo shows how to measure distance with an ultrasonic sensor using the
// digitalSonar class for Uno. The Serial console will display the distance, and
// the onboard LED will turn on if an object is within 12", be off if there is
// an object farther, or blink if the sonar doesn't bounce on anything close.
//
// Ultrasonic sensors are sonars that emit a ping when triggered, and then
// wait for the echo to come back. They return the min bounce time of the ping,
// which can be converted to a distance.
// Sonars supported come with 2 pins, a trigger that expects a 10uSec signal to
// start the sonar, and an echo pin that will return a HIGH square wave matching
// the time it took for the signal to bounce.
//
// The class can be set to use interrupts to hide the ping delay to your loop().
// DIGITAL_IO_SONAR_TIMEOUT can be adjusted to ignore pings that take too long.
// This also limits the max distance monitored. If a ping is lost, it reads 0.
////////////////////////////////////////////////////////////////////////////////
// Example pinout for a HC-SR04 untrasonic sonar board
//
// Arduino Uno        HC-SR04 Ultrasonic Sensor
//         Gnd ------ Gnd
//       Pin 3 ------ Echo
//       Pin 4 ------ Trigger
//          5V ------ Vcc
//
////////////////////////////////////////////////////////////////////////////////
// digitalSonar class template parameters:
//
// Port Name(+)        Only for digitalSonarAvr, letter name of port (A..K)
// Trigger Pin         Pin where the 10usec square wave is output to start
// Echo Pin(*)         Pin where the square wave response is read
// Use Interrupt(*)    Optional: default false. A read() will return a value
//                     read before and trigger a new ping for the next read().
//                     cwIf true, the Echo pin must be one with an interrupt.
// Interrupt Number(*) With digitalSonarAvr provide the interrupt number. It is
//                     the interrupt attached to the Echo pin.
//
// To find the digital pins that support interrupts, and find the interrupt to
// pin pairing, search online for your Arduino board documentation. For Arduino
// Uno, Pin 2 (D2) is tied to interrupt 0, and Pin 3 (D3) to interrupt 3.
// The DIGITAL_IO_SONAR_TIMEOUT macro can be set to change the default timeout
// of 0x7FFF microseconds.
////////////////////////////////////////////////////////////////////////////////

//#define DIGITAL_IO_DEBUG 1
#define DIGITAL_IO_SONAR_TIMEOUT 5000 // divide by 5600 to get the max dist in meter
#include <DigitalIO.hpp>

digitalIo<13, LOW> led;
//digitalIoAvr<B,5, LOW> led; // pin 13

// Uncomment only one of the sonar declarations
digitalSonar<4,3> sonar;               // 2584 byte program storage, 201 bytes RAM
//digitalSonar<4,3,true> sonar;        // 2854 byte program storage, 207 bytes RAM
//digitalSonarAvr<D,4,3> sonar;        // 2504 byte program storage, 201 bytes RAM
//digitalSonarAvr<D,4,3,true,1> sonar; // 2704 byte program storage, 207 bytes RAM

void setup() {
  Serial.begin(9600);
  led.outputMode();
  Serial.println("Start!\n");
}

void loop() {
  // Uncomment only one of the value lines
  uint16_t value = sonar.read(cm); const uint16_t detect = 30; const char* units = " cm";
  //uint16_t value = sonar.read(mm); const uint16_t detect = 300; const char* units = " mm";
  //uint16_t value = sonar.read(sixteenth); const uint16_t detect = 12*16; const char* units = "/16 in";
  //uint16_t value = sonar.read(inch); const uint16_t detect = 12; const char* units = " in";
  //uint16_t value = sonar.read(usec); const uint16_t detect = 1800; const char* units = " usec";

  if (value == 0)
  {
    // Blink LED if sonar is out of range
    led.toggle();
  } else if (value < detect) {
    // Turn on LED if sonar detects something close
    led.turnOn();
  } else {
    // Turn off LED if sonar detects nothing close
    led.turnOff();    
  }

  Serial.print(value);
  Serial.println(units);

  delay(250);
}
