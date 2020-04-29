#ifndef DIGITAL_IO
#define DIGITAL_IO
////////////////////////////////////////////////////////////////////////////////
// @brief
// Copyright (c) 2015-2020 Dan Truong
//
// A Digital IO Library for Arduino
//
// The digitalIo library aims at simplifying use of devices and sensors attached
// to the digital I/O pins. The main class is for simple single pin devices.
// More complex devices have their own classes (rotary encoders and sonars).
//
// The Avr classes have the same functionality but compile smaller and run
// faster as they directly reverence the AVR ports and their pins, bypassing the
// Arduino abstraction layer.
//
// The library defines template classes declaring port parameters as constant 
// to help code optimization.
//
// Macros can override some built-in constants to help tune the library to the
// use case (for example reduce the max debounce delay to 2ms for a good button
// or extend it to 100ms for a noisy knock sensor).
//
// See the header of each class in DigitalIo.h and _DigitalIo.h for details.
////////////////////////////////////////////////////////////////////////////////
// Example:
// Wiring for digitalIo<6, HIGH> as input (reading a switch).
// When you press the switch, it conducts grounding pin 6 to LOW.
//
//           -- Switch
// pin 6 ---o  o---.
//                 |
//                ---
//                /// Gnd
//
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
//
// Code turning on the LED with the push-button:
//
// digitalIo<7, LOW> led;
// digitalIo<6, HIGH> button;
// setup() {
//   led.outputMode();
//   button.inputPullupMode();
// }
// loop() {
//   if (button.isOn()) { led.turnOn();}
//   if (!button.read() == LOW) { led.write(LOW);}
//   if (button.isTransitioned()) { handleKnock();}
//   if (button.changed() == 1) { handlePressed();}
// }
////////////////////////////////////////////////////////////////////////////////
#include <stdint.h>

#ifdef ARDUINO_ARCH_AVR // AVR
//#include <Arduino.h>
#include <util/atomic.h> // ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
#endif // ARDUINO_ARCH_AVR

// Optimize code
#pragma GCC optimize("-O2")



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/// @brief
/// DIGITAL_IO_DEBUG forces the library into debug mode
/// 1: prints settings and raw I/O activity
/// 2: prints transitions only
/// 3: prints both
/// 4: only forces pin initialization as input (usually the default, not needed)
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
#ifdef DIGITAL_IO_DEBUG
#warning DIGITAL_IO_DEBUG mode is set
static_assert(DIGITAL_IO_DEBUG >= 0 && DIGITAL_IO_DEBUG <= 4,
              "Use debug modes 0 to 4");
#define DEBUG_PRINT1(text) {if ((DIGITAL_IO_DEBUG & 1) != 0) {Serial.print(text);}}
#define DEBUG_PRINT2(text) {if ((DIGITAL_IO_DEBUG & 2) != 0) {Serial.print(text);}}
#define DEBUG_PRINT3(text) {if ((DIGITAL_IO_DEBUG & 3) != 0) {Serial.print(text);}}
constexpr bool DEBUG_MODE = (DIGITAL_IO_DEBUG > 1);
#else // undefined DIGITAL_IO_DEBUG
constexpr bool DEBUG_MODE = false;
#define DEBUG_PRINT1(text)
#define DEBUG_PRINT2(text)
#define DEBUG_PRINT3(text)
#endif // DIGITAL_IO_DEBUG



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// AVR port manipulation
// Controls code optimizations related to direct port I/O
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
#ifdef ARDUINO_ARCH_AVR

#define DIGITAL_IO_AVR_DEFAULT 1
#define DIGITAL_IO_AVR_OPTIMIZED 2

////////////////////////////////////////////////////////////////////////////////
// For AVR boards, the non-AVR specific classes using arduino pins can be
// optimized by setting
// DIGITAL_IO_AVR_PINMODE = DIGITAL_IO_AVR_OPTIMIZED
// and can be made more standard/compatible with
// DIGITAL_IO_AVR_PINMODE = DIGITAL_IO_AVR_DEFAULT (default)
////////////////////////////////////////////////////////////////////////////////
#ifndef DIGITAL_IO_AVR_PINMODE
#define DIGITAL_IO_AVR_PINMODE DIGITAL_IO_AVR_DEFAULT
#endif // DIGITAL_IO_AVR_PINMODE

////////////////////////////////////////////////////////////////////////////////
// For AVR boards, the AVR specific classes using AVR port names and port pins
// can optimized by setting
// DIGITAL_IO_AVR_PORTMODE = DIGITAL_IO_AVR_OPTIMIZED (default)
// and can be made more standard/compatible with
// DIGITAL_IO_AVR_PORTMODE = DIGITAL_IO_AVR_DEFAULT
////////////////////////////////////////////////////////////////////////////////
#ifndef DIGITAL_IO_AVR_PORTMODE
#define DIGITAL_IO_AVR_PORTMODE DIGITAL_IO_AVR_OPTIMIZED
#endif // DIGITAL_IO_AVR_PORTMODE

////////////////////////////////////////////////////////////////////////////////
// AVR port name constants.
// These are used to convert a port name (for example D) to the port register
// (for example PIND to read or PORTD to write).
////////////////////////////////////////////////////////////////////////////////
enum avrPorts
{
  A = 1,
  B = 2, // AtTiny
  C = 3,
  D = 4, // Uno
  E = 5,
  F = 6,
  G = 7, // Mega has NUM_DIGITAL_PINS == 70
  H = 8,
  J = 10,
  K = 11,
  L = 12,
};

////////////////////////////////////////////////////////////////////////////////
// We can't use NOT_A_PORT as we can't write to it. It's hard to write a macro
// if we can't allow NOT_A_PORT = value (that code would be optimized out).
// So instead all invalid ports are redirected to a known always valid port.
// @warn Using an invalid port on an AVR board will automatically replace the
// bad port name to port B! This might cause unsuspected results. We use port B
// because port A is usually an analog port, not a digital port.
//
// Arduino will define PIN* and PORT* that exist for the chosen board. For all
// non existing ones the following defines them just for the purpose of not
// having compilation errors.
////////////////////////////////////////////////////////////////////////////////
#define BAD_PIN_REGISTER  PINB
#define BAD_PORT_REGISTER PORTB

////////////////////////////////////////////////////////////////////////////////
// Macros to define PIN* ports that do not exist for an AVR board as NOT_A_PORT
////////////////////////////////////////////////////////////////////////////////
#ifndef PINA
#define PINA BAD_PIN_REGISTER
#endif
#ifndef PINB
#define PINB BAD_PIN_REGISTER
#endif
#ifndef PINB
#define PINB BAD_PIN_REGISTER
#endif
#ifndef PINC
#define PINC BAD_PIN_REGISTER
#endif
#ifndef PIND
#define PIND BAD_PIN_REGISTER
#endif
#ifndef PINE
#define PINE BAD_PIN_REGISTER
#endif
#ifndef PINF
#define PINF BAD_PIN_REGISTER
#endif
#ifndef PING
#define PING BAD_PIN_REGISTER
#endif
#ifndef PINH
#define PINH BAD_PIN_REGISTER
#endif
#ifndef PINJ
#define PINJ BAD_PIN_REGISTER
#endif
#ifndef PINK
#define PINK BAD_PIN_REGISTER
#endif
#ifndef PINL
#define PINL BAD_PIN_REGISTER
#endif

////////////////////////////////////////////////////////////////////////////////
// Macros to define PORT* ports that do not exist for an AVR board as NOT_A_PORT
////////////////////////////////////////////////////////////////////////////////
#ifndef PORTA
#define PORTA BAD_PORT_REGISTER
#endif
#ifndef PORTB
#define PORTB BAD_PORT_REGISTER
#endif
#ifndef PORTB
#define PORTB BAD_PORT_REGISTER
#endif
#ifndef PORTC
#define PORTC BAD_PORT_REGISTER
#endif
#ifndef PORTD
#define PORTD BAD_PORT_REGISTER
#endif
#ifndef PORTE
#define PORTE BAD_PORT_REGISTER
#endif
#ifndef PORTF
#define PORTF BAD_PORT_REGISTER
#endif
#ifndef PORTG
#define PORTG BAD_PORT_REGISTER
#endif
#ifndef PORTH
#define PORTH BAD_PORT_REGISTER
#endif
#ifndef PORTJ
#define PORTJ BAD_PORT_REGISTER
#endif
#ifndef PORTK
#define PORTK BAD_PORT_REGISTER
#endif
#ifndef PORTL
#define PORTL BAD_PORT_REGISTER
#endif

////////////////////////////////////////////////////////////////////////////////
// Apply a command defined in APPLY_AVR_ACTION_TO_PORTS to every AVR port/pin.
// At compile time the port&pin are constants, all but the relevant port code is
// optimized away by the compiler, to generate fast code. This macro needs all
// the port registers to be defined, hence the previous macros.
// Use the ## operator to convert the port name to a register name as needed.
//
// Code usage:
// #define AVR_PIN_ACTION(p,b) PORT ## p = 1<<b
// APPLY_AVR_ACTION_TO_PORTS(port,pin);
// #undef AVR_PIN_ACTION
//
// Assuming port D is used the switch statement at compile time is optimized to
// just be: "PORTD = 1 << portBit;", because we call this where D is constant.
////////////////////////////////////////////////////////////////////////////////
#define APPLY_AVR_ACTION_TO_PORTS(portName, portBit) \
  switch(portName) {                                 \
    case A: AVR_PIN_ACTION(A, portBit); break;       \
    case B: AVR_PIN_ACTION(B, portBit); break;       \
    case C: AVR_PIN_ACTION(C, portBit); break;       \
    case D: AVR_PIN_ACTION(D, portBit); break;       \
    case E: AVR_PIN_ACTION(E, portBit); break;       \
    case F: AVR_PIN_ACTION(F, portBit); break;       \
    case G: AVR_PIN_ACTION(G, portBit); break;       \
    case H: AVR_PIN_ACTION(H, portBit); break;       \
    case J: AVR_PIN_ACTION(J, portBit); break;       \
    case K: AVR_PIN_ACTION(K, portBit); break;       \
    case L: AVR_PIN_ACTION(L, portBit); break;       \
    default: break; }

#endif // ARDUINO_ARCH_AVR



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Interrupt wrapper routines hack
// The classes can attach interrupt handlers (ISR) to the DigitalIO pins they
// are attached to.
// For AVR we want to find the interrupt number for a port/pin. It is not
// possible to get directly from the Arduino infrastructure without using the
// PROGMEM tables which is slow. Hence the pinToIrqAvr does its best to find
// the interrupt for port/pins of the most common AVR boards.
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// @brief
// Wrapper routines to find the interrupt attached to a digital pin. If the pin
// has none, returns NOT_AN_INTERRUPT (-1 or 255).
// This routine just encapsulates the digitalPinToInterrupt() built-in macro.
////////////////////////////////////////////////////////////////////////////////
inline uint8_t pinToIrq(uint8_t pin)
{
  return digitalPinToInterrupt(pin);
}

////////////////////////////////////////////////////////////////////////////////
// @brief
// Wrapper routines to find the interrupt attached to a digital pin. If the pin
// has none, returns NOT_AN_INTERRUPT (-1 or 255).
// This is a hack cobbled up from some variants/*/pins_arduino.h, meant only for
// AVR boards 
// @todo: Implement a slow fail-safe method using PROGMEM tables
////////////////////////////////////////////////////////////////////////////////
#ifdef ARDUINO_ARCH_AVR
inline uint8_t pinToIrqAvr(uint8_t port, uint8_t pin)
{
  uint8_t irq = NOT_AN_INTERRUPT;

  #if NUM_DIGITAL_PINS < 20
  if (port == B && pin == 2)
  {
    irq = 0; // AtTiny*
  }
  #elif NUM_DIGITAL_PINS < 70
  if (port == D)
  {
    switch(pin)
    {
      // Uno
      case 0: irq = 0; break;
      case 1: irq = 1; break;
      // Leonardo
      case 2: irq = 2; break;
      case 3: irq = 3; break;
      case 6: irq = 4; break;
    }
  }
  #else // MEGA
  if (port == D)
  {
    if (pin < 3) { irq = pin+2; }
  }
  else if (port == E)
  {
    if (pin == 4) { irq = 0; }
    if (pin == 5) { irq = 1; }
  }
  #endif // NUM_DIGITAL_PINS
  DEBUG_PRINT1(irq);
  DEBUG_PRINT1("= pinToIrqAvr(");
  DEBUG_PRINT1(port);
  DEBUG_PRINT1(",");
  DEBUG_PRINT1(pin);
  DEBUG_PRINT1(");");
  return irq;
}
#endif // ARDUINO_ARCH_AVR



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// @brief (for AVR boards only)
// Internal class (do not use directly)
// CLASS digitalIoRaw defines RAW I/O primitives and should not be used as-is.
// Supports
// * AVR via pinId
// * AVR via portName, portId
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
#ifdef ARDUINO_ARCH_AVR
class digitalIoRaw
{
public:
  ////////////////////////////////////////////////////////////
  // @brief
  // AVR specific config routine via pinId
  ////////////////////////////////////////////////////////////
  static inline void inputModeRaw(uint8_t pinId)
  {
    // We can't optimize out the pinId function calls because they
    // dereference config tables stored in PROGMEM. Re-hardcoding
    // digitalPinTo* is board dependend so we won't do that here.
    // It also seems like it saves memory to call them in the functions
    // instead of the constructor so we do that (2476B vs 2608B SRAM,
    // 336B vs 349B DRAM). digitalIoAvr uses 2250B/337B because the
    // PORT and BIT don't need to be converted with a PROGMEM table.
    // Savings are 5% and 14% respectively!

    #if DIGITAL_IO_AVR_PINMODE != DIGITAL_IO_AVR_DEFAULT
      const uint8_t portBit = digitalPinToBitMask(pinId);
      const uint8_t portName = digitalPinToPort(pinId);
      DEBUG_PRINT1("inputModeRaw(");
      DEBUG_PRINT1(pinId);
      DEBUG_PRINT1(") => portBit:");
      DEBUG_PRINT1(portBit);
      DEBUG_PRINT1(", portName:");
      DEBUG_PRINT1(portName);
      DEBUG_PRINT1(");\n");
      *portModeRegister(portName) &= 0xFFU ^ portBit; // I
    #else
      DEBUG_PRINT1("pinmode(");
      DEBUG_PRINT1(pinId);
      DEBUG_PRINT1(", INPUT)\n");
      pinMode(pinId, INPUT);
    #endif
  }
  ////////////////////////////////////////////////////////////
  // AVR specific config routine via portName, portBit
  ////////////////////////////////////////////////////////////
  static inline void inputModeRaw(uint8_t portName, uint8_t portPin)
  {
    *portModeRegister(portName) &= 0xFFU ^ (1U << portPin); // I
    DEBUG_PRINT1("inputModeRaw => ");
    DEBUG_PRINT1(*portModeRegister(portName));
    DEBUG_PRINT1("= *portModeRegister(");
    DEBUG_PRINT1(portName);
    DEBUG_PRINT1(") &= ");
    DEBUG_PRINT1(0xFFU ^ (1U << portPin));
    DEBUG_PRINT1("; IN\n");
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // AVR specific config routine via pinId
  ////////////////////////////////////////////////////////////
  static inline void inputPullupModeRaw(uint8_t pinId)
  {
    #if DIGITAL_IO_AVR_PINMODE != DIGITAL_IO_AVR_DEFAULT
      const uint8_t portBit = digitalPinToBitMask(pinId);
      const uint8_t portName = digitalPinToPort(pinId);
      DEBUG_PRINT1("inputPullupModeRaw(");
      DEBUG_PRINT1(pinId);
      DEBUG_PRINT1(") => portBit:");
      DEBUG_PRINT1(portBit);
      DEBUG_PRINT1(", portName:");
      DEBUG_PRINT1(portName);
      DEBUG_PRINT1(");\n");
      *portModeRegister(portName) &= 0xFFU ^ portBit; // I
      *portOutputRegister(portName) |= portBit; // H
    #else
      DEBUG_PRINT1("pinmode(");
      DEBUG_PRINT1(pinId);
      DEBUG_PRINT1(", INPUT_PULLUP)\n");
      pinMode(pinId, INPUT_PULLUP);
    #endif
  }
  ////////////////////////////////////////////////////////////
  // AVR specific config routine via portName, portBit
  ////////////////////////////////////////////////////////////
  static inline void inputPullupModeRaw(uint8_t portName, uint8_t portPin)
  {
    inputModeRaw(portName, portPin);
    *portOutputRegister(portName) |= (1U <<portPin); // H
    DEBUG_PRINT1("inputPullupModeRaw => ");
    DEBUG_PRINT1(*portOutputRegister(portName));
    DEBUG_PRINT1("= *portOutputRegister(");
    DEBUG_PRINT1(portName);
    DEBUG_PRINT1(") |= ");
    DEBUG_PRINT1(1U << portPin);
    DEBUG_PRINT1("; HIGH\n");
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // AVR specific config routine via pinId
  ////////////////////////////////////////////////////////////
  static inline void outputModeRaw(uint8_t pinId)
  {
    #if DIGITAL_IO_AVR_PINMODE != DIGITAL_IO_AVR_DEFAULT
      const uint8_t portBit = digitalPinToBitMask(pinId);
      const uint8_t portName = digitalPinToPort(pinId);
      DEBUG_PRINT1("outputModeRaw(");
      DEBUG_PRINT1(pinId);
      DEBUG_PRINT1(") => portBit:");
      DEBUG_PRINT1(portBit);
      DEBUG_PRINT1(", portName:");
      DEBUG_PRINT1(portName);
      DEBUG_PRINT1(");\n");
      *portModeRegister(portName) |= portBit; // O
    #else
      DEBUG_PRINT1("pinmode(");
      DEBUG_PRINT1(pinId);
      DEBUG_PRINT1(", OUTPUT)\n");
      pinMode(pinId, OUTPUT);
    #endif
  }
  ////////////////////////////////////////////////////////////
  // AVR specific config routine via portName, portBit
  ////////////////////////////////////////////////////////////
  static inline void outputModeRaw(uint8_t portName, uint8_t portPin)
  {
    *portModeRegister(portName) |= (1U << portPin); // O
    DEBUG_PRINT1("outputModeRaw => ");
    DEBUG_PRINT1(*portModeRegister(portName));
    DEBUG_PRINT1("= *portModeRegister(");
    DEBUG_PRINT1(portName);
    DEBUG_PRINT1(") |= ");
    DEBUG_PRINT1(1U << portPin);
    DEBUG_PRINT1("; OUT\n");
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // AVR specific read routine via pinId
  ////////////////////////////////////////////////////////////
  static inline uint8_t readRaw(uint8_t pinId)
  {
    uint8_t val;
    DEBUG_PRINT1("readRaw => ");
    #if DIGITAL_IO_AVR_PINMODE != DIGITAL_IO_AVR_DEFAULT
      // Slightly smaller code 2682 vs 2754 and likely faster
      const uint8_t portBit = digitalPinToBitMask(pinId);
      const uint8_t portName = digitalPinToPort(pinId);
      val = *portInputRegister(portName);
      DEBUG_PRINT1(val);
      DEBUG_PRINT1(" ");
      val = ((val & portBit) != 0) ? HIGH : LOW;
      DEBUG_PRINT1(val);
      DEBUG_PRINT1(" = ((*portInputRegister(");
      DEBUG_PRINT1(portName);
      DEBUG_PRINT1(") & ");
      DEBUG_PRINT1(portBit);
      DEBUG_PRINT1(" != 0)\n");
      return val;
    #else
      val = digitalRead(pinId);
      DEBUG_PRINT1(val);
      DEBUG_PRINT1(" = digitalRead(");
      DEBUG_PRINT1(pinId);
      DEBUG_PRINT1(")\n");
      return val;
    #endif // DIGITAL_IO_AVR_PINMODE
  }
  ////////////////////////////////////////////////////////////
  // AVR specific read routine via portName, portBit
  // If portBit, portName are computed from pinId then it is
  // more efficient to use portInputRegister (dereferences a
  // table in PROGMEM).
  // If they are given directly from constants then the switch
  // statement compiles away to a single line with constants
  // accessing  the port directly.
  ////////////////////////////////////////////////////////////
  static inline uint8_t readRaw(uint8_t portName, uint8_t portPin)
  {
    uint8_t val;
    DEBUG_PRINT1("readRaw => ");
    #if DIGITAL_IO_AVR_PORTMODE == DIGITAL_IO_AVR_OPTIMIZED
      // For all ports, perform AVR_PIN_ACTION, which is a read of the pin
      #define AVR_PIN_ACTION(portName, portPin) val = PIN ## portName
      APPLY_AVR_ACTION_TO_PORTS(portName, portPin);
      #undef AVR_PIN_ACTION
      DEBUG_PRINT1(val);
      DEBUG_PRINT1("= ");
      val = ((val & (1U << portPin)) != 0) ? HIGH : LOW;
      DEBUG_PRINT1(val);
      DEBUG_PRINT1("= (PIN");
      DEBUG_PRINT1(portName);
      DEBUG_PRINT1(" & ");
      DEBUG_PRINT1(1U << portPin);
      DEBUG_PRINT1(" != 0); READ\n");
    #else
      val = *portInputRegister(portName);
      DEBUG_PRINT1(val);
      DEBUG_PRINT1("= ");
      val = ((val & (1U << portPin)) != 0) ? HIGH : LOW;
      DEBUG_PRINT1(val);
      DEBUG_PRINT1("= (*portInputRegister(");
      DEBUG_PRINT1(portName);
      DEBUG_PRINT1(") & ");
      DEBUG_PRINT1(1U << portPin);
      DEBUG_PRINT1(") != 0); READ\n");
    #endif // DIGITAL_IO_AVR_PORTMODE
    return val;
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // AVR specific write routine via pinId
  // input value can be HIGH or LOW
  ////////////////////////////////////////////////////////////
  static inline void writeRaw(uint8_t pinId, uint8_t newValue)
  {
    const uint8_t portName = digitalPinToPort(pinId);
    const uint8_t portBit = digitalPinToBitMask(pinId);
    #if DIGITAL_IO_AVR_PINMODE != DIGITAL_IO_AVR_DEFAULT
      if (newValue == LOW)
      {
        *portOutputRegister(portName) &= 0xFFU ^ portBit; // L
        DEBUG_PRINT1("portOutputRegister(");
        DEBUG_PRINT1(portName);
        DEBUG_PRINT1(") &= 0xFFU ^ ");
        DEBUG_PRINT1(portBit);
        DEBUG_PRINT1("; pinId:");
        DEBUG_PRINT1(pinId);
        DEBUG_PRINT1(", newValue:LOW\n");
      }
      else // if (newValue == HIGH)
      {
        *portOutputRegister(portName) |= portBit; // H
        DEBUG_PRINT1("portOutputRegister(");
        DEBUG_PRINT1(portName);
        DEBUG_PRINT1(") |= ");
        DEBUG_PRINT1(portBit);
        DEBUG_PRINT1("; pinId:");
        DEBUG_PRINT1(pinId);
        DEBUG_PRINT1(", newValue:HIGH\n");
      }
    #else
      digitalWrite(pinId, newValue);
      DEBUG_PRINT1("digitalWrite(");
      DEBUG_PRINT1(pinId);
      DEBUG_PRINT1(", ");
      DEBUG_PRINT1(newValue);
      DEBUG_PRINT1(");\n");
    #endif // DIGITAL_IO_AVR_PINMODE
  }
  ////////////////////////////////////////////////////////////
  // AVR specific write routine via portName, portBit
  ////////////////////////////////////////////////////////////
  static inline void writeRaw(uint8_t portName, uint8_t portPin, uint8_t newValue)
  {
    uint8_t val;
    DEBUG_PRINT1("writeRaw(");
    DEBUG_PRINT1(portName);
    DEBUG_PRINT1(",");
    DEBUG_PRINT1(portPin);
    DEBUG_PRINT1(",");
    DEBUG_PRINT1(newValue);
    DEBUG_PRINT1(") => ");
    if (newValue == LOW)
    {
      #if DIGITAL_IO_AVR_PORTMODE == DIGITAL_IO_AVR_OPTIMIZED
        #define AVR_PIN_ACTION(portName, portPin) PORT ## portName &= 0xFFU ^ (1U << portPin); val = PIN ## portName
        APPLY_AVR_ACTION_TO_PORTS(portName, portPin);
        #undef AVR_PIN_ACTION
        DEBUG_PRINT1(val);
        DEBUG_PRINT1("= PORT");
        DEBUG_PRINT1(portName);
        DEBUG_PRINT1(" &= ");
      #else
        *portOutputRegister(portName) &= 0xFFU ^ (1U << portPin); // L
        DEBUG_PRINT1(*portOutputRegister(portName));
        DEBUG_PRINT1(" = *portOutputRegister(");
        DEBUG_PRINT1(portName);
        DEBUG_PRINT1(") &= ");
      #endif // DIGITAL_IO_AVR_PORTMODE
      DEBUG_PRINT1(0xFFU ^ (1U << portPin));
      DEBUG_PRINT1("; newValue:LOW\n");
    }
    else // if (newValue == HIGH)
    {
      #if DIGITAL_IO_AVR_PORTMODE == DIGITAL_IO_AVR_OPTIMIZED
        #define AVR_PIN_ACTION(portName, portPin) PORT ## portName |= (1U << portPin); val = PIN ## portName
        APPLY_AVR_ACTION_TO_PORTS(portName, portPin);
        #undef AVR_PIN_ACTION
        DEBUG_PRINT1(val);
        DEBUG_PRINT1("= PORT");
        DEBUG_PRINT1(portName);
        DEBUG_PRINT1(" |= ");
      #else
        *portOutputRegister(portName) |= (1U << portPin); // H
        DEBUG_PRINT1(*portOutputRegister(portName));
        DEBUG_PRINT1("= *portOutputRegister(");
        DEBUG_PRINT1(portName);
        DEBUG_PRINT1(") |= ");
      #endif // DIGITAL_IO_AVR_MODE
      DEBUG_PRINT1(1U << portPin);
      DEBUG_PRINT1("; newValue:HIGH\n");
    }
  }
};
#endif



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// @brief (for all boards except AVR)
// Internal class (do not use directly)
// CLASS digitalIoRaw defines RAW I/O primitives and should not be used as-is.
// Supports
// * non-AVR
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
#if !ARDUINO_ARCH_AVR
class digitalIoRaw
{
public:

  ////////////////////////////////////////////////////////////
  // @brief
  // Hardware specific config routine
  ////////////////////////////////////////////////////////////
  static inline void inputModeRaw(uint8_t pinId)
  {
    DEBUG_PRINT1("  pinMode(");
    DEBUG_PRINT1(pinId);
    DEBUG_PRINT1(", INPUT);\n");
    pinMode(pinId, INPUT);
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Hardware specific config routine
  ////////////////////////////////////////////////////////////
  static inline void inputPullupModeRaw(uint8_t pinId)
  {
    DEBUG_PRINT1("  pinMode(");
    DEBUG_PRINT1(pinId);
    DEBUG_PRINT1(", INPUT_PULLUP);\n");
    pinMode(pinId, INPUT_PULLUP);
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Hardware specific config routine
  ////////////////////////////////////////////////////////////
  static inline void outputModeRaw(uint8_t pinId)
  {
    DEBUG_PRINT1("  pinMode(");
    DEBUG_PRINT1(pinId);
    DEBUG_PRINT1(", OUTPUT);\n");
    pinMode(pinId, OUTPUT);
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Hardware specific read routine
  ////////////////////////////////////////////////////////////
  static inline uint8_t readRaw(uint8_t pinId)
  {
    const uint8_t val = digitalRead(pinId);
    DEBUG_PRINT1(val);
    DEBUG_PRINT1("= digitalRead(");
    DEBUG_PRINT1(pinId);
    DEBUG_PRINT1(");\n");
    return val;
  }
  
  ////////////////////////////////////////////////////////////
  // @brief
  // Hardware specific write routine
  // input value can be HIGH or LOW
  ////////////////////////////////////////////////////////////
  static inline void writeRaw(uint8_t pinId, uint8_t val)
  {
    digitalWrite(pinId, val);
    DEBUG_PRINT1(digitalRead(pinId));
    DEBUG_PRINT1("= digitalWrite(");
    DEBUG_PRINT1(pinId);
    DEBUG_PRINT1(", ");
    DEBUG_PRINT1(val);
    DEBUG_PRINT1(");\n");
  }
};
#endif // !ARDUINO_ARCH_AVR



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// digitalIo / digitalIoAvr macro definitions
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/// @brief
/// If a signal is stable for DIGITAL_IO_DEBOUNCE_DELAY, it is considered
/// debounced and stable. Debounce code will hold up the CPU at least that long.
////////////////////////////////////////////////////////////////////////////////
#ifdef DIGITAL_IO_DEBOUNCE_DELAY
static_assert(DIGITAL_IO_DEBOUNCE_DELAY > 0,
  "Debounce duration can't be zero");
static_assert(DIGITAL_IO_DEBOUNCE_DELAY < 1000,
  "Debounce duration is excessively long");
constexpr uint16_t DEBOUNCE_DELAY = DIGITAL_IO_DEBOUNCE_DELAY;
#else
constexpr uint16_t DEBOUNCE_DELAY = 100;
#endif

////////////////////////////////////////////////////////////////////////////////
/// @brief
/// Internal loop delay in msec (you should not need to change this)
////////////////////////////////////////////////////////////////////////////////
#ifdef DIGITAL_IO_DEBOUNCE_LOOP_DELAY
static_assert(DIGITAL_IO_DEBOUNCE_LOOP_DELAY > 0,
  "Debounce loop duration can't be zero");
static_assert(DIGITAL_IO_DEBOUNCE_LOOP_DELAY < 256,
  "Debounce loop duration must be short");
constexpr uint8_t LOOP_DELAY =  DIGITAL_IO_DEBOUNCE_LOOP_DELAY;
#else
constexpr uint8_t LOOP_DELAY = 2;
#endif
static_assert(DEBOUNCE_DELAY >= LOOP_DELAY,
  "Debounce time must be greater than the debounce inner loop period");

////////////////////////////////////////////////////////////////////////////////
/// @brief
/// Number of iterations before a transition is considered stable and debounced
/// defaults to 15, 2msec iterations.
////////////////////////////////////////////////////////////////////////////////
constexpr uint16_t DEBOUNCE_ITERS = (DEBOUNCE_DELAY+LOOP_DELAY-1)/LOOP_DELAY;



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// digitalUltrasonicSensor macro definitions
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// @brief
// Timeout limits the distance measured (returns 0 if out of range)
// currently set to 0x8000/56 = 5.8m (max 11m)
////////////////////////////////////////////////////////////////////////////////
#ifndef DIGITAL_IO_SONAR_TIMEOUT
#define DIGITAL_IO_SONAR_TIMEOUT 0x8000
#endif
static_assert(DIGITAL_IO_SONAR_TIMEOUT < 0xFFFF,
  "DIGITAL_IO_SONAR_TIMEOUT must be less than 65535usec (1129cm)");

////////////////////////////////////////////////////////////////////////////////
// @brief
// Units of measurements supported by the class
////////////////////////////////////////////////////////////////////////////////
enum digitalIoDistanceUnits: uint8_t
{
  usec = 0,      // microseconds for round trip
  cm = 1,
  mm = 2,
  inch = 3,
  tenth = 4,     // 1/10th inch
  sixteenth = 5, // 1/16th inch
};



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Instantiation of all digital* and digital*Avr classes
//
// The main classes are defined in _DigitalIO.hpp. Their definition uses macros
// to create 2 versions of the classes while maintaining only one code
// base for all the classes.
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// @brief
// ALL refers to digitalIo classes and AVR to digitalIoAvr classes
////////////////////////////////////////////////////////////////////////////////
#define _DIGITAL_IO_ALL 1
#define _DIGITAL_IO_AVR 2

////////////////////////////////////////////////////////////////////////////////
// Declare digitalIo<pinNumber,defaultState...> classes
// These classes work for any Arduino board but have overhead to convert the pin
// number to the port register and pin (both more code and slower).
////////////////////////////////////////////////////////////////////////////////
#define _DIGITAL_IO_VERSION _DIGITAL_IO_ALL
#include <_DigitalIO.hpp>
#undef _DIGITAL_IO_VERSION

////////////////////////////////////////////////////////////////////////////////
// Declare digitalIoAvr<portName,pinNumber,defaultState...> classes
// These classes specific to AVR boards are more compact and faster, because it
// avoids PROGMEM tables used to convert pin numbers to ports.
// The classes take an extra port name template parameter (A..F) and the pin
// numbers correspond only to that port (0..7).
// This extension is implemented with macro magic to pass the port name and
// postfix the class names with "Avr".
////////////////////////////////////////////////////////////////////////////////
#ifdef ARDUINO_ARCH_AVR
#define _DIGITAL_IO_VERSION _DIGITAL_IO_AVR
#include <_DigitalIO.hpp>
#undef _DIGITAL_IO_VERSION
#endif // ARDUINO_ARCH_AVR

#endif // DIGITAL_IO
