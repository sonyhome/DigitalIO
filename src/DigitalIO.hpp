#ifndef DIGITAL_IO
#define DIGITAL_IO
////////////////////////////////////////////////////////////////////////////////
// @brief
// Copyright (c) 2015-2020 Dan Truong
//
// A Digital IO Library for Arduino
//
// The digitalIo class makes it easy to use buttons and switches on Arduino
//
// The DigitalvrPin has the same functionality but runs faster and compiled
// code is more compact because it uses AVR ports directly instead of the
// Arduino abstraction layer.
//
// The library follow the same syntax as the Serial library class, with a
// begin() method that initializes the I/O pin, and a read() routine that
// returns true if the button is pressed, or debounce which implements a
// simple debounce heuristic to avoid read glitches.
//
// The class is a templace class, which allows you to create an instance
// customized for the pin and type of switch you are using (on HIGH or
// on when LOW).
// The library defines one default switch on PORT B0.
//
// This template design allows the compiler to optimize away anything that
// is not used.
//
////////////////////////////////////////////////////////////////////////////////
// Wiring example for digitalIo<6, HIGH> as input (reading a switch):
//
//           -- Switch
// pin 6 ---o  o---.
//                 |
//                ---
//                /// Gnd
//
// When you press the switch, it conducts grounding pin 6 to LOW.
////////////////////////////////////////////////////////////////////////////////
// Basic input uses:
//
// digitalIo<6, HIGH> button;
// led.inputPullupMode();
// if (button.isOn()) { handlePress();}
// if (!button.isOff()) { handlePress();}
// if (button.changed() == 1) { handlePress();}
// button.changed(); if (button.value() == LOW) { handlePress();}
// if (button.isTransitioned()) { handleKnockSensor();}
//
// Advanced override macros (set them before including the library):
//
// #define DIGITAL_IO_DEBUG true
//   Enables debug mode
// #define DIGITAL_IO_DEBOUNCE_DELAY 50
//   Sets the debounce duration to 50msec
//
////////////////////////////////////////////////////////////////////////////////
// Wiring example for digitalIo<6, LOW> as output (control a low power LED):
//
//          200 Ohm
// pin 6 ---====---.
//                 |
//                 V  23mA LED
//                 -
//                 |
//                ---
//                /// Gnd
//
// When you output HIGH, it turns on the LED
////////////////////////////////////////////////////////////////////////////////
// Basic output uses:
//
// digitalIo<6, LOW> led;
// led.outputMode();
// led.write(HIGH); // Turn on
// led.turnOn(); // Turn on
// led.turnOff(); // Turn off
////////////////////////////////////////////////////////////////////////////////
// Internals
//
// The code is separated into constants, which are fixed values that can be
// modified by the end user if their sensor is too noisy, or the library too
// slow for sensors with very little noise. For example, regular buttons may
// work fine with 5ms debounce, but knock sensors might need 100msec to become
// stable. The 30msec default should be reliable and fast enough for all
// applications.
//
// The digitalIo class is used to access pins with their pin number as they
// are defined by the Arduino ecosystem for your board (usually the number
// printed next to the pin on the board). The class uses template constants
// to declare them and the default state the port is in. This allows the
// compiler to know these are constants, and optimize away all code that
// depend on them. Simple routines are declared inline to hint the compiler to
// not create function calls. The class only stores one byte of information per
// instance/pin.
//
// Note: digitalIo and DigitalPort classes are identical except for their
// constructors anre read/write functions. We don't use a parent class to
// avoid virtualizing read/write which would prevent inlining and slow toe code
////////////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include <util/atomic.h> // ATOMIC_BLOCK(ATOMIC_RESTORESTATE)


////////////////////////////////////////////////////////////////////////////////
/// Constants
/// These shouldn't need to be changed but may be overriden by the user
////////////////////////////////////////////////////////////////////////////////

/// @brief
/// DIGITAL_IO_DEBUG forces the library into debug mode
/// 1: prints settings only
/// 2: prints transitions only
/// 3: prints both
/// 4: only forces pin initialization as input (usually the default, not needed)
#ifdef DIGITAL_IO_DEBUG
#warning DIGITAL_IO_DEBUG mode is set
static_assert(DIGITAL_IO_DEBUG >= 0 && DIGITAL_IO_DEBUG <= 4);
#define DEBUG_PRINT1(text) {if ((DIGITAL_IO_DEBUG & 1) != 0) {Serial.print(text);}}
#define DEBUG_PRINT2(text) {if ((DIGITAL_IO_DEBUG & 2) != 0) {Serial.print(text);}}
#define DEBUG_PRINT3(text) {if ((DIGITAL_IO_DEBUG & 3) != 0) {Serial.print(text);}}
constexpr bool DEBUG_MODE = (DIGITAL_IO_DEBUG > 1);
#else
constexpr bool DEBUG_MODE = false;
#define DEBUG_PRINT1(text)
#define DEBUG_PRINT2(text)
#define DEBUG_PRINT3(text)
#endif

/// @brief
/// If a signal is stable for DIGITAL_IO_DEBOUNCE_DELAY, it is considered
/// debounced and stable. Debounce code will hold up the CPU at least that long.
#ifdef DIGITAL_IO_DEBOUNCE_DELAY
static_assert(DIGITAL_IO_DEBOUNCE_DELAY > 0);
static_assert(DIGITAL_IO_DEBOUNCE_DELAY < 1000);
constexpr uint16_t DEBOUNCE_DELAY = DIGITAL_IO_DEBOUNCE_DELAY;
#else
constexpr uint16_t DEBOUNCE_DELAY = 100;
#endif

/// @brief
/// Internal loop delay in msec (you should not need to change this)
#ifdef DIGITAL_IO_DEBOUNCE_LOOP_DELAY
static_assert(DIGITAL_IO_DEBOUNCE_LOOP_DELAY > 0);
static_assert(DIGITAL_IO_DEBOUNCE_LOOP_DELAY < 256);
constexpr uint8_t LOOP_DELAY =  DIGITAL_IO_DEBOUNCE_LOOP_DELAY;
#else
constexpr uint8_t LOOP_DELAY = 2;
#endif
static_assert(DEBOUNCE_DELAY >= LOOP_DELAY);

/// Number of iterations before a transition is considered stable and debounced
/// defaults to 15, 2msec iterations.
constexpr uint16_t DEBOUNCE_ITERS = (DEBOUNCE_DELAY+LOOP_DELAY-1)/LOOP_DELAY;


////////////////////////////////////////////////////////////////////////////////
// @brief
// CLASS digitalIo should be used when accessing ports via their pin numbers
//
// Data: value (class uses 1 byte)
// Template constants:
// pinNumber: pin number as printed on the Arduino board, for example 6 is PB6
// defaultState:  input state when the button is idle, for example HIGH or LOW
////////////////////////////////////////////////////////////////////////////////
template<uint8_t pinNumber, uint8_t defaultState>
class digitalIo
{
protected:
  // Last value of the pin read (HIGH or LOW)
  uint8_t value = defaultState;

public:

  ////////////////////////////////////////////////////////////
  // @brief
  // Hardware specific config routine
  ////////////////////////////////////////////////////////////
  inline void inputMode(void)
  {
    DEBUG_PRINT1("  pinMode(");
    DEBUG_PRINT1(pinNumber);
    DEBUG_PRINT1(", INPUT);\n");
    pinMode(pinNumber, INPUT);
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Hardware specific config routine
  ////////////////////////////////////////////////////////////
  inline void inputPullupMode(void)
  {
    DEBUG_PRINT1("  pinMode(");
    DEBUG_PRINT1(pinNumber);
    DEBUG_PRINT1(", INPUT_PULLUP);\n");
    pinMode(pinNumber, INPUT_PULLUP);
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Hardware specific config routine
  ////////////////////////////////////////////////////////////
  inline void outputMode(void)
  {
    DEBUG_PRINT1("  pinMode(");
    DEBUG_PRINT1(pinNumber);
    DEBUG_PRINT1(", OUTPUT);\n");
    pinMode(pinNumber, OUTPUT);
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Hardware specific read routine
  ////////////////////////////////////////////////////////////
  inline uint8_t read(void)
  {
    value = digitalRead(pinNumber);
    return value;
  }
  
  ////////////////////////////////////////////////////////////
  // @brief
  // Hardware specific write routine
  // input value can be HIGH or LOW
  ////////////////////////////////////////////////////////////
  inline void write(uint8_t newValue)
  {
    value = newValue;
    digitalWrite(pinNumber, newValue);
  }
  
  ////////////////////////////////////////////////////////////
  // @brief
  // Constructor
  // Initializes the pin as input
  ////////////////////////////////////////////////////////////
  digitalIo()
  {
    // Pin 13 is usually attached to an LED & resistor so can't use the pullup
    if (defaultState == HIGH && pinNumber != 13)
    {
      inputPullupMode();
    }
    else if (DEBUG_MODE)
    {
      // Pins default to inputs so normally we don't have to force it
      inputMode();
    }
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Writes to the output pin the non-default state voltage
  ////////////////////////////////////////////////////////////
  inline void turnOn(void)
  {
    write((defaultState == LOW) ? HIGH : LOW);
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Writes to the output its default rest state voltage level
  ////////////////////////////////////////////////////////////
  inline void turnOff(void)
  {
    write(defaultState);
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Returns true if the pin is receiving a non default signal
  ////////////////////////////////////////////////////////////
  inline bool isOn(void)
  {
    read();
    DEBUG_PRINT3("isOn: ");
    DEBUG_PRINT3(value);
    DEBUG_PRINT3("\n");
    return (value != defaultState);
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Returns true if the pin is at its default level
  ////////////////////////////////////////////////////////////
  inline bool isOff(void)
  {
    read();
    DEBUG_PRINT3("isOff: ");
    DEBUG_PRINT3(value);
    DEBUG_PRINT3("\n");
    return (value == defaultState);
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Returns the last value of the read (HIGH or LOW).
  // This function does not read the pin's current value
  ////////////////////////////////////////////////////////////
  inline uint8_t lastValue(void)
  {
    DEBUG_PRINT2("lastValue: ");
    DEBUG_PRINT2(value);
    DEBUG_PRINT2("\n");
    return value;
  }


  ////////////////////////////////////////////////////////////
  /// @brief (40B code)
  /// Returns true if a brief transition was detected.
  /// This is useful to detect transiant signal like a knock
  ///
  /// The function debounces in case of dirty transitions to
  /// wait for the signal to be stable again.
  /// Debouncing is done by waiting for DEBOUNCE_ITERS
  /// identical samples of the pin value. This means the delay
  /// will be up to DIGITAL_IO_DEBOUNCE_DELAY after the pin
  /// value stabilized.
  /// If it stabilizes after 50msec, then the function will
  /// return after 80msec (with a 30msec default debounce).
  ////////////////////////////////////////////////////////////
  bool triggered(void)
  {
    uint8_t lastValue = value;
    uint8_t i = 0;

    read();
    // No transition detected
    if (value == lastValue)
    {
      return false;
    }

    // Debounce, wait for multiple identical readings
    lastValue = value;
    while (i < DEBOUNCE_ITERS)
    {
      read();
      if (value == lastValue)
      {
        ++i;
      }
      else
      {
        // Unstable signal detected
        lastValue = value;
        DEBUG_PRINT1("triggered(");
        DEBUG_PRINT1(i);
        DEBUG_PRINT1("):");
        DEBUG_PRINT1(value);
        DEBUG_PRINT1(" glitched!\n");
        // Reset debounce countdown
        i = 0;
      }
      delay(LOOP_DELAY);
    }
    return true;
  }

  ////////////////////////////////////////////////////////////
  /// @brief (64B code)
  /// returns a non-zero value on a stable state transitions
  /// This is used to detect state changes like a button press
  ///
  ///  1 if pin was activated (non default level)
  /// -1 if pin returned to default level
  ///  0 if pin has not changed state
  /// The function reads the pin and returns immediately if
  /// the value is unchanged. If it changed it will sample
  /// the value and register the change only if it stays
  /// changed for a DIGITAL_IO_DEBOUNCE_DELAY duration.
  /// Hence this function returns after no more than
  /// DIGITAL_IO_DEBOUNCE_DELAY msec.
  ////////////////////////////////////////////////////////////
  int8_t changed(void) {
    const uint8_t lastValue = value;
    uint8_t i = 0;

    // Simple debounce: detect a transition if the value read
    // stays different from lastValue for every iteration
    do
    {
      read();
      // No transition detected
      if (value == lastValue)
      {
	if (DEBUG_MODE && i != 0)
        {
          // We detected transient noise on the line
          DEBUG_PRINT1("changed(");
          DEBUG_PRINT1(i);
          DEBUG_PRINT1("):");
          DEBUG_PRINT1(value);
          DEBUG_PRINT1(" glitched!\n");
        }
        return 0;
      }
      delay(LOOP_DELAY); 
    }
    while ( i++ < DEBOUNCE_ITERS);

    DEBUG_PRINT2("changed(");
    DEBUG_PRINT2(i);
    DEBUG_PRINT2("):");
    DEBUG_PRINT2(value);
    DEBUG_PRINT2("\n");

    // Transition detected
    if (value == defaultState)
    {
      // Pin went to rest more
      return -1;
    }
    // Pin activated (for example button is pressed)
    return 1;
  }
}; // digitalIo


////////////////////////////////////////////////////////////////////////////////
// @brief
// should be used to use a rotary encoder
//
// I used the following web page to make this class. Notice how this code is
// more compact than theirs because of the DigitalIO class. Furthermore it
// is also protected from glitches coming from noisy signals that may be
// an issue with worn rotary knobs.
// https://howtomechatronics.com/tutorials/arduino/rotary-encoder-works-use-arduino/
//
// Template parameters:
// The rotary encoder uses 3 pins, for the pushbutton switch (Sw),
// clock and data signals (Ck, Dt) also called channel A and B.
// The code supports limiting the rotary encoder to a [min, max] range of values
// and can be monitored by an interrupt handler, or with the main loop polling
// the pins. In the latter case the loop should be as fast as possible (<10ms).
// The user must also provide an int32_t variable that will be used to store
// internal data.
// The reason of this store variable is to support interrupt handlers which need
// a global variable with aknown address at compile time. The class itself can't
// provide that unless it uses a static variable, but then it is shared by all
// the instances of the class. The ISR itself is a static function. By passing
// it in the template we instantiate the class itself and each variable is a
// different type with separate constants: The code is duplicated.
////////////////////////////////////////////////////////////////////////////////
template<uint8_t pinNumberSw, uint8_t pinNumberDt, uint8_t pinNumberCk, int32_t min, int32_t max, bool useInterrupt, volatile int32_t & store>
class digitalRotaryEncoder : public digitalIo<pinNumberSw, HIGH>
{
protected:
  static constexpr int32_t MASK = (int32_t)((~0UL)^1UL);
  static_assert(MASK == 0xFFFFfffe);

  ////////////////////////////////////////////////////////////
  // @brief
  // Hardware specific config routine
  ////////////////////////////////////////////////////////////
  inline void setRotaryMode()
  {
    if (DEBUG_MODE)
    {
      pinMode(pinNumberCk, INPUT);
      pinMode(pinNumberDt, INPUT);
    }
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Hardware specific read routine for clock pin
  ////////////////////////////////////////////////////////////
  static inline uint8_t readCk(void)
  {
    return digitalRead(pinNumberCk);
  }
  
  ////////////////////////////////////////////////////////////
  // @brief
  // Hardware specific read routine for data pin
  ////////////////////////////////////////////////////////////
  static inline uint8_t readDt(void)
  {
    return digitalRead(pinNumberDt);
  }
  
  ////////////////////////////////////////////////////////////
  // @brief
  // Decode the rotary value from the store. The value is
  // shifted left by 1 bit and stored on 31 bits.
  ////////////////////////////////////////////////////////////
  static inline int32_t getValue()
  {
    return (MASK & store)/2;
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Extract the last clock when the rotary value was updated
  // encoded as the lsbit in the store.
  ////////////////////////////////////////////////////////////
  static inline uint8_t getClock()
  {
    return (store & 0x01) ? HIGH : LOW;
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Update the store with the current value and clock bit.
  ////////////////////////////////////////////////////////////
  static inline int32_t setStore(int32_t val, uint8_t ck)
  {
    store = (val * 2) | (ck == HIGH);
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Hardware specific read routine
  ////////////////////////////////////////////////////////////
  static inline int32_t rotaryUpdate()
  {
    int32_t val = getValue();

    // Extract the previous clock from the lowest bit
    const uint8_t prevCk = getClock();
    const uint8_t ck = readCk();
    if (prevCk == ck) {
      // no clock edge or glitch: unchanged value
      return val;
    }

    const uint8_t dt = readDt();
    if (ck != dt)
    {
      // Clockwise
      if (val < max)
      {
        DEBUG_PRINT2("U");
        val++;
      }
    }
    else
    {
      // Counter clockwise
      if (val > min)
      {
        DEBUG_PRINT2("D");
        val--;
      }
    }

    // Reencode the clock on lowest bit and value on 31 bits
    setStore(val, ck);
    DEBUG_PRINT2(val);
    DEBUG_PRINT2("\n");
    return val;
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Interrupt service routine to read rotary encoder.
  // No debounce, we read the values as-is & update counter
  // Note: We use a template to pass the pointer to the class
  // instance value to update with the pin interrupting.
  ////////////////////////////////////////////////////////////
  static void rotaryISR(void)
  {
    (void) rotaryUpdate();
  }

public:
  ////////////////////////////////////////////////////////////
  // @brief
  // Constructor
  // Attaches the interrupt handler
  // If the rotary encoder can be interrupt driven it's better
  // (Uno, Nano, Duo... but not AtTiny)
  // Not all chips can have interrupts on all pins, check the
  // board's documentation.
  ////////////////////////////////////////////////////////////
  digitalRotaryEncoder(void)
  {
    if (DEBUG_MODE)
    {
      // IO defaults to input, in debug mode force it.
      setRotaryMode();
    }

    if (useInterrupt)
    {
      // Trigger interrupt on both rising and falling edges of the clock
      // Options: RISING, FALLING, CHANGE
      attachInterrupt(digitalPinToInterrupt(pinNumberCk), rotaryISR, CHANGE);
    }
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Track and return the rotary encoder current value.
  // If the encoder is tracked with an interrupt it just
  // reports it, if not it reads the encoder to detect changes
  // In that case the value is stored only on 31 bits, and
  // this routine must be called repetively fast (no call to
  // delay()) to poll the rotary encoder reliably.
  ////////////////////////////////////////////////////////////
  int8_t rotaryRead()
  {
    if (!useInterrupt)
    {
      return rotaryUpdate();
    }
    else
    {
      int32_t val;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // Read value without risk of ISR updating it at same time
        // Keep this code block to a minimum size
        val = getValue();
      }
      return val;
    }
  }
}; // digitalRotaryEncoder



////////////////////////////////////////////////////////////////////////////////
/// digitalIoAvr is only available on AVR artchitecture. For other architectures
/// use digitalIo.
///
/// The difference is we use macros to access directly the AVR I/O registers.
/// The benefit is it compiles more compact and is faster because it bypasses
/// a pretty heavy code layer of the Arduino library.
////////////////////////////////////////////////////////////////////////////////
#ifdef ARDUINO_ARCH_AVR

#include <Arduino.h>

/// @brief
/// Catch ports that don't exist for a given chip:
/// Define a DEFAULT_DDR and DEFAULT_PORT because we can't use NOT_A_PORT to
/// catch writes to a non-existing port (compilation fails on lvalue).
/// Hopefully using a non-existing port will fail when reading the PIN which
/// will map to NOT_A_PORT.
#if defined(DDRA)
#define DEFAULT_DDR  DDRA
#define DEFAULT_PORT PORTA
#define DEFAULT_PIN  PORTA
#elif defined(DDRB)
#define DEFAULT_DDR  DDRB
#define DEFAULT_PORT PORTB
#define DEFAULT_PIN  PORTB
#elif defined(DDRC)
#define DEFAULT_DDR  DDRC
#define DEFAULT_PORT PORTC
#define DEFAULT_PIN  PORTC
#elif defined(DDRD)
#define DEFAULT_DDR  DDRD
#define DEFAULT_PORT PORTD
#define DEFAULT_PIN  PORTD
#elif defined(DDRE)
#define DEFAULT_DDR  DDRE
#define DEFAULT_PORT PORTE
#define DEFAULT_PIN  PORTE
#elif defined(DDRF)
#define DEFAULT_DDR  DDRF
#define DEFAULT_PORT PORTF
#define DEFAULT_PIN  PORTF
#endif

/// Override the non-existing ports
#ifndef DDRA
#define DDRA  DEFAULT_DDR
#define PORTA DEFAULT_PORT
#define PINA  DEFAULT_PIN
#endif
#ifndef DDRB
#define DDRB  DEFAULT_DDR
#define PORTB DEFAULT_PORT
#define PINB  DEFAULT_PIN
#endif
#ifndef DDRC
#define DDRC  DEFAULT_DDR
#define PORTC DEFAULT_PORT
#define PINC  DEFAULT_PIN
#endif
#ifndef DDRD
#define DDRD  DEFAULT_DDR
#define PORTD DEFAULT_PORT
#define PIND  DEFAULT_PIN
#endif
#ifndef DDRE
#define DDRE  DEFAULT_DDR
#define PORTE DEFAULT_PORT
#define PINE  DEFAULT_PIN
#endif
#ifndef DDRF
#define DDRF  DEFAULT_DDR
#define PORTF DEFAULT_PORT
#define PINF  DEFAULT_PIN
#endif

/// @brief
/// Macros to access the port Data Direction control Register's address
/// The macro uses the char to pick the named registers DDRA ... DDRF
#define AVR_DDR(c) ((c == 'A') ? DDRA : (c == 'B') ? DDRB : (c == 'C') ? DDRC : \
                     (c == 'D') ? DDRD : (c == 'E') ? DDRE : (c == 'F') ? DDRF : \
                     DEFAULT_DDR)

/// @brief
/// Set port pin as an input by forcing DDR bit to zero
#define AVR_SET_AS_IN(port, pin)  AVR_DDR(port) = AVR_DDR(port) & (0xFFU ^ (0x01U << (pin)))
#define AVR_SET_AS_OUT(port, pin) AVR_DDR(port) = AVR_DDR(port) | 0x01U << (pin)

#define AVR_PORT(c) ((c == 'A') ? PORTA : (c == 'B') ? PORTB : (c == 'C') ? PORTC : \
                     (c == 'D') ? PORTD : (c == 'E') ? PORTE : (c == 'F') ? PORTF : \
                     DEFAULT_PORT)
/// @brief
/// Macro to access the Port Data Register
/// Set output high/pull-up or low/high-impedance
#define AVR_SET_LOW(port, pin)  AVR_PORT(port) = AVR_PORT(port) & 0xFFU ^ (0x01U << (pin))
#define AVR_SET_HIGH(port, pin) AVR_PORT(port) = AVR_PORT(port) | (0x01U << (pin))
#define AVR_WRITE(port, pin, value) if (value) { AVR_SET_HIGH(port, pin);} else { AVR_SET_LOW(port, pin);}

/// @brief
/// Macro to access the Input Pin Address Regiter
#define AVR_PIN(c) ((c == 'A') ? PINA : (c == 'B') ? PINB : (c == 'C') ? PINC : \
                    (c == 'D') ? PIND : (c == 'E') ? PINE : (c == 'F') ? PINF : \
                    DEFAULT_PIN)
#define AVR_READ(port, pin) (((AVR_PIN(port) & (0x01U << (pin))) == 0) ? LOW : HIGH)


////////////////////////////////////////////////////////////////////////////////
// @brief
// CLASS digitalIoAvr should be used when accessing ports via their pin numbers
//
// Data: value (class uses 1 byte)
// Template constants:
// pinNumber: pin number as printed on the Arduino board, for example 6 is PB6
// defaultState:  input state when the button is idle, for example HIGH or LOW
////////////////////////////////////////////////////////////////////////////////
template<char portName, uint8_t portPin, uint8_t defaultState>
class digitalIoAvr
{
public:

  // Last value of the pin read (HIGH or LOW)
  uint8_t value = defaultState;

  ////////////////////////////////////////////////////////////
  // @brief
  // Hardware specific config routine
  ////////////////////////////////////////////////////////////
  inline void inputMode(void)
  {
    DEBUG_PRINT1("  AVR_SET_AS_IN(");
    DEBUG_PRINT1(portName);
    DEBUG_PRINT1(",");
    DEBUG_PRINT1(portPin);
    DEBUG_PRINT1(");\n");
    AVR_SET_AS_IN(portName, portPin);
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Hardware specific config routine
  ////////////////////////////////////////////////////////////
  inline void inputPullupMode(void)
  {
    inputMode();
    DEBUG_PRINT1("  AVR_SET_HIGH(");
    DEBUG_PRINT1(portName);
    DEBUG_PRINT1(",");
    DEBUG_PRINT1(portPin);
    DEBUG_PRINT1(");\n");
    AVR_SET_HIGH(portName, portPin);
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Hardware specific config routine
  ////////////////////////////////////////////////////////////
  inline void outputMode(void)
  {
    DEBUG_PRINT1("  AVR_SET_AS_OUT(");
    DEBUG_PRINT1(portName);
    DEBUG_PRINT1(",");
    DEBUG_PRINT1(portPin);
    DEBUG_PRINT1(");\n");
    AVR_SET_AS_OUT(portName, portPin);
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Hardware specific read routine
  ////////////////////////////////////////////////////////////
  inline uint8_t read(void)
  {
    value = AVR_READ(portName, portPin);
    return value;
  }
  
  ////////////////////////////////////////////////////////////
  // @brief
  // Hardware specific write routine
  // input value can be HIGH or LOW
  ////////////////////////////////////////////////////////////
  inline void write(uint8_t newValue)
  {
    value = newValue;
    AVR_WRITE(portName, portPin, (bool) newValue);
  }
  
  ////////////////////////////////////////////////////////////
  // @brief
  // Constructor
  // Initializes the pin as input
  ////////////////////////////////////////////////////////////
  digitalIoAvr()
  {
    if (defaultState == HIGH)
    {
      inputPullupMode();
    }
    else if (DEBUG_MODE)
    {
      // Pins default to inputs so normally we don't have to force it
      inputMode();
    }
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Writes to the output pin the non-default state voltage
  ////////////////////////////////////////////////////////////
  inline void turnOn(void)
  {
    write((defaultState == LOW) ? HIGH : LOW);
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Writes to the output its default rest state voltage level
  ////////////////////////////////////////////////////////////
  inline void turnOff(void)
  {
    write(defaultState);
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Returns true if the pin is receiving a non default signal
  ////////////////////////////////////////////////////////////
  inline bool isOn(void)
  {
    read();
    DEBUG_PRINT3("isOn: ");
    DEBUG_PRINT3(value);
    DEBUG_PRINT3("\n");
    return (value != defaultState);
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Returns true if the pin is at its default level
  ////////////////////////////////////////////////////////////
  inline bool isOff(void)
  {
    read();
    DEBUG_PRINT3("isOff: ");
    DEBUG_PRINT3(value);
    DEBUG_PRINT3("\n");
    return (value == defaultState);
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Returns the last value of the read (HIGH or LOW).
  // This function does not read the pin's current value
  ////////////////////////////////////////////////////////////
  inline uint8_t lastValue(void)
  {
    DEBUG_PRINT2("lastValue: ");
    DEBUG_PRINT2(value);
    DEBUG_PRINT2("\n");
    return value;
  }


  ////////////////////////////////////////////////////////////
  /// @brief (40B code)
  /// Returns true if a brief transition was detected.
  /// This is useful to detect transiant signal like a knock
  ///
  /// The function debounces in case of dirty transitions to
  /// wait for the signal to be stable again.
  /// Debouncing is done by waiting for DEBOUNCE_ITERS
  /// identical samples of the pin value. This means the delay
  /// will be up to DIGITAL_IO_DEBOUNCE_DELAY after the pin
  /// value stabilized.
  /// If it stabilizes after 50msec, then the function will
  /// return after 80msec (with a 30msec default debounce).
  ////////////////////////////////////////////////////////////
  bool triggered(void)
  {
    uint8_t lastValue = value;
    uint8_t i = 0;

    read();
    // No transition detected
    if (value == lastValue)
    {
      return false;
    }

    // Debounce, wait for multiple identical readings
    lastValue = value;
    while (i < DEBOUNCE_ITERS)
    {
      read();
      if (value == lastValue)
      {
        ++i;
      }
      else
      {
        // Unstable signal detected
        lastValue = value;
        DEBUG_PRINT1("triggered(");
        DEBUG_PRINT1(i);
        DEBUG_PRINT1("):");
        DEBUG_PRINT1(value);
        DEBUG_PRINT1(" glitched!\n");
        // Reset debounce countdown
        i = 0;
      }
      delay(LOOP_DELAY);
    }
    return true;
  }

  ////////////////////////////////////////////////////////////
  /// @brief (64B code)
  /// returns a non-zero value on a stable state transitions
  /// This is used to detect state changes like a button press
  ///
  ///  1 if pin was activated (non default level)
  /// -1 if pin returned to default level
  ///  0 if pin has not changed state
  /// The function reads the pin and returns immediately if
  /// the value is unchanged. If it changed it will sample
  /// the value and register the change only if it stays
  /// changed for a DIGITAL_IO_DEBOUNCE_DELAY duration.
  /// Hence this function returns after no more than
  /// DIGITAL_IO_DEBOUNCE_DELAY msec.
  ////////////////////////////////////////////////////////////
  int8_t changed(void) {
    const uint8_t lastValue = value;
    uint8_t i = 0;

    // Simple debounce: detect a transition if the value read
    // stays different from lastValue for every iteration
    do
    {
      read();
      // No transition detected
      if (value == lastValue)
      {
	if (DEBUG_MODE && i != 0)
        {
          // We detected transient noise on the line
          DEBUG_PRINT1("changed(");
          DEBUG_PRINT1(i);
          DEBUG_PRINT1("):");
          DEBUG_PRINT1(value);
          DEBUG_PRINT1(" glitched!\n");
        }
        return 0;
      }
      delay(LOOP_DELAY); 
    }
    while ( i++ < DEBOUNCE_ITERS);

    DEBUG_PRINT2("changed(");
    DEBUG_PRINT2(i);
    DEBUG_PRINT2("):");
    DEBUG_PRINT2(value);
    DEBUG_PRINT2("\n");

    // Transition detected
    if (value == defaultState)
    {
      // Pin went to rest more
      return -1;
    }
    // Pin activated (for example button is pressed)
    return 1;
  }
}; // digitalIoAvr


////////////////////////////////////////////////////////////////////////////////
// @brief
// should be used to use a rotary encoder
//
// I used the following web page to make this class. Notice how this code is
// more compact than theirs because of the DigitalIO class. Furthermore it
// is also protected from glitches coming from noisy signals that may be
// an issue with worn rotary knobs.
// https://howtomechatronics.com/tutorials/arduino/rotary-encoder-works-use-arduino/
//
// For AVR, instead of specifying a pin you specify a port letter and pin.
// 
// To use the AVR version you need to provide the interruptVector number.
// On Arduino UNO:
//   INT0_vect for D2 (pin 2)
//   INT1_vect for D3 (pin 3)
// http://www.gammon.com.au/interrupts
////////////////////////////////////////////////////////////////////////////////
template<char portNameSw, uint8_t portPinSw,
         char portNameDt, uint8_t portPinDt,
         char portNameCk, uint8_t portPinCk,
         int32_t min, int32_t max,
         bool useInterrupt, uint8_t interruptVector,
         volatile int32_t & store>
class digitalRotaryEncoderAvr : public digitalIoAvr<portNameSw, portPinSw, HIGH>
{
protected:
  static constexpr int32_t MASK = (int32_t)((~0UL)^1UL);
  static_assert(MASK == 0xFFFFfffe);

  ////////////////////////////////////////////////////////////
  // @brief
  // Hardware specific config routine
  ////////////////////////////////////////////////////////////
  inline void setRotaryMode()
  {
    if (DEBUG_MODE)
    {
      AVR_SET_AS_IN(portNameDt, portPinDt);
      AVR_SET_AS_IN(portNameCk, portPinCk);
    }
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Hardware specific read routine for clock pin
  ////////////////////////////////////////////////////////////
  static inline uint8_t readCk(void)
  {
    return AVR_READ(portNameCk, portPinCk);
  }
  
  ////////////////////////////////////////////////////////////
  // @brief
  // Hardware specific read routine for data pin
  ////////////////////////////////////////////////////////////
  static inline uint8_t readDt(void)
  {
    return AVR_READ(portNameDt, portPinDt);
  }
  
  ////////////////////////////////////////////////////////////
  // @brief
  // Decode the rotary value from the store. The value is
  // shifted left by 1 bit and stored on 31 bits.
  ////////////////////////////////////////////////////////////
  static inline int32_t getValue()
  {
    return (MASK & store)/2;
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Extract the last clock when the rotary value was updated
  // encoded as the lsbit in the store.
  ////////////////////////////////////////////////////////////
  static inline uint8_t getClock()
  {
    return (store & 0x01) ? HIGH : LOW;
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Update the store with the current value and clock bit.
  ////////////////////////////////////////////////////////////
  static inline int32_t setStore(int32_t val, uint8_t ck)
  {
    store = (val * 2) | (ck == HIGH);
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Hardware specific read routine
  ////////////////////////////////////////////////////////////
  static inline int32_t rotaryUpdate()
  {
    int32_t val = getValue();

    // Extract the previous clock from the lowest bit
    const uint8_t prevCk = getClock();
    const uint8_t ck = readCk();
    if (prevCk == ck) {
      // no clock edge or glitch: unchanged value
      return val;
    }

    const uint8_t dt = readDt();
    if (ck != dt)
    {
      // Clockwise
      if (val < max)
      {
        DEBUG_PRINT2("U");
        val++;
      }
    }
    else
    {
      // Counter clockwise
      if (val > min)
      {
        DEBUG_PRINT2("D");
        val--;
      }
    }

    // Reencode the clock on lowest bit and value on 31 bits
    setStore(val, ck);
    DEBUG_PRINT2(val);
    DEBUG_PRINT2("\n");
    return val;
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Interrupt service routine to read rotary encoder.
  // No debounce, we read the values as-is & update counter
  // Note: We use a template to pass the pointer to the class
  // instance value to update with the pin interrupting.
  ////////////////////////////////////////////////////////////
  static void rotaryISR(void)
  {
    (void) rotaryUpdate();
  }

public:
  ////////////////////////////////////////////////////////////
  // @brief
  // Constructor
  // Attaches the interrupt handler
  // If the rotary encoder can be interrupt driven it's better
  // (Uno, Nano, Duo... but not AtTiny)
  // Not all chips can have interrupts on all pins, check the
  // board's documentation.
  ////////////////////////////////////////////////////////////
  digitalRotaryEncoderAvr(void)
  {
    if (DEBUG_MODE)
    {
      // IO defaults to input, in debug mode force it.
      setRotaryMode();
    }

    if (useInterrupt)
    {
      // Trigger interrupt on both rising and falling edges of the clock
      // Options: RISING, FALLING, CHANGE
      attachInterrupt(interruptVector, rotaryISR, CHANGE);
    }
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Track and return the rotary encoder current value.
  // If the encoder is tracked with an interrupt it just
  // reports it, if not it reads the encoder to detect changes
  // In that case the value is stored only on 31 bits, and
  // this routine must be called repetively fast (no call to
  // delay()) to poll the rotary encoder reliably.
  ////////////////////////////////////////////////////////////
  int8_t rotaryRead()
  {
    if (!useInterrupt)
    {
      return rotaryUpdate();
    }
    else
    {
      int32_t val;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // Read value without risk of ISR updating it at same time
        // Keep this code block to a minimum size
        val = getValue();
      }
      return val;
    }
  }
}; // digitalRotaryEncoder
#endif // ARDUINO_ARCH_AVR

#endif // DIGITAL_IO
