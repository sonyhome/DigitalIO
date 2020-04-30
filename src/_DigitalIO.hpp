////////////////////////////////////////////////////////////////////////////////
// @brief
// Copyright (c) 2015-2020 Dan Truong
//
// A Digital IO Library for Arduino
//
// See DigitalIO.hpp
////////////////////////////////////////////////////////////////////////////////
// @todo
// digitalIo:
// - Interrupt handler on change for pin record UP and DOWN and IRQ_STATE
//   -> flipped UP: if STATE == IRQ_STATE != LAST_STATE && transition == UP then
//      assume it's a clean transition no glitch. Else debounce or ignore.
//   -> triggered if UP ^ DOWN and...
// digitalEncoder:
// - Fix direct calls to digitalIoRaw::foo(), maybe inherit protected then
//   inherit?
// HC-SR04 untrasonic sonar:
// - a loop to check port until a timeout.
// - AVR interrupt doesn't work (pin mapping bug?)
// - read: For non-AVR we don't know how to set up an interrupt on a digital pin
////////////////////////////////////////////////////////////////////////////////
//#include <wiring_private.h> // countPulseASM

////////////////////////////////////////////////////////////////////////////////
// @brief
// Macros to define _PORT_NAME_TEMPLATE, _PORT_NAME, _CLASS_NAME based on the
// _DIGITAL_IO_VERSION definition, used to instantiate default classes using the
// Arduino standard pins, and Avr classes using port name and port pins.
// DigitalIO.hpp defines _DIGITAL_IO_VERSION before including _DigitalIO.hpp
// _CLASS_NAME(foo) converts to "foo" or "fooAvr".
// _PORT_NAME converts to nothing or "portName,", used to call functions like
// foo(pin) or fooAvr(portName, pin)
// _PORT_NAME_TEMPLATE converts to nothing or "uint8_t portName,", to create
// classes like template< uint8_t pinNumber>foo
// and template<uint8_t portName, uint8_t pinNumber>foo
////////////////////////////////////////////////////////////////////////////////
#undef _PORT_NAME_TEMPLATE
#undef _PORT_NAME
#undef _CLASS_NAME

#if _DIGITAL_IO_VERSION == _DIGITAL_IO_AVR
#define _PORT_NAME_TEMPLATE uint8_t portName,
#define _PORT_NAME portName,
#define _CLASS_NAME(name) name ## Avr
#else
#define _PORT_NAME_TEMPLATE
#define _PORT_NAME
#define _CLASS_NAME(name) name
#endif

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// @brief
// should support any device connected to a single Digital I/O pin
//
// Hardware:
// Any input device that has on/off state, transient trigger signal (a spike),
// mechanical (button, knock sensor) or solid state (hall sensor).
// Any digital output device that has a HIGH or LOW state (diode, relay).
//
// Class:
// digitalIo/digitalIoAvr
//
// portName: (Avr version) is the port name (A...F). Both pins are on same port
// pinNumber: Either Arduino pin # or port pin # (0..7)
// defaultState: The rest state of the pin for the device connected(HIGH or LOW)
//          to track or set activation (isOn(), turnOn(), etc.)
// Methods:
// inputMode: Set the pin as an input with floating pin (default).
// inputPullupMode: Set the pin as an input with a pull-up resistor (~ 20kOhm)
// outputMode: Set the pin as an output.
//
// write: Write the value to the pin
// turnOff: Write the defaultState to the pin
// turnOn: Write the non-defaultState to the pin
//
// read: Read the pin value (raw, unprocessed value, HIGH or LOW)
// ifOff: The pin's level is at its defaultState
// isOn: The pin's level is not at the defaultState
// triggered: true if transition was detected (temporary spike or level change),
//          and returns once the level on the pin is stable (debouncing), to
//          avoid glitches. Used for knock sensors which can be very noisy.
//          The function will detect a spike only if it happens as the function
//          is running. The spike must be longer than the loop() body.
//          (see Technical note)
// flipped: 1: the pin's level changed from defaultState to non-default.
//          -1: the pin's level changed and reverted to its defaultState.
//          0: the pin's level did not change (can be either state).
//          The signal is debounced and only reports a transition if the signal
//          stays stable in a new level. It filters out spikes. Used for buttons
//          (see Technical note)
//
// DEFINES:
// DIGITAL_IO_DEBOUNCE_DELAY: For flipped and triggered methods, the amount of
//          time the pin's level must not change to be considered stable. You
//          may increase it for very noisy sensors (knock sensors) or lower
//          it to reduce latency (good mechanical switches). Default 100ms.
//
// Example for an Arduino Uno:
// digitalIo<7, HIGH> pushButton;
// digitalIoAvr<D,7, HIGH> pushButton;
// digitalIoAvr<B,5, LOW> led;
//
// led.outputMode();
// pushButton.inputPullupMode();
//
// uint8_t state = pushButton.flipped();
// if (state == 1){Serial.println("ON");}
// else if (state == -1){Serial.println("OFF");}
// if (pushButton.isOn()){led.turnOn();} else {led.turnOff();}
//
// Technical note:
// Each instance of the class uses one byte to hold the last known state.
// triggered and flipped rely on the previous state read to know if the state 
// of the port changed. The previous state will be the one monitored by either
// of them. 
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
template<_PORT_NAME_TEMPLATE uint8_t pinNumber, uint8_t defaultState>
class _CLASS_NAME(digitalIo) : public digitalIoRaw
{
protected:
  static_assert(defaultState == HIGH || defaultState == LOW,
               "Error: defaultState template parameter should be HIGH or LOW.");

  // Last value of the pin read (HIGH or LOW)
  uint8_t value = defaultState;

public:
  ////////////////////////////////////////////////////////////
  // @brief
  // Wrappers for raw hardware routines: pinNumber -> pinId.
  ////////////////////////////////////////////////////////////
  static inline void inputMode(void) { inputModeRaw(_PORT_NAME pinNumber); }
  static inline void inputPullupMode(void) { inputPullupModeRaw(_PORT_NAME pinNumber); }
  static inline void outputMode(void) { outputModeRaw(_PORT_NAME pinNumber);}
  static inline uint8_t read(void) { return readRaw(_PORT_NAME pinNumber); }
  static inline void write(uint8_t newValue) {writeRaw(_PORT_NAME pinNumber, newValue); }
 
  ////////////////////////////////////////////////////////////
  // @brief
  // Constructor
  // Initializes the pin as input
  ////////////////////////////////////////////////////////////
  _CLASS_NAME(digitalIo)()
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
  static inline void turnOn(void)
  {
    constexpr uint8_t notDefaultState =
                      (defaultState == LOW) ? HIGH : LOW;
    write(notDefaultState);
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Writes to the output its default rest state voltage level
  ////////////////////////////////////////////////////////////
  static inline void turnOff(void)
  {
    write(defaultState);
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Writes to the output its opposite state
  ////////////////////////////////////////////////////////////
  static inline void toggle(void)
  {
    const uint8_t flipState = (read() == LOW) ? HIGH : LOW;
    write(flipState);
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Returns true if the pin is receiving a non default signal
  ////////////////////////////////////////////////////////////
  static inline bool isOn(void)
  {
    const uint8_t val = read();
    DEBUG_PRINT3("isOn: ");
    DEBUG_PRINT3(val);
    DEBUG_PRINT3("\n");
    return (val != defaultState);
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Returns true if the pin is at its default level
  ////////////////////////////////////////////////////////////
  static inline bool isOff(void)
  {
    const uint8_t val = read();
    DEBUG_PRINT3("isOff: ");
    DEBUG_PRINT3(val);
    DEBUG_PRINT3("\n");
    return (val == defaultState);
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Returns the last value of the read (HIGH or LOW).
  // This function does not read the pin's current value
  ////////////////////////////////////////////////////////////
  inline uint8_t readLast(void)
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

    value = read();
    // No transition detected
    if (value == lastValue)
    {
      return false;
    }

    // Debounce, wait for multiple identical readings
    lastValue = value;
    while (i < DEBOUNCE_ITERS)
    {
      value = read();
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
  int8_t flipped(void) {
    const uint8_t lastValue = value;
    uint8_t i = 0;

    // Simple debounce: detect a transition if the value read
    // stays different from lastValue for every iteration
    do
    {
      value = read();
      // No transition detected
      if (value == lastValue)
      {
	if (DEBUG_MODE && i != 0)
        {
          // We detected transient noise on the line
          DEBUG_PRINT1("flipped(");
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
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// @brief
// Rotary encoder support class.
//
// Hardware:
// A rotary encoder has 2 channels (A, B or data, clock). The direction of the
// rotation is detected by the sequence of transitions on A & B. This must be
// detected as it happens (time sensitive). A third channel is for the button.
//
// https://howtomechatronics.com/tutorials/arduino/rotary-encoder-works-use-arduino/
// I used the above to write this class, the code is further optimized and
// protected against glitchesi from a noisy signal.
//
// Class:
// digitalEncoder, digitalEncoderAvr
//
// portName: (Avr version) is the port name (A...F). Both pins are on same port
// pinNumberSw: Either Arduino pin # or port pin # (0..7), the pin connected
//           to the push-button switch.
// pinNumberDt: Either Arduino pin # or port pin # (0..7), the pin connected
//           to the data pin
// pinNumberCk: Either Arduino pin # or port pin # (0..7), the pin connected
//           to the clock pin.
// min:      The lowest value the encoder counts down to (limit -32767)
// max:      The highest value the encoder counts up to (limit 32767)
// interruptFlag: If false, use sequential mode, readEncoder() will poll channel
//           A & B to detect changes in the encoder. In that case the function
//           should be called frequently (<10ms) to avoid missing changes. If
//           the knob is rotated too fast some changes will be missed or might
//           be read as an opposite movement. If true, interrupts will be used
//           to detect the state and readEncoder() will just return the current
//           value. The interrupt handler will track the state in time with low
//           overhead so it should be more reliable.
//           Read your documentation, pinNumberEcho must be a pin that has an
//           interrupt vector attached to (for Uno: pin 3 & 4, aka D0 and D1)
// 
// Methods:
// readEncoder(): Read the current value of the rotary encoder.
// read(), isOn(), turnOn(), etc. All the digitalIo methods for the button.
//
// Example:
// digitalEncoder<5,4,3,-64,64,false> encoder;
// digitalEncoder<5,4,3,-64,64,true> encoder;
// digitalEncoderAvr<D,2,1,0,-64,64,false> encoder;
// digitalEncoderAvr<D,2,1,0,-64,64,true> encoder;
//
// if (encoder.isOn()) {Serial.println(encoder.readEncoder();}
//
// Technical note:
// Each instance of the class uses 2 bytes to store the encoder value, and one
// byte to track the push button state.
// When enabled, the interrupt handler monitors HIGH or LOW state changes of the
// clock and waits to see an UP or DOWN transition on the data to detect the
// direction of movement.
// The whole class is static variables and methods, as needed to implement an
// interrupt handler. However the class template constants ensure that each
// instance created by the user will create new functions and variables for
// each trio of pins monitored.
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
template<_PORT_NAME_TEMPLATE uint8_t pinNumberSw, uint8_t pinNumberDt,
         uint8_t pinNumberCk, int16_t min, int16_t max,
         bool interruptFlag = false, uint8_t irqNumber = NOT_AN_INTERRUPT>
class _CLASS_NAME(digitalEncoder) :
      public _CLASS_NAME(digitalIo)<_PORT_NAME pinNumberSw, HIGH>
{
protected:
  static volatile int16_t store;

  ////////////////////////////////////////////////////////////
  // @brief
  // Hardware specific read routine for clock pin
  ////////////////////////////////////////////////////////////
  static inline uint8_t readCk(void)
  {
    return digitalIoRaw::readRaw(_PORT_NAME pinNumberCk);
  }
  
  ////////////////////////////////////////////////////////////
  // @brief
  // Hardware specific read routine for data pin
  ////////////////////////////////////////////////////////////
  static inline uint8_t readDt(void)
  {
    return digitalIoRaw::readRaw(_PORT_NAME pinNumberDt);
  }
  
  ////////////////////////////////////////////////////////////
  // @brief
  // Decode the rotary value from the store.
  // The value is shifted left by 1 bit and stored on 31 bits.
  ////////////////////////////////////////////////////////////
  static inline int16_t getValue()
  {
    return store>>1;
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
  static inline void setStore(int16_t val, uint8_t ck)
  {
    DEBUG_PRINT1("va:");
    DEBUG_PRINT1(val);
    DEBUG_PRINT1(" ck:");
    DEBUG_PRINT1(ck);
    DEBUG_PRINT1(" st:");
    DEBUG_PRINT1(store);
    DEBUG_PRINT1(" > ");
    store = (val * 2) | (ck == HIGH);
    DEBUG_PRINT1(store);
    DEBUG_PRINT1(" STORE\n");
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Hardware specific read routine
  ////////////////////////////////////////////////////////////
  static inline int16_t encoderUpdate()
  {
    int16_t val = getValue();

    // Extract the previous clock from the lowest bit
    const uint8_t prevCk = getClock();
    const uint8_t ck = readCk();

    if (prevCk == ck) {
      // no clock edge or glitch: unchanged value
      return val;
    }
    const uint8_t dt = readDt();

    DEBUG_PRINT1("va:");
    DEBUG_PRINT1(val);
    DEBUG_PRINT1(" pck:");
    DEBUG_PRINT1(prevCk);
    DEBUG_PRINT1(" ck:");
    DEBUG_PRINT1(ck);
    DEBUG_PRINT1(" dt:");
    DEBUG_PRINT1(dt);

    if (ck != dt)
    {
      // Clockwise
      if (val < max)
      {
        DEBUG_PRINT2(" U");
        val++;
      }
    }
    else
    {
      // Counter clockwise
      if (val > min)
      {
        DEBUG_PRINT2(" D");
        val--;
      }
    }
    DEBUG_PRINT1("> ");
    DEBUG_PRINT2(val);
    DEBUG_PRINT1(" encoderUpdate");
    DEBUG_PRINT2("\n");

    // Reencode the clock on lowest bit and value on 31 bits
    setStore(val, ck);
    return val;
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Interrupt service routine to read rotary encoder.
  // No debounce, we read the values as-is & update counter
  ////////////////////////////////////////////////////////////
  static void encoderISR(void)
  {
    DEBUG_PRINT1("ISR: ");
    (void) encoderUpdate();
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
  _CLASS_NAME(digitalEncoder)(void)
  {
    if (DEBUG_MODE)
    {
      // IO defaults to input, in debug mode force it.
      digitalIoRaw::inputModeRaw(_PORT_NAME pinNumberCk);
      digitalIoRaw::inputModeRaw(_PORT_NAME pinNumberDt);
    }

    if (interruptFlag)
    {
      const uint8_t irq = 
                    _CLASS_NAME(pinToIrq)(_PORT_NAME pinNumberCk, irqNumber);
      if (_CLASS_NAME(useInterrupt)(_PORT_NAME pinNumberCk, interruptFlag, irqNumber))
      {
        // Options: Trigger interrupt on RISING, FALLING, CHANGE edge
        attachInterrupt(irq, encoderISR, CHANGE);
      }
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
  static inline int8_t readEncoder()
  {
    if (! _CLASS_NAME(useInterrupt)(_PORT_NAME pinNumberCk, interruptFlag, irqNumber))
    {
      return encoderUpdate();
    }

    int16_t val = 0;
    #ifdef AVR // For AVR only 1B reads are atomic, so prevent IRQs when reading value
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    #endif
    {
      // Read value without risk of ISR updating it at same time
      // Keep this code block to a minimum size
      val = getValue();
    }
    return val;
  }
}; // digitalEncoder

// store: Static variable declarations for digitalEncoder
template<_PORT_NAME_TEMPLATE uint8_t pinNumberSw, uint8_t pinNumberDt,
          uint8_t pinNumberCk, int16_t min, int16_t max, bool interruptFlag, uint8_t irqNumber>
volatile int16_t _CLASS_NAME(digitalEncoder) <_PORT_NAME pinNumberSw,
                 pinNumberDt, pinNumberCk, min, max, interruptFlag, irqNumber>::store;


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// @brief
// HC-SR04 Ultrasonic Digital Sonar support class.
// Hardware:
// After receiving a 10ms HIGH trigger signal, it emits 8 sonic bursts at 40 KHz
// for reliability. After the last one, it sets the echo pin HIGH until the
// bursts come back (spec: 150 µS to 25 mS, 2cm to 4m).
// The duration of the echo corresponds to 2 * distance / MACH1.
// 60ms rest between measurement is ideal.
//
// Class:
// digitalSonar or digitalSonarAvr
//
// The properties are defined via template constants.
// portName: (Avr version) is the port name (A...F). Both pins are on same port
// pinNumberTrig: Either Arduino pin # or port pin # (0..7), the pin connected
//           to the trigger of the sensor (written to)
// pinNumberEcho: Either Arduino pin # or port pin # (0..7), the pin connected
//           to the echo return of the sensor (read)
// interruptFlag: If false, use sequential mode, read() will wait for the echo
//           to come back. If true, use interrupt mode, read() will read the
//           previous value, initialize trigger, and set-up the interupt handler
//           to get the next echo value. The read() does not wait for the echo
//           to come back, making the code more responsive to do other tasks.
//           Read your documentation, pinNumberEcho must be a pin that has an
//           interrupt vector attached to (for Uno: pin 3 & 4, aka D0 and D1)
//
// Methods:
// uint16_t = read(units); Returns the distance detected (0 if out of range).
//           metric:   cm (default), mm
//           imperial: inch, tenth, sixteenth
//           time:     usec
// 
// DEFINES:
// DIGITAL_IO_SONAR_TIMEOUT: If the ping takes longer than that duration, stop
//           monitoring and return 0 (default 0x8000 aka 5.6m max range).
//
// Example:
// digitalSonar<7, 2, true> sonar;
// digitalSonar<7, 6, false> sonar;
// digitalSonarAvr<D, 5, 0, true> sonar;
// digitalSonarAvr<D, 5, 4, false> sonar;
//
// Serial.println(sonar.read(inch));
//
// Technical note:
// The class is built with template constants and static variables and functions
// because that allows instantiating a new class for every sensor variable added
// that will be on a different set of pins, AND allow the class to define a new
// ISR interrupt handler routine (it has to be a static function with no
// parameters).
// All methods are static because pins are defined by the template constants.
// Therefore every template instance should handle only one set of pins, no
// sharing and a new function will be created for ech sensor variable.
// Most functions are very small and inlineable so program overhead is minimal.
//
// The class uses microseconds encoded on 16bit to limit the footprint to 4B
// per instance. This gives the class a max range of 11.7m with a precision of
// 17.86 micrometer.
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
template<_PORT_NAME_TEMPLATE uint8_t pinNumberTrig, uint8_t pinNumberEcho,
         bool interruptFlag = false,  uint8_t irqNumber = NOT_AN_INTERRUPT>
class _CLASS_NAME(digitalSonar) : digitalIoRaw
{
protected:
  // Last ping recorded
  static uint16_t sPingUsec;
  // Last time the sensor was triggered.
  static uint16_t sPrevTimeUsec;

  ////////////////////////////////////////////////////////////
  // @brief Accessor routine
  ////////////////////////////////////////////////////////////
  static inline uint16_t getValue()
  {
    return sPingUsec;
  }

  ////////////////////////////////////////////////////////////
  // @brief Accessor routine
  ////////////////////////////////////////////////////////////
  static inline uint16_t setValue(uint16_t val)
  {
    sPingUsec = val;
    return val;
  }

  ////////////////////////////////////////////////////////////
  // @brief Accessor routine
  ////////////////////////////////////////////////////////////
  static inline uint16_t getPrev(void)
  {
    return sPrevTimeUsec;
  }

  ////////////////////////////////////////////////////////////
  // @brief Accessor routine
  ////////////////////////////////////////////////////////////
  static inline void setPrev(uint16_t prev)
  {
    sPrevTimeUsec = prev;
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Compute the pulse duration as a time delta.
  // Used as Interrupt service routine to read rotary encoder.
  ////////////////////////////////////////////////////////////
  static void computeValue(void)
  {
    uint16_t now = micros(); // Timer doesn't change inside ISR
    uint16_t old = getPrev();
    setValue(now - old);
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // A 10ms signal on the trigger pin initates a sonar ping
  ////////////////////////////////////////////////////////////
  static inline void triggerPing(void)
  {
    writeRaw(_PORT_NAME pinNumberTrig, HIGH);
    delayMicroseconds(10); // Implemented with noops, not timer
    writeRaw(_PORT_NAME pinNumberTrig, LOW);

    // wait for the pulse to start
    // @todo empirical measurement on Uno: loop is 95 cycles
    uint32_t loops =
       microsecondsToClockCycles(DIGITAL_IO_SONAR_TIMEOUT)/95;
    while  (readRaw(_PORT_NAME pinNumberEcho) == LOW) {
      if (loops-- == 0) break;
    }
    setPrev(micros()); // Set time for begining of response
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Wait for ping to come back
  ////////////////////////////////////////////////////////////
  static uint16_t waitForValue(void)
  {
    // wait for the pulse to start
    // @todo empirical measurement on Uno: loop is 95 cycles
    uint32_t loops =
       microsecondsToClockCycles(DIGITAL_IO_SONAR_TIMEOUT)/95;
    // wait for the echo pulse to end
    while  (readRaw(_PORT_NAME pinNumberEcho) == HIGH) {
      if (--loops == 0) return 0;
    }
    computeValue();
    return getValue();
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Convert the usec value to the final distance units
  // * Multiply by MACH1
  // * Divide by 2 for round trip
  // * Adjust unit conversion ratios
  // * Do natural number operators order to max precision
  ////////////////////////////////////////////////////////////
  static inline uint16_t getDistance(uint16_t val16,
                                     digitalIoDistanceUnits units = cm)
  {
    uint32_t val = val16;
    // Speed of sound is 343m/s
    constexpr uint16_t DIGITAL_IO_MACH1 = 343;

    switch (units)
    {
      case cm:
        val = (val * DIGITAL_IO_MACH1) / 20000;
        break;
      case mm:
        val = (val * DIGITAL_IO_MACH1) / 2000;
        break;
      case inch:
        val = (val * DIGITAL_IO_MACH1) / 50800;
        break;
      case tenth:
        val = (val * DIGITAL_IO_MACH1) / 5080;
        break;
      case sixteenth:
        val = (val * DIGITAL_IO_MACH1) / 3175;
        break;
      default: // usec (microseconds latency)
       break;
    }
    return val;
  }

public:
  ////////////////////////////////////////////////////////////
  // @brief
  ////////////////////////////////////////////////////////////
  // @brief
  _CLASS_NAME(digitalSonar)(void)
  {
    outputModeRaw(_PORT_NAME pinNumberTrig);
    if (interruptFlag)
    {
      const uint8_t irq = 
                    _CLASS_NAME(pinToIrq)(_PORT_NAME pinNumberTrig, irqNumber);
      if (_CLASS_NAME(useInterrupt)(_PORT_NAME pinNumberTrig, interruptFlag, irqNumber))
      {
        // Options: Trigger interrupt on RISING, FALLING, CHANGE edge
        attachInterrupt(irq, computeValue, FALLING);
        // Set trigger pin to start interrupt handler's first sample
        triggerPing();
      }
    }
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Return the ping delay converted into a distance
  ////////////////////////////////////////////////////////////
  static uint16_t read(digitalIoDistanceUnits units = cm)
  {
    if (! _CLASS_NAME(useInterrupt)(_PORT_NAME pinNumberTrig, interruptFlag, irqNumber))
    {
      // Trigger the ping and wait to read echo that comes back
      triggerPing();
      return getDistance(waitForValue(), units);
    }

    uint16_t val;
    #ifdef AVR
    // Read value collected by interupt handler. Block ISR update to red coherent value
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    #endif // AVR
    {
      val = getValue();
    }
    // Trigger ping, will be collected by ISR for the next read
    triggerPing();
    return getDistance(val, units);
  }
}; // digitalSonar

// Static variable declarations for digitalSonar
template<_PORT_NAME_TEMPLATE uint8_t pinNumberTrig, uint8_t pinNumberEcho, bool interruptFlag, uint8_t irqNumber>
uint16_t _CLASS_NAME(digitalSonar)<_PORT_NAME pinNumberTrig, pinNumberEcho, interruptFlag, irqNumber>::sPingUsec = 0;

template<_PORT_NAME_TEMPLATE uint8_t pinNumberTrig, uint8_t pinNumberEcho, bool interruptFlag, uint8_t irqNumber>
uint16_t _CLASS_NAME(digitalSonar)<_PORT_NAME pinNumberTrig, pinNumberEcho, interruptFlag, irqNumber>::sPrevTimeUsec = 0;
