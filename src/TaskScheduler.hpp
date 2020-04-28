#ifndef TASK_SCHEDULER
#define TASK_SCHEDULER
////////////////////////////////////////////////////////////////////////////////
// @brief
// Copyright (c) 2020 Dan Truong
//
// A Timer Interrupt based Scheduler Library for Arduino
//
// The Scheduler class makes it easy on AVR to schedule tasks repetitively on
// a programmable period of 1 msec to 65 seconds.
// Usage:
// Declare your function, declare the scheduler variable, and it just runs
// independently in the background of your main loop().
////////////////////////////////////////////////////////////////////////////////
// TODO: Reenable interrupts and add a flag to prevent a handler from running
// again if it's not finished. Compute its next time so it doesnt overrun the cpu.
// TODO: non interrupt driven task scheduler
////////////////////////////////////////////////////////////////////////////////
// Basic use:
//
// void foo(void) { Serial.println("foo is running!"); }
// void setup() { Serial.begin(9600); }
// void loop() {
//   static taskScheduler fooLoop(1000, foo);
//   Serial.println(nowUsec/1000000);
//   delay(2000);
// }
//
// Advanced use:
//
// Override macros (set them before including the library):
//
// TASK_SCHEDULER_DEBUG Debug mode to display traces on Serial console (1~4)
// TASK_SCHEDULER_TIMER Override use of Timer0 (0~2)
// TASK_SCHEDULER_OCR   Override use of Timer0's counter A ('A' or 'B')
// 
////////////////////////////////////////////////////////////////////////////////
// Internals
//
// The library takes advantage of Timer0 being used by the Arduino timer, used
// for delay() and millis(). It does not modify this part of the code nor its
// interrupt handler. Instead it attaches to the Timer0 Comparator A register.
//
// This triggers a separate interrupt handler which counts the time elapsed at
// every overflow of the 8bit Timer0. If less than 1msec has elapsed it returns.
//
// The interrupt handler implemented scans a linked list of callbacks every
// millisecond which contain their timeout values and triggers them as needed.
//
// Tasks are added to the head of the list as they are declared. This means
// that we'll try to run the last task declared first.
//
// Low Memory overhead:
// 22B of dynamic memory for global variables + 18 bytes for each Scheduler
// variable used in the main program. The program uses 580 to 690 bytes of
// program storage space.
////////////////////////////////////////////////////////////////////////////////
#include <stdint.h>

#ifndef ARDUINO_ARCH_AVR
#error Unsupported Microncontroller Architecture (Only AVR is supported now)
#endif


////////////////////////////////////////////////////////////////////////////////
/// Definitions
/// These shouldn't need to be changed but may be overriden by the user
////////////////////////////////////////////////////////////////////////////////

/// @brief
/// TASK_SCHEDULER_DEBUG forces the library into debug mode
/// 1: prints settings only
/// 2: prints transitions only
/// 3: prints both
/// 4: only forces pin initialization as input (usually the default, not needed)
#ifdef TASK_SCHEDULER_DEBUG
#warning TASK_SCHEDULER_DEBUG mode is set
static_assert(TASK_SCHEDULER_DEBUG >= 0 && TASK_SCHEDULER_DEBUG <= 4);
#define DEBUG_PRINT1(text) {if ((TASK_SCHEDULER_DEBUG & 1) != 0) {Serial.print(text);}}
#define DEBUG_PRINT2(text) {if ((TASK_SCHEDULER_DEBUG & 2) != 0) {Serial.print(text);}}
#define DEBUG_PRINT3(text) {if ((TASK_SCHEDULER_DEBUG & 3) != 0) {Serial.print(text);}}
#define DEBUG(action) action
constexpr bool DEBUG_MODE = (TASK_SCHEDULER_DEBUG > 1);
#else
#define DEBUG_PRINT1(text)
#define DEBUG_PRINT2(text)
#define DEBUG_PRINT3(text)
#define DEBUG(action)
constexpr bool DEBUG_MODE = false;
#endif

////////////////////////////////////////////////////////////////////////////////
// @brief
// Allow override of Timer used and Counter used just by specifying the counter
// Default is "OCR0A"
// Allowed OCR0A, OCR0B, OCR1A, OCR1B, OCR2A, OCR2B
// Note for non-default values, the timers MUST be programmed to count to 255
////////////////////////////////////////////////////////////////////////////////
#ifndef TASK_SCHEDULER_TIMER
#define TASK_SCHEDULER_TIMER 0
#endif
#ifndef TASK_SCHEDULER_OCR
#define TASK_SCHEDULER_OCR 'A'
#endif
////////////////////////////////////////////////////////////////////////////////
#if (TASK_SCHEDULER_TIMER == 0)
#if (TASK_SCHEDULER_OCR == 'A')
#define OCRxA			OCR0A
#else
#define OCRxA			OCR0B
#endif // TASK_SCHEDULER_OCR
//#define TCNTx			TCNT0
#define TCCRxA			TCCR0A
#define TCCRxB			TCCR0B
#define TIMSKx			TIMSK0
#define TIFRx			TIFR0
#elif (TASK_SCHEDULER_TIMER == 1)
#if   (TASK_SCHEDULER_OCR == 'A')
#define OCRxA			OCR1A
#else
#define OCRxA			OCR1B
#endif // TASK_SCHEDULER_OCR
//#define TCNTx			TCNT1
#define TCCRxA			TCCR1A
#define TCCRxB			TCCR1B
#define TIMSKx			TIMSK1
#define TIFRx			TIFR1
#elif (TASK_SCHEDULER_TIMER == 2)
#if   (TASK_SCHEDULER_OCR == 'A')
#define OCRxA			OCR2A
#else
#define OCRxA			OCR2B
#endif // TASK_SCHEDULER_OCR
//#define TCNTx			TCNT2
#define TCCRxA			TCCR2A
#define TCCRxB			TCCR2B
#define TIMSKx			TIMSK2
#define TIFRx			TIFR2
#endif // TASK_SCHEDULER_TIMER

////////////////////////////////////////////////////////////////////////////////
// @brief
// Read AVR internal prescaler configuration (CS02|CS02|CS00 bits in TCCR0B reg
// (a frequency fed from an external pin is not supported)
////////////////////////////////////////////////////////////////////////////////
#ifndef PRESCALER
#define PRESCALER(CSxx) (((CSxx) == 0x05) ? 1024 : ((CSxx) == 0x04) ? 256 :    \
                        ((CSxx) == 0x03) ? 64 : ((CSxx) == 0x02) ? 8 :         \
                        ((CSxx) == 0x01) ? 1 : 0)
#endif // PRESCALER



////////////////////////////////////////////////////////////////////////////////
// @brief
// Head of linked list of callbacks
////////////////////////////////////////////////////////////////////////////////
struct taskScheduler; // Forward declaration
static struct taskScheduler* schedulerCallbackList = nullptr;

////////////////////////////////////////////////////////////////////////////////
// @brief
// Single static running counter of elapsed time
////////////////////////////////////////////////////////////////////////////////
static uint32_t nowUsec = 0;

////////////////////////////////////////////////////////////////////////////////
// @brief
// Macros to compute time adjustments
// The time between OCR0A interrupts depends on the CPU frequency (16MHz),
// the prescaler settings (64) and max value of the Timer0 (8bits) => 1024usec
////////////////////////////////////////////////////////////////////////////////
//static uint16_t usecPerIrq = 1000;
static uint16_t usecPerIrq;


////////////////////////////////////////////////////////////////////////////////
// @brief
// This is the main class!
// Defines a global linked list entry for OCR0A callbacks
////////////////////////////////////////////////////////////////////////////////
class taskScheduler
{
public:
  // Cannot protect class variables that ISR need to access
  struct taskScheduler* next;
  void (* isrCallback)(void);
  uint32_t timeoutUsec;
  uint16_t periodMsec;

  ////////////////////////////////////////////////////////////
  // @brief
  // Executes the callback (not needed)
  ////////////////////////////////////////////////////////////
  inline void run(void)
  {
    isrCallback();
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Return current time in usec (micro seconds)
  ////////////////////////////////////////////////////////////
  inline uint32_t getNow()
  {
    DEBUG_PRINT1("nowUsec = ");
    DEBUG_PRINT1(nowUsec);
    DEBUG_PRINT1("\n");
    return nowUsec;
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Return the period programmed in usec
  ////////////////////////////////////////////////////////////
  inline uint32_t getPeriod()
  {
    DEBUG_PRINT1("periodUsec = ");
    DEBUG_PRINT1(periodMsec * 1000UL);
    DEBUG_PRINT1("\n");
    return periodMsec * 1000UL;
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Set the period programmed in msec
  ////////////////////////////////////////////////////////////
  inline void setPeriod(uint16_t _periodMsec)
  {
    periodMsec = _periodMsec;
    DEBUG_PRINT1("periodMsec = ");
    DEBUG_PRINT1(periodMsec);
    DEBUG_PRINT1("\n");
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Return next time the callback runs in usec
  ////////////////////////////////////////////////////////////
  inline uint32_t getTimeout()
  {
    DEBUG_PRINT1("timeoutUsec = ");
    DEBUG_PRINT1(timeoutUsec);
    DEBUG_PRINT1("\n");
    return timeoutUsec;
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Return how long until callback is called again in usec
  ////////////////////////////////////////////////////////////
  inline uint32_t getWhen()
  {
    return getTimeout() - getNow();
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Return how long until callback is called again in usec
  ////////////////////////////////////////////////////////////
  static inline uint32_t begin()
  {
    // Init static values once but not too early so TCCR0B gets set
    usecPerIrq = 256UL * (1000000UL * PRESCALER((TCCRxB & 0x07)) / F_CPU);

    // disable interrupts
    cli();
    // Set TIMER0_COMPA to trigger at opposite period of TIMER0
    OCRxA = 128;
    // Enable CCRA interrupt
    TIMSKx |= 1<<OCIE0A;
    // enable interrupts
    sei();

    DEBUG_PRINT2("\nScheduler properties: \n");
    DEBUG_PRINT2("PRESCALER=");
    DEBUG_PRINT2(PRESCALER(TCCRxB & 0x07));
    DEBUG_PRINT2("\nF=");
    DEBUG_PRINT2(F_CPU/PRESCALER(TCCRxB & 0x07));
    DEBUG_PRINT2("\nTICK USEC=");
    DEBUG_PRINT2(1000000UL*PRESCALER(TCCRxB & 0x07)/F_CPU);
    DEBUG_PRINT2("\nIRQ USEC=");
    DEBUG_PRINT2(256UL*(1000000UL*PRESCALER(TCCRxB & 0x07)/F_CPU));

    DEBUG_PRINT2("\nusecPerIrq=");
    DEBUG_PRINT2(usecPerIrq);
    DEBUG_PRINT2("\nusecPerIrq=");
    DEBUG_PRINT2(usecPerIrq);

    // Timer/Counter Register (running counter, useless)
    //DEBUG_PRINT2(TCNT0);
    // 0 Output Compare Register A
    DEBUG_PRINT2("\nOCR**=");
    DEBUG_PRINT2(OCRxA);
    // 3 Timer/Counter Control Registers A (WGM0x = b11 = FastPWM 0xFF)
    DEBUG_PRINT2("\nTCCR*A=");
    DEBUG_PRINT2(TCCRxA);
    // 3 Timer/Counter Control Registers B (CS0x=b011 = clk/64 = 4us/tick = 1.024ms/irq)
    DEBUG_PRINT2("\nTCCR*B=");
    DEBUG_PRINT2(TCCRxB);
    // 1 Timer/Counter Interrupt Mask Register
    DEBUG_PRINT2("\nTIMSK*=");
    DEBUG_PRINT2(TIMSKx);
    // 0 Timer/Counter Interrupt Flag Register
    DEBUG_PRINT2("\nTIFR*=");
    DEBUG_PRINT2(TIFRx);

    DEBUG_PRINT2("\nStarting scheduler!\n");
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Destructor
  // It's odd to do but you can uninstall the callback
  // @todo We could turn off the OCR0A interupt handler if
  // there are no more callbacks registered.
  ////////////////////////////////////////////////////////////
  ~taskScheduler()
  {
    DEBUG_PRINT2("Destructor of ");
    DEBUG_PRINT2(periodMsec);
    DEBUG_PRINT2("msec handler called!\n");
    if (schedulerCallbackList == this)
    {
      schedulerCallbackList = schedulerCallbackList->next;
      return;
    }

    taskScheduler * current = schedulerCallbackList;
    while (current)
    {
      if (current->next == this)
      {
        // Detach this
        current->next = current->next->next;
        return;
      }
    }
    DEBUG_PRINT2("ERROR! Handler not detached!\n");
  }

  ////////////////////////////////////////////////////////////
  // @brief
  // Constructor
  // Enable to OCR0A interrupt vector, and attach callback
  // On exit of this function, the callback is running.
  //
  // isrCallback:
  //    name of function to run - no inputs or return value!
  // periodMsec:
  //    time between calls in milliseconds (1ms to 65s)
  // offsetMsec:
  //    optional offset to not have all callbacks with same
  //    periods and interfere with each other's schedule
  ////////////////////////////////////////////////////////////
  taskScheduler(
    void (* _callback)(void),
    uint16_t _periodMsec,
    uint16_t _offsetMsec = 0)
  {
    // Init interupt handler only once (static config)
    if (schedulerCallbackList == nullptr)
    {
      begin();
    }

    // Create an entry for this handler
    next = schedulerCallbackList;
    isrCallback = _callback;
    periodMsec = _periodMsec;
    timeoutUsec = nowUsec + _offsetMsec + ((uint32_t) _periodMsec * 1000UL);

    // Insert handler in global list of existing handlers
    schedulerCallbackList = this;
    // Handler is now live!

    DEBUG_PRINT2("Starting ");
    DEBUG_PRINT2(periodMsec);
    DEBUG_PRINT2("msec handler!\n");
  }
}; // taskScheduler


////////////////////////////////////////////////////////////
// @brief
// Attach interrupt handler to OCR0A interrupt vector.
// The handler tracks and adjust time accounting.
////////////////////////////////////////////////////////////
ISR(TIMER0_COMPA_vect)
//SIGNAL(TIMER0_COMPA_vect)
{
  // Snapshot of running counter
  const uint32_t prevUsec = nowUsec;

  nowUsec += usecPerIrq;
  
  // If 1ms or more has elapsed, check to run timed-out callbacks
  if (nowUsec/1000 != prevUsec/1000)
  {
    DEBUG(uint8_t index = 0;)

    // Load the static list of callbacks
    auto * current = schedulerCallbackList;

    while (current)
    {
      DEBUG(index++;)
      // If nowUsec < prevUsec we have a wraparound
      if (current->timeoutUsec <= nowUsec && current->timeoutUsec > prevUsec)
      {
        DEBUG_PRINT1(index);
        DEBUG_PRINT1(": ");
        DEBUG_PRINT1(current->timeoutUsec);
        DEBUG_PRINT1(" = ");
        DEBUG_PRINT1(nowUsec);
        DEBUG_PRINT1(" + ");
        DEBUG_PRINT1((uint32_t) current->periodMsec * 1000UL);
        DEBUG_PRINT1(" = ");
        DEBUG_PRINT1(index);
        // Update timeout
        current->timeoutUsec = nowUsec + ((uint32_t) current->periodMsec * 1000UL);
        DEBUG_PRINT1(current->timeoutUsec);
        DEBUG_PRINT1("\n");
        // Run callback
        current->isrCallback();
      }
      current = current->next;
    }
  }
}

#endif // TASK_SCHEDULER
