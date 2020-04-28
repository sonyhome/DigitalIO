////////////////////////////////////////////////////////////////////////////////
// TaskScheduler demo - Copyright (c) 2020 Dan Truong
// A class to run functions at chosen time intervals
////////////////////////////////////////////////////////////////////////////////
// This simple example shows how to use the Task Scheduler to run 2 functions at
// different time intervals.
// Note:
// This code only works on Arduinos with AVR microcontroller like UNO, Duo, etc.
// If your board is incompatible a compile time error will notify you of it.
////////////////////////////////////////////////////////////////////////////////
// Global variables exported:
// nowUsec time since first task scheduler started (close to time since boot)
//
// TaskScheduler usage:
// TaskScheduler yourTask(periodMsec, yourFunction)
//         this declaration will configure the scheduler to run the yourFunction
//         at the given repeat interval. The function will start running and
//         repeating itself as soon as that line of code is executed.
//
// TaskScheduler methods:
// begin() Call it in setup() if yourTask was declared as a global variable.
//         If you declare yourTask as a static variable of loop(), don't bother.
// run()   just calls yourTask(). You won't need to use it.
// getNow() it just returns nowUsec.
// getPeriod() read the period you specified in usec.
// setPeriod() change the period, in msec (notice the unit).
// getTimeout() read the next time the function will run.
// getWhen() read the microseconds left before the function will run.
// destructor: It is possible to free yourTask. If so it is removed from the
//         task list. However the counter interupt won't be turned off even if
//         there are no tasks left to run. You won't use this feature anyways.
//
// Advanced Macros:
// TASK_SCHEDULER_DEBUG Debug mode to display traces on Serial console (1~4)
// TASK_SCHEDULER_OCR   Override use of Timer0's counter A ('A' or 'B')
// TASK_SCHEDULER_TIMER Override use of Timer0 (0~2). Note that YOU must program
//         and enable the timer to count to 255 if you don't use Timer0.
////////////////////////////////////////////////////////////////////////////////
// The library is coded to minimise its overheads.
//
// Memory Overhead:
// Calling foo() directly  1792 - 218 bytes
// Calling foo() scheduler 2376 - 240 bytes (+584 +22)
// Calling foo() bar() directly  1832 - 234 bytes
// Calling foo() bar() scheduler 2522 - 274 bytes (+690 +40)
////////////////////////////////////////////////////////////////////////////////
//#define TASK_SCHEDULER_DEBUG 3
#include <TaskScheduler.hpp>

void foo(void) {
  // Put your code in here, but keep it small and running fast.
  // Do NOT call delay() in the method, it does not work.
  // Do not expect nowUsec to change while the function runs.s
  Serial.println("foo is running!");
}

void bar(void) {
  Serial.println("bar is running!");
}

// Define time constants for code clarity
constexpr uint16_t loopPeriodMsec = 500;
constexpr uint16_t fooPeriodMsec = 1000;
constexpr uint16_t barPeriodMsec = 2000;
constexpr uint16_t fooIter = fooPeriodMsec/loopPeriodMsec;
constexpr uint16_t barIter = barPeriodMsec/loopPeriodMsec;

void setup() {
  Serial.begin(9600);
  delay(500);
  Serial.println("\nStarting TaskScheduler demo!");
}

void loop() {
  // Notice the "static" keyword is needed in the declarations
  // These 2 lines are sufficient to run those functions at the
  // specified period. No other code is needed.
  static taskScheduler fooLoop(fooPeriodMsec, foo);
  static taskScheduler barLoop(barPeriodMsec, bar);

  // Equivalent code to run foo at the same period from the main loop:
  // static uint32_t i = 0;
  // i++;
  // if (i % fooIter == 0) { foo(); }
  // if (i % barIter == 0) { bar(); }

  // The taskScheduler exposes the nowUsec global variable which
  // reports the time since the taskScheduler engine started.
  Serial.print(nowUsec/1000);
  Serial.println("ms");

  // let loop() sleep for a while
  delay(loopPeriodMsec);
}
