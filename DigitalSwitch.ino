#ifndef DIGITAL_SWITCH
#define DIGITAL_SWITCH
////////////////////////////////////////////////////////////////////////////////
/// @brief
/// Copyright (c) 2015 Dan Truong
///
/// Digital Switch Library for Arduino
///
/// The DigitalSwitch class makes it easy to use buttons on Arduino by exporting
/// a read() method and a debounce() method to know the state of the button with
/// a single line of code.
/// The library follow the same syntax as the Serial library class, with a
/// begin() method that initializes the I/O pin, and a read() routine that
/// returns true if the button is pressed, or debounce which implements a
/// simple debounce heuristic to avoid read glitches.
///
/// The class is a templace class, which allows you to create an instance
/// customized for the pin and type of switch you are using (on HIGH or
/// on when LOW).
/// The library defines one default switch on PORT B0.
///
/// This template design allows the compiler to optimize away anything that
/// is not used.
////////////////////////////////////////////////////////////////////////////////

template<uint8_t pin, int restState>
class digitalSwitch {
public:
  /// @brief Call me once in setup()
  void begin(void) {
    pinMode(pin, INPUT);
    if (restState == HIGH) {
      //pinMode(pin, INPUT_PULLUP);
      digitalWrite(pin, HIGH);
    }
  }
  
  /// @brief Returns true if the button is currently being pressed
  bool read(void) {
    return (restState != digitalRead(pin));
  }

  /// @brief
  /// Debounce returns a non-zero value when the button transitions state
  /// 1 if button was pressed
  /// -1 if button was released
  /// 0 if button is not changed state
  bool debounce(void) {
    static const uint8_t timeDelay = 30;
    static uint8_t lastState = restState;
    static uint16_t lastTime = 0;
    
    uint8_t state = digitalRead(pin);
    
    if (state == lastState) {
      // Button is in same state as last call: Check if we're debouncing a transition
      if (lastTime = 0xFFFF) {
        // We're not debouncing. State changed long ago. 
        return 0;
      } else {
        // We're debouncing, so we check the time.
        if (millis() >= lastTime + timeDelay) {
          // Detected a debounced transition
          lastTime = 0xFFFF;
          return (state != restState) ? 1 : -1;
        }
        return 0;
      }
    } else {
      // Button changed from pressed to not pressed or vice versa. Reset timer.
      lastState = state;
      lastTime = millis();
      return 0;
    }
  }
};

// Set switch to high when not pressed for standard switches which will leave the
// pin floating and not grounded to put a pull-up internally forcing the pin to be
// at 5V when nothing is connected. When the switch is pressed it connects the pin
// to the ground, a logical LOW.
digitalSwitch<PB0, HIGH> Switch;

#endif // DIGITAL_SWITCH

/*
// Example code
void setup() {
  Switch.begin();
}

void loop() {
  if (1 == Switch.debounce()) {
    Serial.println("Button pressed");
  } else if (-1 == Switch.debounce()) {
    Serial.println("Button released");
  }
}
*/
