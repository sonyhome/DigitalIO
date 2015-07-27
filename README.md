# digitalSwitch
A straightforward library to use digital switches on Arduinos

## Installation

Create a DigitalSwitch directory and copy DigitalSwitch.ino into it (on Windows: MyDocuments/Arduino/libraries/DigitalSwitch) and rename to DigitalSwitch.h

## Usage

Instantiate the class and use as follows:

```cpp
#include <DigitalSwitch.h>

// For example your switch is on pin 10, and when pressed
// it grounds the pin, so it is HIGH when unused, and LOW
// when used.
digitalSwitch<10, HIGH> switch10;

void setup() {
  Serial.begin(9600);
  switch10.begin();
}

void loop() {
  // Read()  tells you the current state
  if (switch10.read()) {
    Serial.println("Pressed!");
  }
 // Debounce will detect transitions and return zero
 // otherwise
  if (1 == switch10.debounce()) {
    Serial.println("Button pressed down");
  } else if (-1 == switch10.debounce()) {
    Serial.println("Button released");
  }
  delay(500);
}
```

## Contributing

1. Fork it!
2. Create your feature branch: `git checkout -b my-new-feature`
3. Commit your changes: `git commit -am 'Add some feature'`
4. Push to the branch: `git push origin my-new-feature`
5. Submit a pull request

## History

1. Version 1.0 (alpha version)

## Credits

Copyright (c) 2015, Dan Truong

## License

T.B.D.
