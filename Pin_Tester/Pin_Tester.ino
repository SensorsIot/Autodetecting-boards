/*
  Pin Connection Test Sketch
  This sketch defines a function isPinConnected() that tests a pin using both an
  internal pull-up and pull-down resistor. It returns true if:
    - With pull-up: the pin, which normally reads HIGH, is pulled LOW.
    - With pull-down: the pin, which normally reads LOW, is pulled HIGH.
  The sketch then calls this function and prints the result to the Serial Monitor.
*/

bool isPinConnected(int pin) {
  bool connected = false;
  unsigned long startTime;

  pinMode(pin, INPUT_PULLUP);
  startTime = millis();
  while (millis() - startTime < 1000) {  // Check for 1 second.
    if (digitalRead(pin) == LOW) {
      connected = true;
      break;
    }
  }
  if (!connected) {
    pinMode(pin, INPUT_PULLDOWN);
    startTime = millis();
    while (millis() - startTime < 1000) {  // Check for 1 second.
      if (digitalRead(pin) == HIGH) {
        connected = true;
        break;
      }
    }
  }
  return connected;
}

void setup() {
  Serial.begin(115200);
  int testPin = 15;  // Replace with the desired pin number.

  Serial.println("Starting Pin Connection Test...");

  // Test if something is connected to the testPin.
  if (isPinConnected(testPin)) {
    Serial.println("Something is connected to the pin.");
  } else {
    Serial.println("Nothing detected on the pin.");
  }
}

void loop() {
  // Nothing to do in loop for this simple test.
}
