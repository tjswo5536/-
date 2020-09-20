#define PIN_LED 7
unsigned int count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200); // Initialize serial port
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  Serial.println("Hello World!");
  count = toggle = 0;
  digitalWrite(PIN_LED, toggle); // turn off LED.
}

void loop() {
  int i;
  Serial.println(++count);
  toggle = toggle_state(toggle); // toggle LED value.
  digitalWrite(PIN_LED, toggle); // update LED status.
  delay(1000); // wait for 1,000 milliseconds

  i=0;
  while(i<10) {
    toggle = 1 - toggle;
    toggle = toggle_state(toggle);
    digitalWrite(PIN_LED, toggle);
    delay(100);
    i++;
  }
  while(1) {
    toggle = 1;
    toggle = toggle_state(toggle);
    digitalWrite(PIN_LED, toggle);
  }
}

int toggle_state(int toggle) {
    return toggle;
}
