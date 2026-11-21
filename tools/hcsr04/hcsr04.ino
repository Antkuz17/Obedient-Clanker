// HC-SR04 downward rangefinder for the drone. Prints one line per reading at ~20 Hz:
//   "d <millimetres>"   or   "d -1"  (no echo / out of range)
// Wiring: VCC->5V, GND->GND, Trig->D9, Echo->D10. 115200 baud.
// tools/rangefinder.py reads these lines (USB) and forwards them to the edge.
const int TRIG = 9, ECHO = 10;
const unsigned long TIMEOUT_US = 15000;  // ~2.5 m round trip; the FC caps at 2 m anyway

void setup() {
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  digitalWrite(TRIG, LOW);
  Serial.begin(115200);
}

void loop() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  unsigned long us = pulseIn(ECHO, HIGH, TIMEOUT_US);
  Serial.print("d ");
  if (us == 0) Serial.println(-1);
  else Serial.println((long)(us * 343UL / 2000UL));  // speed of sound 343 m/s -> mm
  delay(50);  // ~20 Hz, and lets echoes die down between pings
}
