// Two HC-SR04s for the drone, pinged one after the other (never together, so one sensor can't
// hear the other's echo). Prints one line per reading, each sensor at ~20 Hz:
//   "d <millimetres>"  downward: height above the floor (to the FC as DISTANCE_SENSOR)
//   "f <millimetres>"  forward: obstacles (to the edge's safety gate)
//   "<x> -1"           no echo / out of range
// Wiring, 115200 baud:
//   down:    VCC->5V, GND->GND, Trig->D9,  Echo->D10
//   forward: VCC->5V, GND->GND, Trig->D11, Echo->D12
// tools/rangefinder.py reads these lines (USB) and forwards them to the edge.
// With no forward sensor wired, "f -1" (clear) is printed: check the forward reading with
// `python -m tools.rangefinder ... --print` before trusting it.
const int DOWN_TRIG = 9, DOWN_ECHO = 10;
const int FWD_TRIG = 11, FWD_ECHO = 12;
const unsigned long TIMEOUT_US = 15000;  // ~2.5 m round trip; the edge and FC cap at 2 m

long ping(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long us = pulseIn(echo, HIGH, TIMEOUT_US);
  return us == 0 ? -1 : (long)(us * 343UL / 2000UL);  // speed of sound 343 m/s -> mm
}

void report(char tag, long mm) {
  Serial.print(tag);
  Serial.print(' ');
  Serial.println(mm);
}

void setup() {
  pinMode(DOWN_TRIG, OUTPUT);
  pinMode(DOWN_ECHO, INPUT);
  pinMode(FWD_TRIG, OUTPUT);
  pinMode(FWD_ECHO, INPUT);
  digitalWrite(DOWN_TRIG, LOW);
  digitalWrite(FWD_TRIG, LOW);
  Serial.begin(115200);
}

void loop() {
  report('d', ping(DOWN_TRIG, DOWN_ECHO));
  delay(10);  // let stray echoes die down before the other sensor pings
  report('f', ping(FWD_TRIG, FWD_ECHO));
  delay(10);
}
