// Smart Traffic-Light Controller (UK sequence) - finite-state machine, no delay()
//
// A main road, a side road and a pedestrian crossing. The main road rests on green;
// the side road and the crossing are only served when there is a demand.
// Includes vehicle-actuated green extension, an emergency mode, a pedestrian
// countdown on a 7-segment display, red-lamp proving and a conflict monitor
// that drops the junction into flashing amber if anything is unsafe.
//
// Wiring (see README.md for diagrams):
//   Main road:  red D4, amber D5, green D6
//   Side road:  red D7, amber D8, green D9
//   Crossing:   red man D10, green man D11
//   Every LED:  pin -> LED -> 220R -> GND
//   Red-lamp proving: for the two vehicle REDS the 220R goes on the cathode side and the
//               LED/resistor junction goes to A4 (main red) or A5 (side red)
//   Buttons (to GND, internal pull-ups): pedestrian D2, side-road vehicle D3, emergency A3
//   Active buzzer: D12 -> buzzer -> GND      WAIT lamp: D13 (on-board LED)
//   74HC595 -> 1-digit 7-segment (common cathode, 220R per segment):
//               DS (pin 14) A0, ST_CP/latch (pin 12) A1, SH_CP/clock (pin 11) A2
//               Q0..Q6 -> segments a..g, Q7 -> dp, MR (10) -> 5V, OE (13) -> GND
//
// Serial (115200): type  ped | car | emergency | fault | reset | mute | status | help

// ---------- Pins ----------
const int PIN_MAIN_R = 4, PIN_MAIN_A = 5, PIN_MAIN_G = 6;
const int PIN_SIDE_R = 7, PIN_SIDE_A = 8, PIN_SIDE_G = 9;
const int PIN_PED_R = 10, PIN_PED_G = 11;
const int PIN_BUZZER = 12, PIN_WAIT = 13;
const int PIN_BTN_PED = 2, PIN_BTN_CAR = 3, PIN_BTN_EMERG = A3;
const int PIN_SR_DATA = A0, PIN_SR_LATCH = A1, PIN_SR_CLOCK = A2;
const int PIN_SENSE_MAIN_R = A4, PIN_SENSE_SIDE_R = A5;

const bool SEGMENT_COMMON_ANODE = false;  // set true if your display is common anode
const bool LAMP_PROVING = true;           // set false if you haven't wired A4/A5

// ---------- Timings (ms). Shortened for the bench; real junctions are longer ----------
const unsigned long T_MAIN_MIN_GREEN = 8000;
const unsigned long T_AMBER          = 3000;
const unsigned long T_ALL_RED        = 2000;
const unsigned long T_RED_AMBER      = 2000;
const unsigned long T_SIDE_MIN_GREEN = 5000;
const unsigned long T_SIDE_EXTENSION = 2000;   // each vehicle detection adds this
const unsigned long T_SIDE_MAX_GREEN = 15000;
const unsigned long T_PED_WALK       = 6000;
const unsigned long T_PED_CLEAR      = 5000;   // flashing green man
const unsigned long T_LAMP_SETTLE    = 100;    // ignore lamp sense this long after switching
const int SENSE_THRESHOLD = 150;               // ADC counts (~0.7 V)

// ---------- State machine ----------
enum State {
  MAIN_GREEN, MAIN_AMBER, ALL_RED,
  SIDE_RED_AMBER, SIDE_GREEN, SIDE_AMBER,
  PED_WALK, PED_CLEAR,
  MAIN_RED_AMBER, EMERGENCY, FAULT
};
const char *const STATE_NAMES[] = {
  "MAIN_GREEN", "MAIN_AMBER", "ALL_RED",
  "SIDE_RED_AMBER", "SIDE_GREEN", "SIDE_AMBER",
  "PED_WALK", "PED_CLEAR",
  "MAIN_RED_AMBER", "EMERGENCY", "FAULT"
};

enum Phase { PHASE_MAIN, PHASE_SIDE, PHASE_PED, PHASE_NONE };

State state = ALL_RED;
unsigned long stateStart = 0;
Phase lastPhase = PHASE_NONE;     // which phase had right of way before the all-red

bool pedDemand = false;
bool sideDemand = false;
bool emergency = false;
bool muted = false;
unsigned long sideGreenUntil = 0;
const char *faultReason = "";

// ---------- Outputs ----------
struct Lamps {
  bool mainR, mainA, mainG;
  bool sideR, sideA, sideG;
  bool pedR, pedG;
};
Lamps lamps = {};
Lamps prevLamps = {};
unsigned long mainRedChangedAt = 0, sideRedChangedAt = 0;
int shownDigit = -2;

// ---------- Buttons ----------
struct Button {
  int pin;
  bool stable;        // debounced level (HIGH = released)
  bool lastRaw;
  unsigned long lastChange;
  unsigned long pressedAt;
};
Button btnPed = {PIN_BTN_PED, HIGH, HIGH, 0, 0};
Button btnCar = {PIN_BTN_CAR, HIGH, HIGH, 0, 0};
Button btnEmerg = {PIN_BTN_EMERG, HIGH, HIGH, 0, 0};
bool emergHoldHandled = false;

// Returns true once per press (after 30 ms of stable LOW)
bool buttonPressed(Button &b, unsigned long now) {
  bool raw = digitalRead(b.pin);
  if (raw != b.lastRaw) {
    b.lastRaw = raw;
    b.lastChange = now;
  }
  if (now - b.lastChange > 30 && raw != b.stable) {
    b.stable = raw;
    if (b.stable == LOW) {
      b.pressedAt = now;
      return true;
    }
  }
  return false;
}

// ---------- 7-segment via 74HC595 ----------
const byte DIGITS[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};

void showDigit(int d) {  // d < 0 = blank
  if (d == shownDigit) return;
  shownDigit = d;
  byte pattern = (d >= 0 && d <= 9) ? DIGITS[d] : 0;
  if (SEGMENT_COMMON_ANODE) pattern = ~pattern;
  digitalWrite(PIN_SR_LATCH, LOW);
  shiftOut(PIN_SR_DATA, PIN_SR_CLOCK, MSBFIRST, pattern);
  digitalWrite(PIN_SR_LATCH, HIGH);
}

// ---------- Helpers ----------
void enterState(State s, const char *reason) {
  Serial.print(F("# "));
  Serial.print(millis() / 1000.0, 1);
  Serial.print(F("s  "));
  Serial.print(STATE_NAMES[state]);
  Serial.print(F(" -> "));
  Serial.print(STATE_NAMES[s]);
  Serial.print(F("  ("));
  Serial.print(reason);
  Serial.println(')');
  state = s;
  stateStart = millis();

  // Entry actions
  switch (s) {
    case MAIN_GREEN: lastPhase = PHASE_MAIN; break;
    case SIDE_GREEN:
      lastPhase = PHASE_SIDE;
      sideDemand = false;
      sideGreenUntil = stateStart + T_SIDE_MIN_GREEN;
      break;
    case PED_WALK:
      lastPhase = PHASE_PED;
      pedDemand = false;
      break;
    case EMERGENCY: lastPhase = PHASE_NONE; break;
    default: break;
  }
}

void enterFault(const char *reason) {
  if (state == FAULT) return;
  faultReason = reason;
  enterState(FAULT, reason);
}

bool flashOn(unsigned long now, unsigned long period) {
  return (now / (period / 2)) % 2 == 0;
}

// ---------- The state machine ----------
void updateStateMachine(unsigned long now) {
  unsigned long t = now - stateStart;

  switch (state) {
    case MAIN_GREEN:
      if (emergency) enterState(MAIN_AMBER, "emergency");
      else if (t >= T_MAIN_MIN_GREEN && sideDemand) enterState(MAIN_AMBER, "side road demand");
      else if (t >= T_MAIN_MIN_GREEN && pedDemand) enterState(MAIN_AMBER, "pedestrian demand");
      break;

    case MAIN_AMBER:
      if (t >= T_AMBER) enterState(ALL_RED, "amber done");
      break;

    case ALL_RED:
      if (t < T_ALL_RED) break;
      if (emergency) enterState(EMERGENCY, "emergency");
      else if (lastPhase == PHASE_MAIN && sideDemand) enterState(SIDE_RED_AMBER, "serve side road");
      else if (lastPhase == PHASE_MAIN && pedDemand) enterState(PED_WALK, "serve pedestrians");
      else enterState(MAIN_RED_AMBER, "return to main road");
      break;

    case SIDE_RED_AMBER:
      if (emergency) enterState(ALL_RED, "emergency");
      else if (t >= T_RED_AMBER) enterState(SIDE_GREEN, "red+amber done");
      break;

    case SIDE_GREEN:
      if (emergency) enterState(SIDE_AMBER, "emergency");
      else if (now >= sideGreenUntil) enterState(SIDE_AMBER, t >= T_SIDE_MAX_GREEN ? "max green" : "gap out");
      break;

    case SIDE_AMBER:
      if (t >= T_AMBER) enterState(ALL_RED, "amber done");
      break;

    case PED_WALK:
      if (emergency) enterState(PED_CLEAR, "emergency");
      else if (t >= T_PED_WALK) enterState(PED_CLEAR, "walk done");
      break;

    case PED_CLEAR:
      if (t >= T_PED_CLEAR) enterState(ALL_RED, "crossing clear");
      break;

    case MAIN_RED_AMBER:
      if (emergency) enterState(ALL_RED, "emergency");
      else if (t >= T_RED_AMBER) enterState(MAIN_GREEN, "red+amber done");
      break;

    case EMERGENCY:
      if (!emergency) enterState(ALL_RED, "emergency cleared");
      break;

    case FAULT:
      break;  // latched until 'reset' or holding the emergency button for 3 s
  }
}

// Decide which lamps are lit. Purely a function of the state and time.
Lamps lampsForState(unsigned long now) {
  Lamps L = {};
  bool flash = flashOn(now, 1000);
  switch (state) {
    case MAIN_GREEN:     L.mainG = 1; L.sideR = 1; L.pedR = 1; break;
    case MAIN_AMBER:     L.mainA = 1; L.sideR = 1; L.pedR = 1; break;
    case ALL_RED:
    case EMERGENCY:      L.mainR = 1; L.sideR = 1; L.pedR = 1; break;
    case SIDE_RED_AMBER: L.mainR = 1; L.sideR = 1; L.sideA = 1; L.pedR = 1; break;
    case SIDE_GREEN:     L.mainR = 1; L.sideG = 1; L.pedR = 1; break;
    case SIDE_AMBER:     L.mainR = 1; L.sideA = 1; L.pedR = 1; break;
    case PED_WALK:       L.mainR = 1; L.sideR = 1; L.pedG = 1; break;
    case PED_CLEAR:      L.mainR = 1; L.sideR = 1; L.pedG = flash; break;
    case MAIN_RED_AMBER: L.mainR = 1; L.mainA = 1; L.sideR = 1; L.pedR = 1; break;
    case FAULT:          L.mainA = flash; L.sideA = flash; break;  // fail-safe flashing amber
  }
  return L;
}

// Independent safety check, like the conflict monitor in a real controller:
// it doesn't trust the state table, it just looks at the outputs.
bool conflicting(const Lamps &L) {
  bool mainGo = L.mainG || (L.mainA && !L.mainR);
  bool sideGo = L.sideG || (L.sideA && !L.sideR);
  if (mainGo && sideGo) return true;
  if (L.pedG && (mainGo || sideGo || !L.mainR || !L.sideR)) return true;
  if (L.pedG && L.pedR) return true;
  return false;
}

// Red-lamp proving: a commanded red must draw current, an unlit one must not.
void checkLamp(bool commanded, int sensePin, unsigned long changedAt, unsigned long now, const char *name) {
  if (!LAMP_PROVING || now - changedAt < T_LAMP_SETTLE) return;
  int v = analogRead(sensePin);
  if (commanded && v < SENSE_THRESHOLD) {
    Serial.print(F("# lamp fault: ")); Serial.print(name); Serial.println(F(" red is not drawing current"));
    enterFault("red lamp failed");
  } else if (!commanded && v > SENSE_THRESHOLD) {
    Serial.print(F("# lamp fault: ")); Serial.print(name); Serial.println(F(" red has current while off"));
    enterFault("red lamp short");
  }
}

void writeLamps(const Lamps &L, unsigned long now) {
  if (L.mainR != prevLamps.mainR) mainRedChangedAt = now;
  if (L.sideR != prevLamps.sideR) sideRedChangedAt = now;
  prevLamps = L;
  digitalWrite(PIN_MAIN_R, L.mainR); digitalWrite(PIN_MAIN_A, L.mainA); digitalWrite(PIN_MAIN_G, L.mainG);
  digitalWrite(PIN_SIDE_R, L.sideR); digitalWrite(PIN_SIDE_A, L.sideA); digitalWrite(PIN_SIDE_G, L.sideG);
  digitalWrite(PIN_PED_R, L.pedR);   digitalWrite(PIN_PED_G, L.pedG);
}

void updateOutputs(unsigned long now) {
  Lamps L = lampsForState(now);
  if (state != FAULT && conflicting(L)) {
    enterFault("conflicting greens");
    L = lampsForState(now);
  }
  writeLamps(L, now);

  if (state != FAULT) {
    checkLamp(L.mainR, PIN_SENSE_MAIN_R, mainRedChangedAt, now, "main");
    checkLamp(L.sideR, PIN_SENSE_SIDE_R, sideRedChangedAt, now, "side");
  }

  // WAIT lamp while a pedestrian demand is registered
  digitalWrite(PIN_WAIT, pedDemand ? HIGH : LOW);

  // Countdown: seconds left for pedestrians (walk + clear), capped at 9
  unsigned long t = now - stateStart;
  if (state == PED_WALK) {
    showDigit(min(9UL, (T_PED_WALK - t + T_PED_CLEAR + 999) / 1000));
  } else if (state == PED_CLEAR) {
    showDigit(min(9UL, (T_PED_CLEAR - min(t, T_PED_CLEAR) + 999) / 1000));
  } else {
    showDigit(-1);
  }

  // Buzzer patterns
  bool buzz = false;
  if (state == PED_WALK)       buzz = (t % 400) < 60;     // crossing pips
  else if (state == EMERGENCY) buzz = flashOn(now, 1000);
  else if (state == FAULT)     buzz = (now % 2000) < 80;
  digitalWrite(PIN_BUZZER, (buzz && !muted) ? HIGH : LOW);
}

// ---------- Inputs ----------
void requestPed(const char *src) {
  if (state == PED_WALK || pedDemand) return;
  pedDemand = true;
  Serial.print(F("# pedestrian demand (")); Serial.print(src); Serial.println(')');
}

void vehicleDetected(const char *src) {
  if (state == SIDE_GREEN) {
    // Extend the side-road green while traffic keeps arriving, up to the maximum
    unsigned long cap = stateStart + T_SIDE_MAX_GREEN;
    sideGreenUntil = min(cap, max(sideGreenUntil, millis() + T_SIDE_EXTENSION));
    Serial.println(F("# side green extended"));
  } else if (!sideDemand) {
    sideDemand = true;
    Serial.print(F("# side road demand (")); Serial.print(src); Serial.println(')');
  }
}

void toggleEmergency() {
  emergency = !emergency;
  Serial.println(emergency ? F("# EMERGENCY requested") : F("# emergency cleared"));
}

void resetFault() {
  if (state != FAULT) return;
  emergency = false;
  lastPhase = PHASE_NONE;
  enterState(ALL_RED, "fault reset");
}

void readButtons(unsigned long now) {
  if (buttonPressed(btnPed, now)) requestPed("button");
  if (buttonPressed(btnCar, now)) vehicleDetected("sensor");
  if (buttonPressed(btnEmerg, now)) {
    emergHoldHandled = false;
  }
  // Emergency button: short press toggles, hold 3 s resets a fault
  if (btnEmerg.stable == LOW && !emergHoldHandled && now - btnEmerg.pressedAt > 3000) {
    emergHoldHandled = true;
    resetFault();
  }
  static bool emergWasDown = false;
  if (emergWasDown && btnEmerg.stable == HIGH && !emergHoldHandled && state != FAULT) toggleEmergency();
  emergWasDown = (btnEmerg.stable == LOW);
}

String rxLine;

void printStatus() {
  Serial.print(F("# state=")); Serial.print(STATE_NAMES[state]);
  Serial.print(F(" t=")); Serial.print((millis() - stateStart) / 1000.0, 1);
  Serial.print(F("s pedDemand=")); Serial.print(pedDemand);
  Serial.print(F(" sideDemand=")); Serial.print(sideDemand);
  Serial.print(F(" emergency=")); Serial.print(emergency);
  if (state == FAULT) { Serial.print(F(" fault=")); Serial.print(faultReason); }
  Serial.println();
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  if (cmd == "ped") requestPed("serial");
  else if (cmd == "car") vehicleDetected("serial");
  else if (cmd == "emergency") toggleEmergency();
  else if (cmd == "fault") enterFault("simulated fault");
  else if (cmd == "reset") resetFault();
  else if (cmd == "mute") { muted = !muted; Serial.println(muted ? F("# muted") : F("# sound on")); }
  else if (cmd == "status") printStatus();
  else Serial.println(F("# commands: ped | car | emergency | fault | reset | mute | status"));
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxLine.length()) handleCommand(rxLine);
      rxLine = "";
    } else if (rxLine.length() < 32) {
      rxLine += c;
    }
  }
}

// ---------- Setup / loop ----------
void setup() {
  Serial.begin(115200);
  const int outputs[] = {PIN_MAIN_R, PIN_MAIN_A, PIN_MAIN_G, PIN_SIDE_R, PIN_SIDE_A, PIN_SIDE_G,
                         PIN_PED_R, PIN_PED_G, PIN_BUZZER, PIN_WAIT,
                         PIN_SR_DATA, PIN_SR_LATCH, PIN_SR_CLOCK};
  for (int p : outputs) pinMode(p, OUTPUT);
  pinMode(PIN_BTN_PED, INPUT_PULLUP);
  pinMode(PIN_BTN_CAR, INPUT_PULLUP);
  pinMode(PIN_BTN_EMERG, INPUT_PULLUP);
  showDigit(-1);

  Serial.println(F("# Smart traffic-light controller. Type 'help' for commands."));
  // Start safely: everything red, then give the main road green
  stateStart = millis();
  lastPhase = PHASE_NONE;
  mainRedChangedAt = sideRedChangedAt = millis();
}

void loop() {
  unsigned long now = millis();
  readButtons(now);
  readSerial();
  updateStateMachine(now);
  updateOutputs(millis());
}
