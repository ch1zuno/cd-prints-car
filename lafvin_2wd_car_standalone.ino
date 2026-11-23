/*
  LAFVIN 2WD Robot Car: IR remote + obstacle sensor (ultrasonic)
  Board: Arduino UNO R3, LAFVIN 2WD Smart Robot Car V2.2 (TB6612FNG motor driver)
  Remove the Bluetooth module (pins 0/1) before uploading.

  NO IR LIBRARY NEEDED. The NEC decoder is built into this sketch.
  Do NOT put the kit's IR_remote / Keymap files in this sketch's folder.

  ACTIONS: FORWARD, BACKWARD, LEFT, RIGHT, STOP, SLOW, MEDIUM, FAST, AUTO MODE

  The ultrasonic sensor sweeps left and right on its servo while the car drives forward.
  An obstacle only counts after several readings in a row, so stray echoes are ignored.
  In auto mode the car stops, looks left and right, then turns toward the clearer side.

  TEACH THE CAR YOUR REMOTE (do this once, it is saved in memory):
    1) Open Serial Monitor at 9600 baud
    2) Type  M  and press Enter
    3) Press the remote button you want for each action, as the monitor asks
  Type  X  to go back to the default codes.

  PC / Bluetooth remote letters (prefixed with !):
    !U(fwd) !D(back) !L(left) !R(right) !F(fwd-left) !G(fwd-right) !B(back-left) !N(back-right)
    !O or !0(stop) !1(slow) !2(med) !3(fast) !4(auto) !5(line track) !6(follow) !7(light seek) !S(motor test)
  PC / Bluetooth settings (prefixed with !): !V150 speed, !T20 stop distance, !H3 confirmations, !E1/!E0 sensor, !W1/!W0 sweep, !A90 aim sensor (0-180), !J0/!J1 track invert, !P read distance, !K test line, !Z test light/IR, !Q telemetry/battery
*/

#include <Servo.h>
#include <EEPROM.h>

#define DEBUG 1 // un-comment to print debug & shield LED states to Serial Monitor

// ---------------- Pin setup (LAFVIN 2WD V2.2, TB6612FNG driver) ----------------
#define LEFT_DIR   2    // left motor direction (leftDIO)
#define LEFT_PWM   5    // left motor speed (leftPWM)
#define RIGHT_DIR  4    // right motor direction (rightDIO)
#define RIGHT_PWM  6    // right motor speed (rightPWM)

#define IR_PIN     3    // IR receiver & shield LED_IRREMOTE (D3 LOW = LED ON)
#define TRIG_PIN   12   // ultrasonic trigger
#define ECHO_PIN   13   // ultrasonic echo (shares D13 / LED_BUILTIN; keep as INPUT for echo)
#define SERVO_PIN  10   // servo that holds the ultrasonic sensor

// ---------------- Sensor pins (LAFVIN official) ----------------
#define TRACK_LEFT_PIN   7    // Left tracking sensor (D7)
#define TRACK_MID_PIN    8    // Center tracking sensor (D8)
#define TRACK_RIGHT_PIN  9    // Right tracking sensor (D9)
#define LIGHT_LEFT_PIN   A0   // Left photoresistor (A0)
#define FOLLOW_LEFT_IR   A1   // Left IR obstacle/follow sensor (A1)
#define FOLLOW_RIGHT_IR  A2   // Right IR obstacle/follow sensor (A2)
#define LIGHT_RIGHT_PIN  A3   // Right photoresistor (A3)
#define BATT_PIN         A4   // Optional 1:1 voltage divider on free pin A4
#define USE_BATT_DIVIDER 0    // 0 = built-in 5V rail health (no extra wiring), 1 = A4 divider (6.0-8.4V)

// ---------------- Actions and remote codes ----------------
enum Action { ACT_FWD, ACT_BACK, ACT_LEFT, ACT_RIGHT, ACT_STOP,
              ACT_SLOW, ACT_MED, ACT_FAST, ACT_AUTO, ACT_TRACK,
              ACT_FOLLOW, ACT_LIGHT, ACT_COUNT };

const char* const actionNames[ACT_COUNT] =
  { "FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP", "SLOW", "MEDIUM", "FAST",
    "AUTO MODE", "LINE TRACK", "FOLLOW MODE", "LIGHT SEEK" };

// Defaults = LAFVIN logo remote from Keymap.cpp
const unsigned char defaultCodes[ACT_COUNT] =
  { 0x46, 0x15, 0x44, 0x43, 0x40, 0x16, 0x19, 0x0D, 0x0C, 0x18, 0x5A, 0x52 };

unsigned char keyCodes[ACT_COUNT];
int learnStep = -1;   // -1 = not learning

// ---------------- Settings ----------------
int STOP_DISTANCE_CM = 25;        // closer than this counts as an obstacle
int HITS_TO_STOP     = 2;         // readings in a row needed before stopping (higher = less sensitive)

bool sensorEnabled = true;   // obstacle stop while driving forward (auto mode always uses the sensor)
bool sweepEnabled  = true;   // sweep the sensor left and right while driving

// Sensor sweep while driving forward (90 = straight ahead)
const int SWEEP_MIN  = 65;
const int SWEEP_MAX  = 115;
const int SWEEP_STEP = 5;
const unsigned long SWEEP_INTERVAL = 45; // ms per sweep step

// Auto mode look-around angles. If the car turns toward the obstacle, swap these two numbers.
const int SERVO_LEFT  = 150;
const int SERVO_RIGHT = 30;
const int TURN_MS        = 350;          // how long to turn after looking (tune for your car)
const int TURN_AROUND_MS = 700;

const int SPEED_SLOW = 120;
const int SPEED_MED  = 180;
const int SPEED_FAST = 255;

// ---------------- State ----------------
enum Move { STOPPED, FORWARD, BACKWARD, TURN_LEFT, TURN_RIGHT,
            FWD_LEFT, FWD_RIGHT, BACK_LEFT, BACK_RIGHT };
Move currentMove = STOPPED;
int carSpeed = SPEED_MED;
bool autoMode = false;
bool trackMode = false;
bool followMode = false;
bool lightMode = false;
bool trackInverted = false; // false = HIGH means line (black), true = LOW means line
int lastTrackTurn = 0;      // -1 = left, 1 = right, 0 = straight

enum TrackState {
  TRACK_INIT,      // just started
  TRACK_FOLLOWING, // actively on line
  TRACK_SCAN_L,    // line lost / absent: scanning left
  TRACK_SCAN_R,    // scanning right
  TRACK_SCAN_FULL, // return sweep
  TRACK_LOST       // no line detected anywhere, stopped
};
TrackState trackState = TRACK_INIT;
unsigned long trackStateTimer = 0;
unsigned long lastLineSeenTimer = 0;
int lastLineDir = -1; // -1 = left, 1 = right
bool lineWasDetected = false;
unsigned long lastSweep = 0;
int sweepAngle = 90;
int sweepDir = 1;
int hitCount = 0;
bool headManual = false;   // true while the sensor is aimed from the remote

Servo headServo;

// ---------------- Built-in NEC IR decoder (external interrupt) ----------------
volatile unsigned long irData = 0;
volatile uint8_t irBits = 0;
volatile bool irReceiving = false;
volatile bool irNewCode = false;
volatile unsigned char irCode = 0;
unsigned long irLastFall = 0;

void irISR() {                            // runs on every falling edge of the IR pin
  unsigned long now = micros();
  unsigned long dt = now - irLastFall;
  irLastFall = now;

  if (dt >= 12500 && dt <= 14500) {       // leader
    irReceiving = true;
    irBits = 0;
    irData = 0;
  } else if (dt >= 10000 && dt < 12500) { // repeat frame, ignore
    irReceiving = false;
  } else if (irReceiving) {
    if (dt >= 800 && dt <= 1600) {
      // bit 0
    } else if (dt >= 1800 && dt <= 2800) {
      irData |= (1UL << irBits);          // bit 1 (LSB first)
    } else {
      irReceiving = false;
      return;
    }
    irBits++;
    if (irBits == 32) {
      irReceiving = false;
      irCode = (irData >> 16) & 0xFF;
      irNewCode = true;
    }
  }
}

// ---------------- Saved codes ----------------
void loadCodes() {
  bool saved = (EEPROM.read(0) == 0xA5);
  for (int i = 0; i < ACT_COUNT; i++) {
    byte b = saved ? EEPROM.read(1 + i) : defaultCodes[i];
    keyCodes[i] = (b == 0xFF || b == 0x00) ? defaultCodes[i] : b;
  }
}

void saveCodes() {
  EEPROM.update(0, 0xA5);
  for (int i = 0; i < ACT_COUNT; i++) EEPROM.update(1 + i, keyCodes[i]);
}

// ---------------- Motor functions (TB6612FNG) ----------------
//***************************************************************
// TB6612FNG PIN STATES                         MOTOR DIRECTIONS
//***************************************************************
//  LDIO  LPWM  RDIO  RPWM  RESULT              LEFT      RIGHT
//  HIGH  rate  LOW   rate  forward             forward   forward (BLUE  BLUE)
//  LOW   rate  HIGH  rate  reverse             reverse   reverse (GREEN GREEN)
//  LOW   rate  LOW   rate  rotate left         reverse   forward (GREEN BLUE)
//  HIGH  rate  HIGH  rate  rotate right        forward   reverse (BLUE  GREEN)
//  LOW   LOW   LOW   rate  skid forward left   stop      forward (OFF   BLUE)
//  HIGH  rate  LOW   LOW   skid forward right  forward   stop    (BLUE  OFF)
//  LOW   rate  LOW   LOW   skid reverse left   reverse   stop    (GREEN OFF)
//  HIGH  LOW   HIGH  rate  skid reverse right  stop      reverse (OFF   GREEN)
//  LOW   LOW   LOW   LOW   stop                stop      stop    (OFF   OFF)
//***************************************************************
// Motor polarity configuration. Set to false if motors now follow standard polarity.
const bool LEFT_MOTOR_REVERSED  = false;
const bool RIGHT_MOTOR_REVERSED = false;

void drive(bool leftDir, int leftRate, bool rightDir, int rightRate) {
  digitalWrite(LEFT_DIR,  leftRate  > 0 ? (leftDir  != LEFT_MOTOR_REVERSED)  : LOW);
  digitalWrite(RIGHT_DIR, rightRate > 0 ? (rightDir != RIGHT_MOTOR_REVERSED) : LOW);
  analogWrite(LEFT_PWM,  leftRate);
  analogWrite(RIGHT_PWM, rightRate);
}

void forward() {
#ifdef DEBUG
  Serial.println(F("BLUE  BLUE"));
#endif
  drive(HIGH, carSpeed, LOW, carSpeed);
  currentMove = FORWARD;
}

void backward() {
#ifdef DEBUG
  Serial.println(F("GREEN GREEN"));
#endif
  drive(LOW, carSpeed, HIGH, carSpeed);
  currentMove = BACKWARD;
}

void turnLeft() {
#ifdef DEBUG
  Serial.println(F("GREEN BLUE"));
#endif
  drive(LOW, carSpeed, LOW, carSpeed);
  currentMove = TURN_LEFT;
}

void turnRight() {
#ifdef DEBUG
  Serial.println(F("BLUE  GREEN"));
#endif
  drive(HIGH, carSpeed, HIGH, carSpeed);
  currentMove = TURN_RIGHT;
}

void forward_skid_left() {
#ifdef DEBUG
  Serial.println(F("OFF   BLUE"));
#endif
  drive(LOW, 0, LOW, carSpeed);
  currentMove = FWD_LEFT;
}

void forward_skid_right() {
#ifdef DEBUG
  Serial.println(F("BLUE  OFF"));
#endif
  drive(HIGH, carSpeed, LOW, 0);
  currentMove = FWD_RIGHT;
}

void reverse_skid_left() {
#ifdef DEBUG
  Serial.println(F("GREEN OFF"));
#endif
  drive(LOW, carSpeed, LOW, 0);
  currentMove = BACK_LEFT;
}

void reverse_skid_right() {
#ifdef DEBUG
  Serial.println(F("OFF   GREEN"));
#endif
  drive(HIGH, 0, HIGH, carSpeed);
  currentMove = BACK_RIGHT;
}

void stopCar() {
#ifdef DEBUG
  if (currentMove != STOPPED) Serial.println(F("OFF   OFF"));
#endif
  drive(LOW, 0, LOW, 0);
  currentMove = STOPPED;
}

// ---------------- Motor & Rate-Change Self-Test (LAFVIN Tutorial) ----------------
void runMotorTest() {
  autoMode = false;
  trackMode = false;
  followMode = false;
  lightMode = false;
  stopCar();
  Serial.println(F("=== TB6612FNG MOTOR & RATE TEST ==="));
  int saved = carSpeed;
  carSpeed = 80;

  forward();            delay(400);
  backward();           delay(400);
  stopCar();            delay(200);
  turnLeft();           delay(400);
  turnRight();          delay(400);
  stopCar();            delay(200);
  forward_skid_left();  delay(350);
  forward_skid_right(); delay(350);
  reverse_skid_left();  delay(350);
  reverse_skid_right(); delay(350);
  stopCar();            delay(200);

  // Rate change ramp (50 -> 250 -> 50)
  Serial.println(F("= BLUE LED (Rate Ramp) ="));
  for (int i = 50; i < 250; i += 25) { drive(HIGH, i, LOW, i); delay(60); }
  for (int i = 50; i < 250; i += 25) { drive(HIGH, 250 - i, LOW, 250 - i); delay(60); }
  stopCar();
  carSpeed = saved;

  // Brief end-of-test blink on LED_BUILTIN (D13), then restore ECHO_PIN as INPUT
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < 6; i++) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(80);
  }
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(ECHO_PIN, INPUT);
  Serial.println(F("=== MOTOR TEST DONE ==="));
}

// ---------------- Battery health (internal 1.1V bandgap or optional A4 divider) ----------------
// shortcut: without an A4 resistor divider, Uno reads its regulated 5V rail (sags once pack drops <~6.2V).
int readBatteryMv() {
#if USE_BATT_DIVIDER
  return (int)((long)analogRead(BATT_PIN) * 10000L / 1023L); // 1:1 divider -> 0-10V in mV
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);    // measure 1.1V bandgap against AVcc
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC));
  uint8_t low  = ADCL;
  uint8_t high = ADCH;
  long res = (high << 8) | low;
  return res > 0 ? (int)(1125300L / res) : 0;                // AVcc in mV
#endif
}

// ---------------- Obstacle sensor (D12 = TRIG, D13 / LED_BUILTIN = ECHO) ----------------
int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(4);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 15000UL);
  if (duration == 0) return 999;
  int cm = (int)(duration / 58);
  return cm < 1 ? 1 : cm;
}

// ---------------- Smarter obstacle detection (servo sweep) ----------------
int readDistance() {
  return getDistance();
}

int median3(int a, int b, int c) {
  int lo = min(a, b), hi = max(a, b);
  return max(lo, min(hi, c));
}

void aimHead(int angle) {
  angle = constrain(angle, 0, 180);
  sweepAngle = angle;                       // sweepAngle always tracks where the servo really is
  hitCount = 0;
  headServo.write(angle);
}

void centerHead() {
  if (headManual) return;                   // keep the angle chosen from the remote
  if (sweepAngle != 90) { sweepDir = 1; aimHead(90); }
}

// Quick check before driving off: 3 readings straight ahead, blocked only if 2 or more are close
bool pathClear() {
  if ((!sensorEnabled && !autoMode) || headManual) return true;
  if (sweepAngle != 90) { centerHead(); delay(150); }
  int hits = 0;
  for (int i = 0; i < 3; i++) {
    if (readDistance() <= STOP_DISTANCE_CM) hits++;
    delay(25);
  }
  return hits < 2;
}

// Call this often while driving forward. Confirms obstacles immediately at the current angle
// before rotating the servo to the next sweep step.
bool obstacleAhead() {
  if (millis() - lastSweep < SWEEP_INTERVAL) return false;
  lastSweep = millis();

  if (readDistance() <= STOP_DISTANCE_CM) {
    int hits = 1;
    for (int i = 1; i < HITS_TO_STOP; i++) {
      delay(10);
      if (readDistance() <= STOP_DISTANCE_CM) hits++;
      else break;
    }
    if (hits >= HITS_TO_STOP) {
      hitCount = 0;
      return true;
    }
  }
  hitCount = 0;

  if (sweepEnabled || autoMode) {
    sweepAngle += sweepDir * SWEEP_STEP;
    if (sweepAngle >= SWEEP_MAX)      { sweepAngle = SWEEP_MAX; sweepDir = -1; }
    else if (sweepAngle <= SWEEP_MIN) { sweepAngle = SWEEP_MIN; sweepDir = 1; }
    headServo.write(sweepAngle);
  } else if (sweepAngle != 90) {
    sweepAngle = 90;
    headServo.write(90);
  }

  return false;
}

// Point the sensor at an angle and return the median of 3 readings
int scanAt(int angle) {
  headServo.write(angle);
  delay(350);
  int a = readDistance(); delay(40);
  int b = readDistance(); delay(40);
  int c = readDistance();
  return median3(a, b, c);
}

bool isForwardMove() {
  return currentMove == FORWARD || currentMove == FWD_LEFT || currentMove == FWD_RIGHT;
}

void reapplyMove() {
  switch (currentMove) {
    case FORWARD:    forward();            break;
    case BACKWARD:   backward();           break;
    case TURN_LEFT:  turnLeft();           break;
    case TURN_RIGHT: turnRight();          break;
    case FWD_LEFT:   forward_skid_left();  break;
    case FWD_RIGHT:  forward_skid_right(); break;
    case BACK_LEFT:  reverse_skid_left();  break;
    case BACK_RIGHT: reverse_skid_right(); break;
    default: break;
  }
}

void doDiagonal(Move m) {
  autoMode = false;
  trackMode = false;
  followMode = false;
  lightMode = false;
  if (m == FWD_LEFT || m == FWD_RIGHT) {
    headManual = false;
    if (!pathClear()) { stopCar(); Serial.println(F("Blocked: obstacle ahead")); return; }
  }
  switch (m) {
    case FWD_LEFT:   Serial.println(F("-> UPPER LEFT"));  forward_skid_left();  break;
    case FWD_RIGHT:  Serial.println(F("-> UPPER RIGHT")); forward_skid_right(); break;
    case BACK_LEFT:  Serial.println(F("-> LOWER LEFT"));  reverse_skid_left();  break;
    case BACK_RIGHT: Serial.println(F("-> LOWER RIGHT")); reverse_skid_right(); break;
    default: break;
  }
}

// ---------------- Line tracking (LAFVIN Infrared_Tracing with Scan Recovery) ----------------
bool lineSensorActive() {
  int Left = digitalRead(TRACK_LEFT_PIN);
  int Right = digitalRead(TRACK_RIGHT_PIN);
  int Mid = digitalRead(TRACK_MID_PIN);
  int Black = trackInverted ? 0 : 1;
  return (Left == Black || Right == Black || Mid == Black);
}

void runLineTracking() {
  int Left_Tra_Value = digitalRead(TRACK_LEFT_PIN);
  int Right_Tra_Value = digitalRead(TRACK_RIGHT_PIN);
  int Mid_Tra_Value = digitalRead(TRACK_MID_PIN);
  int Black = trackInverted ? 0 : 1;

  // Stop if obstacle detected ahead while tracking
  if (sensorEnabled && readDistance() <= STOP_DISTANCE_CM) {
    stopCar();
    return;
  }

  // Speeds
  int fwdSpeed = 50;
  int turnSpeed = 65;
  int scanSpeed = 65;
  if (carSpeed > SPEED_MED) {
    fwdSpeed = 65;
    turnSpeed = 75;
    scanSpeed = 75;
  }

  bool leftHit  = (Left_Tra_Value == Black);
  bool rightHit = (Right_Tra_Value == Black);
  bool midHit   = (Mid_Tra_Value == Black);

  // If any sensor touches the line:
  if (leftHit || rightHit || midHit) {
    lastLineSeenTimer = millis();

    // Recovered from scan state:
    if (trackState != TRACK_FOLLOWING) {
      trackState = TRACK_FOLLOWING;
      lineWasDetected = true;
      Serial.println(F("Line found: resuming tracking"));
    }

    if (leftHit && !rightHit) {
      lastLineDir = -1;
      drive(LOW, turnSpeed, LOW, turnSpeed);
      currentMove = TURN_LEFT;
    } else if (rightHit && !leftHit) {
      lastLineDir = 1;
      drive(HIGH, turnSpeed, HIGH, turnSpeed);
      currentMove = TURN_RIGHT;
    } else if (leftHit && rightHit) {
      stopCar();
    } else if (midHit) {
      drive(HIGH, fwdSpeed, LOW, fwdSpeed);
      currentMove = FORWARD;
    }
    return;
  }

  // Neither sensor currently on black:
  if (trackState == TRACK_FOLLOWING) {
    // Straddling line moving forward
    drive(HIGH, fwdSpeed, LOW, fwdSpeed);
    currentMove = FORWARD;

    // Traveled forward without touching line -> line is gone, start scan
    if (millis() - lastLineSeenTimer > 650) {
      trackState = (lastLineDir == -1) ? TRACK_SCAN_L : TRACK_SCAN_R;
      trackStateTimer = millis();
      Serial.println(F("Line gone: scanning for line..."));
    }
  } else if (trackState == TRACK_SCAN_L) {
    // Rotate left to scan for line
    drive(LOW, scanSpeed, LOW, scanSpeed);
    currentMove = TURN_LEFT;
    if (millis() - trackStateTimer > 450) {
      trackState = TRACK_SCAN_R;
      trackStateTimer = millis();
      Serial.println(F("Scanning: rotating right..."));
    }
  } else if (trackState == TRACK_SCAN_R) {
    // Rotate right across original heading
    drive(HIGH, scanSpeed, HIGH, scanSpeed);
    currentMove = TURN_RIGHT;
    if (millis() - trackStateTimer > 900) {
      trackState = TRACK_SCAN_FULL;
      trackStateTimer = millis();
      Serial.println(F("Scanning: returning..."));
    }
  } else if (trackState == TRACK_SCAN_FULL) {
    // Rotate back toward center
    drive(LOW, scanSpeed, LOW, scanSpeed);
    currentMove = TURN_LEFT;
    if (millis() - trackStateTimer > 500) {
      // Completed full scan, no line found anywhere
      stopCar();
      trackState = TRACK_LOST;
      trackMode = false;
      Serial.println(F("No line detected: stopped"));
    }
  } else if (trackState == TRACK_LOST) {
    stopCar();
  }
}

// ---------------- Ultrasonic Follow Mode (LAFVIN official) ----------------
void runFollowMode() {
  int Infrared_Trigger_Flag = 0;
  int Left_IR_Value = digitalRead(FOLLOW_LEFT_IR);
  int Right_IR_Value = digitalRead(FOLLOW_RIGHT_IR);
  float Front_Distance = getDistance();

  int baseSpd = constrain(carSpeed, 80, 140);
  int revSpd  = baseSpd * 3 / 4;
  int curveLow = baseSpd / 4;

  if (Front_Distance < 5 && (Left_IR_Value != Infrared_Trigger_Flag && Right_IR_Value != Infrared_Trigger_Flag)) {
    drive(LOW, revSpd, HIGH, revSpd);
    currentMove = BACKWARD;
  } else if (Front_Distance < 5 && (Left_IR_Value == Infrared_Trigger_Flag && Right_IR_Value != Infrared_Trigger_Flag)) {
    drive(LOW, baseSpd, HIGH, curveLow);
    currentMove = BACK_LEFT;
  } else if (Front_Distance < 5 && (Left_IR_Value != Infrared_Trigger_Flag && Right_IR_Value == Infrared_Trigger_Flag)) {
    drive(LOW, curveLow, HIGH, baseSpd);
    currentMove = BACK_RIGHT;
  } else if (Front_Distance < 5 && (Left_IR_Value == Infrared_Trigger_Flag && Right_IR_Value == Infrared_Trigger_Flag)) {
    drive(LOW, revSpd, HIGH, revSpd);
    currentMove = BACKWARD;
  } else if (Front_Distance > 10 && Front_Distance <= 40 && (Left_IR_Value != Infrared_Trigger_Flag && Right_IR_Value != Infrared_Trigger_Flag)) {
    drive(HIGH, baseSpd, LOW, baseSpd);
    currentMove = FORWARD;
  } else if (Front_Distance > 10 && (Left_IR_Value == Infrared_Trigger_Flag && Right_IR_Value != Infrared_Trigger_Flag)) {
    drive(LOW, baseSpd, LOW, baseSpd);
    currentMove = TURN_LEFT;
  } else if (Front_Distance > 10 && (Left_IR_Value != Infrared_Trigger_Flag && Right_IR_Value == Infrared_Trigger_Flag)) {
    drive(HIGH, baseSpd, HIGH, baseSpd);
    currentMove = TURN_RIGHT;
  } else if ((5 <= Front_Distance && Front_Distance <= 10) && (Left_IR_Value != Infrared_Trigger_Flag && Right_IR_Value == Infrared_Trigger_Flag)) {
    drive(HIGH, baseSpd, LOW, curveLow);
    currentMove = FWD_RIGHT;
  } else if ((5 <= Front_Distance && Front_Distance <= 10) && (Left_IR_Value == Infrared_Trigger_Flag && Right_IR_Value != Infrared_Trigger_Flag)) {
    drive(HIGH, curveLow, LOW, baseSpd);
    currentMove = FWD_LEFT;
  } else {
    stopCar();
  }
}

// ---------------- Light Seeking Mode (LAFVIN official) ----------------
void runLightSeeking() {
  float Left_photosensitive = analogRead(LIGHT_LEFT_PIN) / 10.0;
  float Right_photosensitive = analogRead(LIGHT_RIGHT_PIN) / 10.0;

  if (sensorEnabled && readDistance() <= STOP_DISTANCE_CM) {
    stopCar();
    return;
  }

  // Both sensors dark -> stop
  if (Left_photosensitive > 55.0 && Right_photosensitive > 55.0) {
    stopCar();
    return;
  }

  int speed_value = constrain(carSpeed, 80, 160);
  int Lightseeking_Degree = 90;

  if (Left_photosensitive > Right_photosensitive && Left_photosensitive > 0.1) {
    Lightseeking_Degree = (int)((Right_photosensitive / Left_photosensitive) * 90.0);
  } else if (Left_photosensitive <= Right_photosensitive && Right_photosensitive > 0.1) {
    Lightseeking_Degree = 180 - (int)((Left_photosensitive / Right_photosensitive) * 90.0);
  }

  if (Lightseeking_Degree < 90) {
    float f = (float)Lightseeking_Degree / 90.0;
    drive(HIGH, speed_value, LOW, (int)(speed_value * f));
    currentMove = FWD_RIGHT;
  } else if (Lightseeking_Degree > 90) {
    float f = (float)(180 - Lightseeking_Degree) / 90.0;
    drive(HIGH, (int)(speed_value * f), LOW, speed_value);
    currentMove = FWD_LEFT;
  } else {
    drive(HIGH, speed_value, LOW, speed_value);
    currentMove = FORWARD;
  }
}

// ---------------- Actions ----------------
void doAction(int a) {
  Serial.print(F("-> "));
  Serial.println(actionNames[a]);

  switch (a) {
    case ACT_FWD:
      autoMode = false;
      trackMode = false;
      followMode = false;
      lightMode = false;
      headManual = false;
      if (pathClear()) forward();
      else { stopCar(); Serial.println(F("Blocked: obstacle ahead")); }
      break;
    case ACT_BACK:  autoMode = false; trackMode = false; followMode = false; lightMode = false; backward();  break;
    case ACT_LEFT:  autoMode = false; trackMode = false; followMode = false; lightMode = false; turnLeft();  break;
    case ACT_RIGHT: autoMode = false; trackMode = false; followMode = false; lightMode = false; turnRight(); break;
    case ACT_STOP:  autoMode = false; trackMode = false; followMode = false; lightMode = false; stopCar();   break;
    case ACT_SLOW:  carSpeed = SPEED_SLOW; reapplyMove(); break;
    case ACT_MED:   carSpeed = SPEED_MED;  reapplyMove(); break;
    case ACT_FAST:  carSpeed = SPEED_FAST; reapplyMove(); break;
    case ACT_AUTO:
      trackMode = false;
      followMode = false;
      lightMode = false;
      autoMode = !autoMode;
      if (autoMode) {
        headManual = false;
        centerHead();
        if (pathClear()) forward();
        else avoidObstacle();
      } else {
        stopCar();
      }
      break;
    case ACT_TRACK:
      autoMode = false;
      followMode = false;
      lightMode = false;
      trackMode = !trackMode;
      if (trackMode) {
        headManual = false;
        centerHead();
        trackStateTimer = millis();
        lastLineSeenTimer = millis();
        lastLineDir = -1;
        lineWasDetected = false;
        if (lineSensorActive()) {
          lineWasDetected = true;
          trackState = TRACK_FOLLOWING;
          Serial.println(F("Line detected: starting tracking"));
        } else {
          trackState = TRACK_SCAN_L;
          Serial.println(F("No line detected under sensors: scanning to find line..."));
        }
      } else {
        stopCar();
        Serial.println(F("Line tracking OFF"));
      }
      break;
    case ACT_FOLLOW:
      autoMode = false;
      trackMode = false;
      lightMode = false;
      followMode = !followMode;
      if (followMode) {
        headManual = false;
        centerHead();
        Serial.println(F("Follow mode ON"));
      } else {
        stopCar();
        Serial.println(F("Follow mode OFF"));
      }
      break;
    case ACT_LIGHT:
      autoMode = false;
      trackMode = false;
      followMode = false;
      lightMode = !lightMode;
      if (lightMode) {
        headManual = false;
        centerHead();
        Serial.println(F("Light seeking ON"));
      } else {
        stopCar();
        Serial.println(F("Light seeking OFF"));
      }
      break;
  }
}

// ---------------- Learn mode ----------------
void promptLearn() {
  Serial.print(F("Press the button for: "));
  Serial.println(actionNames[learnStep]);
}

void startLearn() {
  stopCar();
  autoMode = false;
  learnStep = 0;
  Serial.println(F("LEARN MODE"));
  promptLearn();
}

void learnCode(unsigned char code) {
  keyCodes[learnStep] = code;
  Serial.print(actionNames[learnStep]);
  Serial.print(F(" = 0x"));
  Serial.println(code, HEX);
  learnStep++;
  if (learnStep >= ACT_COUNT) {
    saveCodes();
    learnStep = -1;
    Serial.println(F("Saved. Remote is ready."));
  } else {
    promptLearn();
  }
}

// ---------------- IR code received ----------------
void handleIrCode(unsigned char code) {
  Serial.print(F("IR 0x"));
  Serial.println(code, HEX);

  if (learnStep >= 0) { learnCode(code); return; }

  for (int i = 0; i < ACT_COUNT; i++) {
    if (keyCodes[i] == code) { doAction(i); return; }
  }
  Serial.println(F("(button not assigned, type M to learn your remote)"));
}

// ---------------- PC / Bluetooth remote (serial) ----------------
// Commands are prefixed with '!' so BLE module status text (e.g. +CONNECTED) is ignored.
// Drive letters: !U !D !L !R !O(stop) !0(stop) !1 !2 !3 !4(auto)
// Settings are '!', a letter, a number, then a newline:
//   !V150 speed (60-255)   !T20 stop distance in cm   !H3 confirmations in a row
//   !E1 / !E0 obstacle sensor on / off   !W1 / !W0 sweep on / off   !P = read distance
char numCmd = 0;     // setting letter waiting for its number, 0 = none
int  numVal = -1;    // -1 = no digits received yet
bool cmdArmed = false;

void applySetting(char cmd, int v) {
  switch (cmd) {
    case 'V':
      carSpeed = constrain(v, 60, 255);
      reapplyMove();
      Serial.print(F("Speed ")); Serial.println(carSpeed);
      break;
    case 'T':
      STOP_DISTANCE_CM = constrain(v, 5, 80);
      Serial.print(F("Stop distance ")); Serial.print(STOP_DISTANCE_CM); Serial.println(F(" cm"));
      break;
    case 'H':
      HITS_TO_STOP = constrain(v, 1, 8);
      Serial.print(F("Confirmations ")); Serial.println(HITS_TO_STOP);
      break;
    case 'E':
      sensorEnabled = (v != 0);
      Serial.println(sensorEnabled ? F("Obstacle sensor ON") : F("Obstacle sensor OFF"));
      break;
    case 'A':
      if (autoMode) { Serial.println(F("Turn auto mode off to aim the sensor")); break; }
      headManual = (abs(v - 90) > 10);      // near center = normal behavior again
      aimHead(v);
      Serial.print(F("Sensor angle ")); Serial.print(constrain(v, 0, 180));
      Serial.println(headManual ? F(" (obstacle stop paused)") : F(""));
      break;
    case 'W':
      sweepEnabled = (v != 0);
      Serial.println(sweepEnabled ? F("Sweep ON") : F("Sweep OFF"));
      break;
    case 'J':
      trackInverted = (v != 0);
      Serial.println(trackInverted ? F("Track inverted (LOW=line)") : F("Track normal (HIGH=line)"));
      break;
  }
}

void handleSerialRemote() {
  while (Serial.available()) {
    char c = Serial.read();

    if (numCmd) {                                   // collecting a number for a setting
      if (c >= '0' && c <= '9') {
        numVal = (numVal < 0 ? 0 : numVal) * 10 + (c - '0');
        continue;
      }
      if (numVal >= 0) applySetting(numCmd, numVal); // any other character ends the number
      numCmd = 0;
      if (c == '\n' || c == '\r') continue;
    }

    if (c == '!') { cmdArmed = true; continue; }

    if (!cmdArmed) {
      // Ignore all characters without '!' prefix (discards BLE status text like +MAC:, +CONNECTED, etc.)
      continue;
    }
    cmdArmed = false;

    if (c == 'V' || c == 'T' || c == 'H' || c == 'E' || c == 'W' || c == 'A' || c == 'J') {
      numCmd = c;
      numVal = -1;
      continue;
    }
    if (c == 'P') {
      int d = readDistance();
      Serial.print(F("Distance "));
      if (d >= 999) Serial.println(F("clear"));
      else { Serial.print(d); Serial.println(F(" cm")); }
      continue;
    }
    if (c == 'Q') {
      int mv = readBatteryMv();
      int d  = readDistance();
      Serial.print(F("TEL:"));
      Serial.print(mv); Serial.print(',');
      Serial.print(d);  Serial.print(',');
      Serial.print(carSpeed); Serial.print(',');
      Serial.print(currentMove != STOPPED ? 1 : 0); Serial.print(',');
      Serial.println(USE_BATT_DIVIDER);
      continue;
    }
    if (c == 'K') {
      int Left_Tra_Value = digitalRead(TRACK_LEFT_PIN);
      int Right_Tra_Value = digitalRead(TRACK_RIGHT_PIN);
      int Center_Tra_Value = digitalRead(TRACK_MID_PIN);
      Serial.print(F("Left Tracking value: "));
      Serial.println(Left_Tra_Value);
      Serial.print(F("Right Tracking value: "));
      Serial.println(Right_Tra_Value);
      Serial.print(F("Center: "));
      Serial.print(Center_Tra_Value);
      Serial.print(F(" (Black="));
      Serial.print(trackInverted ? 0 : 1);
      Serial.println(F(")"));
      continue;
    }
    if (c == 'Z') {
      float leftLDR = analogRead(LIGHT_LEFT_PIN);
      float rightLDR = analogRead(LIGHT_RIGHT_PIN);
      int leftIR = digitalRead(FOLLOW_LEFT_IR);
      int rightIR = digitalRead(FOLLOW_RIGHT_IR);
      Serial.print(F("Light L=")); Serial.print(leftLDR);
      Serial.print(F(" R=")); Serial.print(rightLDR);
      Serial.print(F(" | Follow IR L=")); Serial.print(leftIR);
      Serial.print(F(" R=")); Serial.println(rightIR);
      continue;
    }
    if (c == 'S') { runMotorTest(); continue; }
    if (c == 'M') { startLearn(); continue; }
    if (c == 'X') {
      EEPROM.update(0, 0);
      loadCodes();
      learnStep = -1;
      Serial.println(F("Default remote codes restored."));
      continue;
    }
    if (c == 'Y') {
      Serial.print(F("AT+NAMECD PRINTS CAR\r\n"));
      delay(100);
      Serial.println(F("Bluetooth name set to: CD PRINTS CAR"));
      continue;
    }
    if (c == 'O' || c == '0') {
      learnStep = -1; // stop also cancels learn mode if active
      doAction(ACT_STOP);
      continue;
    }
    if (learnStep >= 0) continue;                   // ignore drive letters while learning
    switch (c) {
      case 'U': doAction(ACT_FWD);     break;
      case 'D': doAction(ACT_BACK);    break;
      case 'L': doAction(ACT_LEFT);    break;
      case 'R': doAction(ACT_RIGHT);   break;
      case 'F': doDiagonal(FWD_LEFT);   break;
      case 'G': doDiagonal(FWD_RIGHT);  break;
      case 'B': doDiagonal(BACK_LEFT);  break;
      case 'N': doDiagonal(BACK_RIGHT); break;
      case 'O':
      case '0': doAction(ACT_STOP);    break;
      case '1': doAction(ACT_SLOW);  break;
      case '2': doAction(ACT_MED);   break;
      case '3': doAction(ACT_FAST);   break;
      case '4': doAction(ACT_AUTO);   break;
      case '5': doAction(ACT_TRACK);  break;
      case '6': doAction(ACT_FOLLOW); break;
      case '7': doAction(ACT_LIGHT);  break;
      default: break;
    }
  }
}

// ---------------- Auto obstacle avoidance ----------------
void avoidObstacle() {
  stopCar();
  delay(150);

  const int angles[4] = { SERVO_LEFT, 120, 60, SERVO_RIGHT };
  int bestIdx = -1;
  int bestDist = STOP_DISTANCE_CM;

  for (int i = 0; i < 4; i++) {
    handleSerialRemote();
    if (!autoMode) { headServo.write(90); sweepAngle = 90; return; }
    int d = scanAt(angles[i]);
    Serial.print(F("Scan ")); Serial.print(angles[i]);
    Serial.print(F("deg: ")); Serial.print(d); Serial.println(F(" cm"));
    if (d > bestDist) {
      bestDist = d;
      bestIdx = i;
    }
  }

  headServo.write(90);
  sweepAngle = 90; sweepDir = 1; hitCount = 0;
  delay(250);

  handleSerialRemote();
  if (!autoMode) return;

  int savedSpeed = carSpeed;
  if (carSpeed < SPEED_MED) carSpeed = SPEED_MED;

  if (bestIdx < 0) {
    backward();  delay(500);                 // boxed in, back out and turn around
    turnRight(); delay(TURN_AROUND_MS);
  } else {
    int turnTime = (bestIdx == 0 || bestIdx == 3) ? TURN_MS : (TURN_MS * 3 / 4);
    if (angles[bestIdx] > 90) turnLeft();
    else                      turnRight();
    delay(turnTime);
  }
  stopCar();
  carSpeed = savedSpeed;
  delay(120);

  if (autoMode && pathClear()) forward();
}

void runAutoMode() {
  if (currentMove == FORWARD) {
    if (obstacleAhead()) avoidObstacle();
  } else if (currentMove == STOPPED) {
    if (pathClear()) forward();
    else avoidObstacle();
  }
}

// ---------------- Setup / Loop ----------------
void setup() {
  Serial.begin(9600);

  pinMode(LEFT_DIR, OUTPUT);  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT); pinMode(RIGHT_PWM, OUTPUT);
  stopCar();
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(TRACK_LEFT_PIN, INPUT);
  pinMode(TRACK_MID_PIN, INPUT);
  pinMode(TRACK_RIGHT_PIN, INPUT);

  pinMode(FOLLOW_LEFT_IR, INPUT);
  pinMode(FOLLOW_RIGHT_IR, INPUT);
  pinMode(LIGHT_LEFT_PIN, INPUT);
  pinMode(LIGHT_RIGHT_PIN, INPUT);

  headServo.attach(SERVO_PIN);
  headServo.write(90);
  delay(300);

  loadCodes();

  pinMode(IR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IR_PIN), irISR, FALLING);

  learnStep = -1;

  // Configure Bluetooth broadcast name to "CD PRINTS CAR" (JDY-16 AT command)
  delay(300);
  Serial.print(F("AT+NAMECD PRINTS CAR\r\n"));
  delay(150);
  while (Serial.available()) Serial.read(); // flush any +OK / +MAC: boot noise
  learnStep = -1;

  Serial.println(F("LAFVIN 2WD car ready. Use !M to teach remote, !Y for BLE name."));
}

void loop() {
  handleSerialRemote();

  if (irNewCode) {
    noInterrupts();
    unsigned char code = irCode;
    irNewCode = false;
    interrupts();
    handleIrCode(code);
  }

  if (learnStep >= 0) return;               // no driving while learning

  if (trackMode) {
    runLineTracking();
  } else if (followMode) {
    runFollowMode();
  } else if (lightMode) {
    runLightSeeking();
  } else if (autoMode) {
    runAutoMode();
  } else if (isForwardMove() && sensorEnabled && !headManual) {
    if (obstacleAhead()) {
      stopCar();
      Serial.println(F("Obstacle detected, stopped"));
    }
  } else {
    centerHead();                           // sensor looks straight ahead when not driving forward
  }
}
