// ============================================================
//  Full System — With Kill Switch + Startup LED Test
//  Kill switch on Pin 2 (interrupt) halts all movement
// ============================================================

#include <HX711.h>

// --- STEPPER MOTOR PINS ---
#define STEP_PIN        3
#define DIR_PIN         4
#define ENABLE_PIN      5

// --- ULTRASONIC PINS ---
#define TRIG_PIN        6
#define ECHO_PIN        7

// --- HX711 PINS ---
#define HX711_DT        8
#define HX711_SCK       9

// --- LED PINS ---
#define LED_LIDAR       10  // Red
#define LED_ULTRASONIC  11  // Green
#define LED_WEIGHT      12  // Blue

// --- KILL SWITCH PIN ---
#define KILL_SWITCH_PIN 2   // Must be pin 2 or 3 for interrupt on MEGA

// --- LIDAR SERIAL ---
#define LUNA_SERIAL     Serial1
#define LUNA_BAUD       115200

// --- MOTOR SETTINGS ---
#define STEPS_PER_SWEEP 100
#define STEP_DELAY_US   800

// --- THRESHOLDS ---
#define OBSTACLE_CM     30
#define ULTRASONIC_INCH 5.0
#define WEIGHT_LBS      10.0

// --- CALIBRATION ---
#define CALIBRATION_FACTOR -7050.0

// --- GLOBALS ---
int   currentStep  = 0;
bool  sweepForward = true;
float degPerStep   = (float)180 / (float)STEPS_PER_SWEEP;
volatile bool killed = false;  // Kill switch flag — volatile because used in interrupt

HX711 scale;

// ============================================================
//  KILL SWITCH INTERRUPT
//  Triggered instantly when button is pressed
// ============================================================
void killSwitch() {
  killed = !killed;  // Toggle on/off so button can resume too
  if (killed) {
    digitalWrite(ENABLE_PIN, HIGH);   // Disable motor immediately
    digitalWrite(LED_LIDAR,      LOW);
    digitalWrite(LED_ULTRASONIC, LOW);
    digitalWrite(LED_WEIGHT,     LOW);
    Serial.println("!!! KILL SWITCH ACTIVATED — System paused !!!");
  } else {
    digitalWrite(ENABLE_PIN, LOW);    // Re-enable motor
    Serial.println("--- System resumed ---");
  }
}

// ============================================================
//  STARTUP LED SEQUENCE — Red → Green → Blue
// ============================================================
void startupLEDTest() {
  Serial.println("Running LED test...");

  // Red
  digitalWrite(LED_LIDAR, HIGH);
  delay(500);
  digitalWrite(LED_LIDAR, LOW);
  delay(200);

  // Green
  digitalWrite(LED_ULTRASONIC, HIGH);
  delay(500);
  digitalWrite(LED_ULTRASONIC, LOW);
  delay(200);

  // Blue
  digitalWrite(LED_WEIGHT, HIGH);
  delay(500);
  digitalWrite(LED_WEIGHT, LOW);
  delay(200);

  // All on together briefly as final confirmation
  digitalWrite(LED_LIDAR,      HIGH);
  digitalWrite(LED_ULTRASONIC, HIGH);
  digitalWrite(LED_WEIGHT,     HIGH);
  delay(500);
  digitalWrite(LED_LIDAR,      LOW);
  digitalWrite(LED_ULTRASONIC, LOW);
  digitalWrite(LED_WEIGHT,     LOW);

  Serial.println("LED test complete!");
  delay(300);
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("=== Full System Starting ===");

  // LiDAR serial
  LUNA_SERIAL.begin(LUNA_BAUD);

  // Motor pins
  pinMode(STEP_PIN,    OUTPUT);
  pinMode(DIR_PIN,     OUTPUT);
  pinMode(ENABLE_PIN,  OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);
  setDirection(true);

  // Ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // LED pins
  pinMode(LED_LIDAR,       OUTPUT);
  pinMode(LED_ULTRASONIC,  OUTPUT);
  pinMode(LED_WEIGHT,      OUTPUT);

  // Kill switch — INPUT_PULLUP means no external resistor needed
  // Interrupt triggers killSwitch() instantly on button press
  pinMode(KILL_SWITCH_PIN, INPUT_PULLUP);
  attachInterrupt(
    digitalPinToInterrupt(KILL_SWITCH_PIN),
    killSwitch,
    FALLING  // Triggers when button is pressed (pin goes LOW)
  );

  // HX711 setup
  scale.begin(HX711_DT, HX711_SCK);
  scale.set_scale(CALIBRATION_FACTOR);
  scale.tare();
  Serial.println("Scale zeroed!");

  // Run startup LED test
  startupLEDTest();

  delay(500);
  Serial.println("Format: Angle, LiDAR(cm), Ultrasonic(in), Weight(lbs)");
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  // Check kill switch — if killed, do nothing until resumed
  if (killed) {
    delay(100);
    return;
  }

  // --- READ LIDAR ---
  int lidarDist = readTFLuna();
  float angle   = -90.0 + (currentStep * degPerStep);

  // --- READ ULTRASONIC ---
  float ultrasonicInch = readUltrasonic();

  // --- READ WEIGHT ---
  float weightLbs = 0;
  if (scale.is_ready()) {
    weightLbs = scale.get_units(1);
    if (weightLbs < 0) weightLbs = 0;
  }

  // --- CONTROL LEDs ---
  digitalWrite(LED_LIDAR,
    (lidarDist > 0 && lidarDist <= OBSTACLE_CM) ? HIGH : LOW);

  digitalWrite(LED_ULTRASONIC,
    (ultrasonicInch > 0 && ultrasonicInch <= ULTRASONIC_INCH) ? HIGH : LOW);

  digitalWrite(LED_WEIGHT,
    (weightLbs >= WEIGHT_LBS) ? HIGH : LOW);

  // --- PRINT STATUS ---
  Serial.print(angle, 1);
  Serial.print("°, ");
  Serial.print(lidarDist > 0 ? String(lidarDist) + "cm" : "no reading");
  Serial.print(", ");
  Serial.print(ultrasonicInch, 1);
  Serial.print("in, ");
  Serial.print(weightLbs, 2);
  Serial.println("lbs");

  // --- STEP MOTOR ---
  stepMotor();

  if (sweepForward) {
    currentStep++;
    if (currentStep >= STEPS_PER_SWEEP) {
      sweepForward = false;
      setDirection(false);
    }
  } else {
    currentStep--;
    if (currentStep <= 0) {
      sweepForward = true;
      setDirection(true);
    }
  }
}

// ============================================================
//  FUNCTION: Read TF-Luna LiDAR
// ============================================================
int readTFLuna() {
  unsigned long timeout = millis() + 50;
  while (millis() < timeout) {
    if (LUNA_SERIAL.available() >= 9) {
      if (LUNA_SERIAL.read() == 0x59) {
        if (LUNA_SERIAL.read() == 0x59) {
          byte buf[7];
          for (int i = 0; i < 7; i++) buf[i] = LUNA_SERIAL.read();
          byte checksum = 0x59 + 0x59;
          for (int i = 0; i < 6; i++) checksum += buf[i];
          if (checksum == buf[6]) return buf[0] + (buf[1] << 8);
        }
      }
    }
  }
  return 0;
}

// ============================================================
//  FUNCTION: Read HC-SR04 Ultrasonic (returns inches)
// ============================================================
float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return -1;
  return duration * 0.0135 / 2.0;
}

// ============================================================
//  FUNCTION: Set stepper direction
// ============================================================
void setDirection(bool forward) {
  digitalWrite(DIR_PIN, forward ? HIGH : LOW);
  delayMicroseconds(200);
}

// ============================================================
//  FUNCTION: Step motor once
// ============================================================
void stepMotor() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_DELAY_US);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(STEP_DELAY_US);
}