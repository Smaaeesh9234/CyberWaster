// PIN Definitons
#define STEP_PIN 3   // A4988 pin
#define DIR_PIN 4   // A4988 pin
#define ENABLE_PIN 5   // A4988 ENABLE pin (LOW = enabled)

//     Sweep Functionality:
// Sweep ±90° front facing
#define STEPS_PER_REV 200
#define STEP_DELAY_US 1500 // Adjust as needed
#define SWEEP_DEGREES 180
#define STEPS_PER_SWEEP     (STEPS_PER_REV * SWEEP_DEGREES / 360) // should make about 100 distinct steps, more than enough. (Adjust as needed)

// Define distance at which an obstacle is an obstacle
#define OBSTACLE_DISTANCE_CM  50 //Adjust as needed

// Set up serial session with the LiDAR
#define LUNA_SERIAL   Serial1
#define LUNA_BAUD     115200

// Variable definitons
int currentSweep = 0;
bool sweepForward = True;
float degPerStep = SWEEP_DEGREES/STEPS_PER_SWEEP

void setup() {
  //Output Data to PC (testing):
  Serial.begin(115200);
  Serial.println("=== LiDAR Sweep Scanner Starting ===");
  Serial.println("Format: Angle(deg), Distance(cm), Status");
  
  // Start TF-Luna serial
  LUNA_SERIAL.begin(LUNA_BAUD);
  pinMode(STEP_PIN,   OUTPUT);
  pinMode(DIR_PIN,    OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(ENABLE_PIN, LOW);

  //Choose starting sweep direction, True = positive direction first
  setDirection(true);

  delay(500);
}

// main but c++
void loop(){ 
  //read distance from the TF luna LiDAR
  int distance = readTFLuna();

  //variable to contain current angle
  float angle = -90.0 + (currentStep * degPerStep);

  // [Insert data output here]
  
  //Move one step
  stepMotor();

  // Update sweep step and direction if needed.
  if (sweepForward) {
    currentStep++;
    if (currentStep >= STEPS_PER_SWEEP) {
      sweepForward = false;
      setDirection(false);        // Reverse direction
    }
  } else {
    currentStep--;
    if (currentStep <= 0) {
      sweepForward = true;
      setDirection(true);         // Forward direction
    }
  }
}

// forward = true -> sweep left to right, forward = false -> right to left
// May want to make this an input that can be adjusted in real time to change direction to direction of movement
void setDirection(bool forward) {
  digitalWrite(DIR_PIN, forward ? HIGH : LOW);
  delayMicroseconds(5); // Small delay after changing direction (required by A4988)
}

// move the motor 1 step for the scan
void stepMotor() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_DELAY_US);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(STEP_DELAY_US);
}

//read the distance from the LiDAR
int readTFLuna() {
  // Timeout after ~50ms
  unsigned long timeout = millis() + 50;
  while (millis() < timeout) {
    if (LUNA_SERIAL.available() >= 9) {
      // Look for the two-byte header: 0x59 0x59
      if (LUNA_SERIAL.read() == 0x59) {
        if (LUNA_SERIAL.read() == 0x59) {
          // Read the remaining 7 bytes
          byte buf[7];
          for (int i = 0; i < 7; i++) {
            buf[i] = LUNA_SERIAL.read();
          }

          // buf[0] = distance low byte
          // buf[1] = distance high byte
          int dist = buf[0] + (buf[1] << 8);  // Combine into 16-bit int

          // Checksum = lower 8 bits of sum of all bytes 0x59+0x59+buf[0..5]
          // helps counter invalid data inputs
          byte checksum = 0x59 + 0x59;
          for (int i = 0; i < 6; i++) {
            checksum += buf[i];
          }
          // buf[6] is the checksum byte from the sensor
          if (checksum == buf[6]) {
            return dist;  // Valid reading in cm
          }
          // Checksum failed - flush and try again
        }
      }
    }
  }

  return 0; // in case of time out or bad reading
}
