void setup() {
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
}

void loop() {
  int speed = 77; // 30% of 255

  // Forward
  digitalWrite(4, HIGH);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);  // Motor 2 flipped
  digitalWrite(7, HIGH);
  analogWrite(9, speed);
  analogWrite(10, speed);
  delay(5000);

  // Brake before reversing
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  delay(200);

  // Reverse
  digitalWrite(4, LOW);
  digitalWrite(5, HIGH);
  digitalWrite(6, HIGH);  // Motor 2 flipped
  digitalWrite(7, LOW);
  analogWrite(9, speed);
  analogWrite(10, speed);
  delay(5000);

  // STOP for 15 seconds
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  analogWrite(9, 0);
  analogWrite(10, 0);
  delay(15000);
}
