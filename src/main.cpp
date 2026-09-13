#include <ESP32Servo.h>

Servo myServo;

const int SERVO_PIN = 18;

void setup() {
  Serial.begin(115200);

  myServo.attach(SERVO_PIN);

  myServo.write(90);

  Serial.println("Servo test ready.");
  Serial.println("Enter an angle from 0 to 180:");
}

void loop() {
  if (Serial.available()) {
    int angle = Serial.parseInt();

    if (angle >= 0 && angle <= 180) {
      myServo.write(angle);

      Serial.print("Moving servo to: ");
      Serial.print(angle);
      Serial.println(" degrees");
    } else {
      Serial.println("Invalid angle. Enter 0 to 180.");
    }

    // Clear remaining newline characters
    while (Serial.available()) {
      Serial.read();
    }
  }
}