#include <ESP32Servo.h>
#include <Bluepad32.h>

Servo servo;
int ServoPin = 18;
GamepadPtr myGamepad = nullptr;  // Pointer to store the connected gamepad

void onGamepadConnected(GamepadPtr gp) {
  Serial.println("Gamepad connected!");
  myGamepad = gp;
}

void onGamepadDisconnected(GamepadPtr gp) {
  Serial.println("Gamepad disconnected!");
  if (myGamepad == gp) {
    myGamepad = nullptr;
  }
}

void setup() {
  Serial.begin(115200);
  
  // Attach the servo to the specified pin
  servo.attach(ServoPin);
  
  // Initialize Bluepad32 with connection callbacks
  BP32.setup(onGamepadConnected, onGamepadDisconnected);
}

void loop() {
  // Fetch the latest gamepad state
  BP32.update();

  // Check if a gamepad is connected
  if (myGamepad && myGamepad->isConnected()) {
    if (myGamepad->b()) {
      // If Circle is pressed, move the servo forward
      servo.writeMicroseconds(1100);
      delay(1600);
      servo.writeMicroseconds(1500);
    }
    
    if (myGamepad-> y()) {
      // If Circle is pressed, move the servo forward
      servo.writeMicroseconds(1600);
      } else {
      servo.writeMicroseconds(1500);
    }
  }
  
  delay(10); // Small delay to improve stability
}


