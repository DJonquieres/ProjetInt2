#include <ESP32Servo.h>
#include <Bluepad32.h>

// === Servo Setup ===
Servo servo;
const int ServoPin = 18;

// === Motor Driver Pins for Wheels ===
const int LEFT_MOTOR_IN1 = 25;
const int LEFT_MOTOR_IN2 = 26;
const int RIGHT_MOTOR_IN1 = 27;
const int RIGHT_MOTOR_IN2 = 14;
const int LEFT_PWM = 4;    // ENA
const int RIGHT_PWM = 2;    //ENB

// === Forklift Motor Driver Pins ===
const int FORKLIFT_IN1 = 19;
const int FORKLIFT_IN2 = 21;
const int FORKLIFT_PWM = 22;

// === PWM Channels ===
const int LEFT_PWM_CH = 10;
const int RIGHT_PWM_CH = 1;
const int FORKLIFT_PWM_CH = 2;

// === Status LED ===
const int STATUS_LED_PIN = 2;

// === Gamepad ===
GamepadPtr myGamepad = nullptr;

void onGamepadConnected(GamepadPtr gp) {
  Serial.println("Gamepad connected!");
  myGamepad = gp;
  digitalWrite(STATUS_LED_PIN, HIGH); // LED ON
}

void onGamepadDisconnected(GamepadPtr gp) {
  Serial.println("Gamepad disconnected!");
  if (myGamepad == gp) {
    myGamepad = nullptr;
  }
  digitalWrite(STATUS_LED_PIN, LOW); // LED OFF
}

void setupPWMChannel(int channel, int pin) {
  ledcSetup(channel, 1000, 8);     // 1kHz, 8-bit
  ledcAttachPin(pin, channel);
}

void setup() {
  Serial.begin(115200);

  // Set up direction pins
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  pinMode(FORKLIFT_IN1, OUTPUT);
  pinMode(FORKLIFT_IN2, OUTPUT);

  // Setup PWM channels
  setupPWMChannel(LEFT_PWM_CH, LEFT_PWM);
  setupPWMChannel(RIGHT_PWM_CH, RIGHT_PWM);
  setupPWMChannel(FORKLIFT_PWM_CH, FORKLIFT_PWM);

  // Attach servo
  servo.attach(ServoPin);

  // Setup status LED
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW); // Start off

  // Setup Bluepad32
  BP32.setup(onGamepadConnected, onGamepadDisconnected);
}

void setMotor(int in1, int in2, int pwmChannel, int speed) {
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(pwmChannel, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(pwmChannel, -speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(pwmChannel, 0);
  }
}

void loop() {
  BP32.update();

  if (myGamepad && myGamepad->isConnected()) {
    // === Speed control from triggers ===
    int l2 = myGamepad->brake(); // 0–1012
    int r2 = myGamepad->throttle(); // 0–1012
    int speedFactor = max(l2, r2); // Scale speed

    // === Joystick with deadzone ===
    int rawY = myGamepad->axisY(); // Forward/backward
    int rawX = myGamepad->axisX(); // Turning
    const int DEADZONE = 20;

    int y = abs(rawY) > DEADZONE ? rawY : 0;
    int x = abs(rawX) > DEADZONE ? rawX / 2 : 0;  // Reduce turning speed by 50%

    // Scale motor speed with speed factor
    int leftSpeed = constrain(map(-y + x, -512, 512, -speedFactor, speedFactor), -1024, 1024);
    int rightSpeed = constrain(map(-y - x, -512, 512, -speedFactor, speedFactor), -1024, 1024);

    Serial.print("Left Speed: ");
    Serial.print(leftSpeed);
    Serial.print(" | Right Speed: ");
    Serial.println(rightSpeed);

    setMotor(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, LEFT_PWM_CH, leftSpeed);
    setMotor(RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, RIGHT_PWM_CH, rightSpeed);

    // === Forklift Control ===
    if (myGamepad->dpad() & DPAD_UP) {
      setMotor(FORKLIFT_IN1, FORKLIFT_IN2, FORKLIFT_PWM_CH, 200);
    } else if (myGamepad->dpad() & DPAD_DOWN) {
      setMotor(FORKLIFT_IN1, FORKLIFT_IN2, FORKLIFT_PWM_CH, -200);
    } else {
      setMotor(FORKLIFT_IN1, FORKLIFT_IN2, FORKLIFT_PWM_CH, 0);
    }

    // === Servo Control ===
    if (myGamepad->b()) {
      servo.writeMicroseconds(1100);  // One turn forward
      delay(1600);
      servo.writeMicroseconds(1500);  // Stop
    }

    if (myGamepad->y()) {
      servo.writeMicroseconds(1600);  // Opposite direction
    } else {
      servo.writeMicroseconds(1500);  // Stop
    }
  }

  delay(10);
}

