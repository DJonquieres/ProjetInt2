#include <ESP32Servo.h>
#include <Bluepad32.h>

// === Servo Setup ===
Servo servo;
const int ServoPin = 13;

// === Motor Driver Pins for Wheels ===
const int LEFT_MOTOR_IN1 = 18;
const int LEFT_MOTOR_IN2 = 5;
const int RIGHT_MOTOR_IN1 = 16;
const int RIGHT_MOTOR_IN2 = 17;
const int LEFT_PWM = 19;    // ENA
const int RIGHT_PWM = 26;   // ENB

// === Forklift Motor Driver Pins ===
const int FORKLIFT_IN1 = 22;
const int FORKLIFT_IN2 = 23;
const int FORKLIFT_PWM = 12;

// === PWM Channels ===
const int LEFT_PWM_CH = 10;
const int RIGHT_PWM_CH = 9;
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

  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  pinMode(FORKLIFT_IN1, OUTPUT);
  pinMode(FORKLIFT_IN2, OUTPUT);

  setupPWMChannel(LEFT_PWM_CH, LEFT_PWM);
  setupPWMChannel(RIGHT_PWM_CH, RIGHT_PWM);
  setupPWMChannel(FORKLIFT_PWM_CH, FORKLIFT_PWM);

  servo.attach(ServoPin);

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

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
    uint8_t dpad = myGamepad->dpad();
    const int motorSpeed = 150;

    int leftSpeed = 0;
    int rightSpeed = 0;

    if (dpad & DPAD_UP) {
      leftSpeed = motorSpeed;
      rightSpeed = motorSpeed;
    } else if (dpad & DPAD_DOWN) {
      leftSpeed = -motorSpeed;
      rightSpeed = -motorSpeed;
    } else if (dpad & DPAD_LEFT) {
      leftSpeed = -motorSpeed;
      rightSpeed = motorSpeed;
    } else if (dpad & DPAD_RIGHT) {
      leftSpeed = motorSpeed;
      rightSpeed = -motorSpeed;
    }

    setMotor(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, LEFT_PWM_CH, leftSpeed);
    setMotor(RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, RIGHT_PWM_CH, rightSpeed);

    // === Forklift Control via Right Joystick ===
    int rawLift = myGamepad->axisRY(); // RY: -512 to 512
    const int DEADZONE = 50;
    int liftSpeed = 0;

    if (abs(rawLift) > DEADZONE) {
      liftSpeed = map(-rawLift, -512, 512, -255, 255); // Invert axis
    }

    setMotor(FORKLIFT_IN1, FORKLIFT_IN2, FORKLIFT_PWM_CH, liftSpeed);

    // === Servo Control ===
    if (myGamepad->x()) {
      servo.writeMicroseconds(1100);
      delay(1600);
      servo.writeMicroseconds(1500);
    }

    if (myGamepad->a()) {
      servo.writeMicroseconds(1100);
      delay(1400);
      setMotor(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, LEFT_PWM_CH, 1024);
      setMotor(RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, RIGHT_PWM_CH, 1024);
      delay(200);
      setMotor(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, LEFT_PWM_CH, 0);
      setMotor(RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, RIGHT_PWM_CH, 0);      
      servo.writeMicroseconds(1500);
    }

    if (myGamepad->b()) {
      servo.writeMicroseconds(1000);
    } else if (myGamepad->y()) {
      servo.writeMicroseconds(1600);
    } else {
      servo.writeMicroseconds(1500);
    }
  }

  delay(10);
}

