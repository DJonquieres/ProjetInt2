#include <ESP32Servo.h>
#include <Bluepad32.h>
#include "driver/ledc.h"
#include <uni.h>

// --- Servo Setup ---
Servo servo;
#define SERVO_PIN 18

// --- Motor Pins ---
#define MOTOR1_IN1  27  // Moteur Gauche?
#define MOTOR1_IN2  26
#define MOTOR1_PWM  4

#define MOTOR2_IN1  14  // Moteur Droit?
#define MOTOR2_IN2  12
#define MOTOR2_PWM  5

#define MOTOR3_IN1  33 // Vis sans fin
#define MOTOR3_IN2  32
#define MOTOR3_PWM  25

// --- Control Variables ---
int currentSpeedLeft = 0;
int currentSpeedRight = 0;
int currentSpeedMotor3 = 0;

// --- PWM Constants ---
#define PWM_FREQ        1000
#define PWM_RESOLUTION  LEDC_TIMER_8_BIT
#define PWM_MAX_DUTY    255

// --- Deadzone ---
#define DEADZONE 10

// --- Bluepad32 Controller ---
ControllerPtr myController;

// --- Allowlist Setup ---
static const char *controller_addr_string = "41:42:7D:FE:5C:BB";

void onConnectedController(ControllerPtr ctl) {
    Serial.println("Controller connected!");
    ctl->setColorLED(0, 255, 0);
    myController = ctl;
}

void onDisconnectedController(ControllerPtr ctl) {
    Serial.println("Controller disconnected");
    rampMotorsToTarget(0, 0, 0);
    myController = nullptr;
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting Bluepad32...");

    // servo.attach(SERVO_PIN);
    servo.attach(SERVO_PIN, 1000, 2000); // Explicitly define PWM range


    // Setup Bluepad32
    BP32.forgetBluetoothKeys();
    BP32.setup(onConnectedController, onDisconnectedController);
    
    // Allowlist Configuration
    bd_addr_t controller_addr;
    sscanf_bd_addr(controller_addr_string, controller_addr);
    uni_bt_allowlist_add_addr(controller_addr);
    uni_bt_allowlist_set_enabled(true);

    pinMode(MOTOR1_IN1, OUTPUT);
    pinMode(MOTOR1_IN2, OUTPUT);
    pinMode(MOTOR2_IN1, OUTPUT);
    pinMode(MOTOR2_IN2, OUTPUT);
    pinMode(MOTOR3_IN1, OUTPUT);
    pinMode(MOTOR3_IN2, OUTPUT);

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .duty_resolution  = PWM_RESOLUTION,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = PWM_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t motor_channels[] = {
        {MOTOR1_PWM, LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, LEDC_INTR_DISABLE, LEDC_TIMER_0, 0, 0},
        {MOTOR2_PWM, LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, LEDC_INTR_DISABLE, LEDC_TIMER_0, 0, 0},
        {MOTOR3_PWM, LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, LEDC_INTR_DISABLE, LEDC_TIMER_0, 0, 0}
    };
    
    for (auto& ch : motor_channels) {
        ledc_channel_config(&ch);
    }

    Serial.println("Ready to connect controller!");
}

void setMotor(int in1, int in2, ledc_channel_t channel, int speed) {
    digitalWrite(in1, speed > 0);
    digitalWrite(in2, speed < 0);

    int pwm_value = constrain(abs(speed), 0, PWM_MAX_DUTY);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, pwm_value);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel);
}

void rampMotorsToTarget(int targetSpeedLeft, int targetSpeedRight, int targetSpeedMotor3) {
    setMotor(MOTOR1_IN1, MOTOR1_IN2, LEDC_CHANNEL_0, targetSpeedRight);
    setMotor(MOTOR2_IN1, MOTOR2_IN2, LEDC_CHANNEL_1, targetSpeedLeft);
    setMotor(MOTOR3_IN1, MOTOR3_IN2, LEDC_CHANNEL_2, targetSpeedMotor3);
}

void controlMotors() {
  int ly = -(myController->axisY());
  int lx = myController->axisX();

  if (myController->dpad() & DPAD_RIGHT)

  if (abs(ly) < DEADZONE) ly = 0;
  if (abs(lx) < DEADZONE) lx = 0;

  // int maxSpeed = PWM_MAX_DUTY ;
  // int targetLeftSpeed  = map(ly + lx, -512, 511, -maxSpeed/3, maxSpeed/3);
  // int targetRightSpeed = map(ly - lx, -512, 511, -maxSpeed/3, maxSpeed/3);

  int maxSpeed = PWM_MAX_DUTY ;
  int throttle = myController->throttle();
  int brake = myController->brake();
  int actualSpeed = maxSpeed * (throttle - brake);

  int targetLeftSpeed = 0;
  int targetRightSpeed = 0;
  if (actualSpeed < 0) {
    targetLeftSpeed = map(ly + lx, -512, 0, -actualSpeed, actualSpeed);
    targetRightSpeed = map(ly - lx, -512, 0, -actualSpeed, actualSpeed);

  } else {
    targetLeftSpeed = map(ly + lx, 0, 512, -actualSpeed, actualSpeed);
    targetRightSpeed = map(ly - lx, 0, 512, -actualSpeed, actualSpeed);
  }


  int targetMotor3Speed = 0;
  if (myController->dpad() & DPAD_UP) {
    targetMotor3Speed = -maxSpeed;
  } else if (myController->dpad() & DPAD_DOWN) {
    targetMotor3Speed = maxSpeed;
  }

  rampMotorsToTarget(targetLeftSpeed, targetRightSpeed, targetMotor3Speed);
}

void controlServo() {
    if (myController->b()) {
        servo.writeMicroseconds(1100);
        delay(1600);
        servo.writeMicroseconds(1500);
    }
    
    if (myController->y()) {

        servo.writeMicroseconds(1600);
    } else {
        servo.writeMicroseconds(1500);
    }
}

void loop() {
    BP32.update();
    if (myController && myController->isConnected() && myController->hasData()) {
      controlMotors();
      controlServo();
    }

    if (!myController || !myController->isConnected()) {  //added to stop movement after disconnect
      rampMotorsToTarget(0, 0, 0); //stop motors
      servo.writeMicroseconds(1500); //stop servo

    }
    delay(10);
}

