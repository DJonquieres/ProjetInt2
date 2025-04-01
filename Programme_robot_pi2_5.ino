#include <ESP32Servo.h>
#include <Bluepad32.h>
#include "driver/ledc.h"

// --- Servo Setup ---
Servo servo180_1;
constexpr int SERVO_PIN = 18;

// --- Motor Pins ---
constexpr int MOTOR_PINS[][3] = {
    {27, 26, 4},  // Motor 1: IN1, IN2, PWM
    {14, 12, 5},  // Motor 2
    {33, 32, 25}  // Motor 3
};

// --- Control Variables ---
int currentSpeed[3] = {0, 0, 0};

// --- PWM Constants ---
constexpr int PWM_FREQ = 1000;
constexpr int PWM_RESOLUTION = LEDC_TIMER_8_BIT;
constexpr int PWM_MAX_DUTY = 255;
constexpr int DEADZONE = 10;

// --- Bluepad32 Controller ---
ControllerPtr myController;

void onConnectedController(ControllerPtr ctl) {
    Serial.println("Controller connected!");
    myController = ctl;
}

void onDisconnectedController(ControllerPtr ctl) {
    Serial.println("Controller disconnected");
    myController = nullptr;
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting Bluepad32...");
    
    servo180_1.attach(SERVO_PIN);
    
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();

    for (auto& motor : MOTOR_PINS) {
        pinMode(motor[0], OUTPUT);
        pinMode(motor[1], OUTPUT);
    }

    ledc_timer_config_t ledc_timer = {
      .speed_mode = LEDC_HIGH_SPEED_MODE,
      .duty_resolution = (ledc_timer_bit_t)PWM_RESOLUTION,  // Cast here
      .timer_num = LEDC_TIMER_0,
      .freq_hz = PWM_FREQ,
      .clk_cfg = LEDC_AUTO_CLK
    };

    ledc_timer_config(&ledc_timer);

    for (int i = 0; i < 3; i++) {
        ledc_channel_config_t channel = {
            .gpio_num = MOTOR_PINS[i][2],
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .channel = (ledc_channel_t)i,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0
        };
        ledc_channel_config(&channel);
    }
    Serial.println("Ready to connect controller!");
}

void setMotor(int index, int speed) {
    int in1 = MOTOR_PINS[index][0];
    int in2 = MOTOR_PINS[index][1];
    ledc_channel_t channel = (ledc_channel_t)index;
    
    digitalWrite(in1, speed > 0);
    digitalWrite(in2, speed < 0);
    
    int pwm_value = constrain(abs(speed), 0, PWM_MAX_DUTY);
    if (pwm_value != currentSpeed[index]) {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, pwm_value);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel);
        currentSpeed[index] = pwm_value;
    }
}

void rampMotorsToTarget(int targets[3], int stepSize) {
    for (int i = 0; i < 3; i++) {
        int delta = constrain(targets[i] - currentSpeed[i], -stepSize, stepSize);
        setMotor(i, currentSpeed[i] + delta);
    }
}

void controlServo(ControllerPtr ctl) {
    int servoSpeed = 1500;  // Neutral (stop)

    if (ctl->buttons() & BUTTON_B) {
        servoSpeed = 1100;   // Rotate one direction
    } else if (ctl->buttons() & BUTTON_Y) {
        servoSpeed = 1600;  // Rotate the opposite direction
    }

    servo180_1.writeMicroseconds(servoSpeed);
}

void loop() {
    BP32.update();

    int targetSpeed[3] = {0, 0, 0};
    if (myController && myController->isConnected()) {
        int ly = -myController->axisY();
        int lx = myController->axisX();

        if (abs(ly) < DEADZONE) ly = 0;
        if (abs(lx) < DEADZONE) lx = 0;

        targetSpeed[0] = map(ly - lx, -512, 511, -PWM_MAX_DUTY, PWM_MAX_DUTY);
        targetSpeed[1] = map(ly + lx, -512, 511, -PWM_MAX_DUTY, PWM_MAX_DUTY);
        
        if (myController->dpad() & DPAD_UP) {
            targetSpeed[2] = -PWM_MAX_DUTY;
        } else if (myController->dpad() & DPAD_DOWN) {
            targetSpeed[2] = PWM_MAX_DUTY;
        }

        rampMotorsToTarget(targetSpeed, 5);
        controlServo(myController);
    } else {
        rampMotorsToTarget(targetSpeed, 5);
    }

    delay(20);
}

