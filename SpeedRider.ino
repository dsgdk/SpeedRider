/*Copyright (C) 2024 dsgdk. All rights reserved.

This code is proprietary and confidential. Unauthorized copying, sharing, or use of this code in any form is strictly prohibited without prior written consent from the owner.
*/

#include <Servo.h>
Servo servo;

const bool DEBUG_RIDER = true;
const bool FAST_START = false;
const bool GREEN_START = true;
const bool ALGORITHM_TIME = true;

#define ZERO_POS 90
#define MAX_SERVO_ANGLE 30

uint8_t MIN_MOTOR_SPEED = 170;
uint8_t MAX_MOTOR_SPEED = 220;

/* ---- Pin Definitions ---- */
#define PWM_PIN 8
#define MOTORIN1_PIN 7
#define MOTORIN2_PIN 6

#define DIGITAL_SENS_PIN 40
#define ANALOG_SENS_PIN 54
#define TOTAL_SENSORS 11

#define SERVO_PIN 12

// Structure for sensor data
struct SensorData {
    int16_t analog_val[TOTAL_SENSORS];
    int16_t filtered_val[TOTAL_SENSORS];
    int16_t line_vector_val;
    int8_t lr_sum;
    int16_t white_color_val;
    int16_t black_color_val;
};

// Initialize sensor data
SensorData sensors = {0};

// Structure for storing motor parameters
struct MotorControl {
    uint8_t min_speed;
    uint8_t max_speed;
};

MotorControl motor_control = {MIN_MOTOR_SPEED, MAX_MOTOR_SPEED};

// Debugging function
void debugPrint(const String& message) {
    if (DEBUG_RIDER) {
        Serial.print(message);
    }
}

// Function to read sensor data
void readSensors(SensorData& sensors) {
    sensors.white_color_val = 1023;
    sensors.black_color_val = 0;

    // Reading analog values from sensors and updating min/max values
    for (int i = 0; i < TOTAL_SENSORS; i++) {
        sensors.analog_val[i] = analogRead(ANALOG_SENS_PIN + i) + sensor_correction[i];
        sensors.white_color_val = min(sensors.white_color_val, sensors.analog_val[i]);
        sensors.black_color_val = max(sensors.black_color_val, sensors.analog_val[i]);

        debugPrint(String(sensors.analog_val[i]) + ",");
    }

    // Print black and white sensor values and their ratio
    debugPrint("b " + String(sensors.black_color_val) + " w " + String(sensors.white_color_val) + " coef [" + String(float(sensors.black_color_val) / float(sensors.white_color_val)) + "]\n");

    // Filter sensor values based on threshold
    for (int i = 0; i < TOTAL_SENSORS; i++) {
        sensors.filtered_val[i] = (sensors.analog_val[i] >= (sensors.black_color_val - (sensors.black_color_val - sensors.white_color_val) / 4) && sensors.analog_val[i] >= sensors.white_color_val * 3) ? 1 : 0;
        debugPrint(String(sensors.filtered_val[i]));
    }

    // Analyze the error based on sensor values
    sensors.line_vector_val = errorAnalyzer(sensors);
    debugPrint("Error: " + String(sensors.line_vector_val) + "\n");
}

// Function to analyze sensor error
int32_t errorAnalyzer(const SensorData& sensors) {
    int32_t left_black = 0, right_black = 0, left_white = 0, right_white = 0;

    // Count white and black values on both sides
    for (int i = 0; i < TOTAL_SENSORS; i++) {
        if (!sensors.filtered_val[i] && !left_black) left_white++;
        else left_black++;

        int j = TOTAL_SENSORS - 1 - i;
        if (!sensors.filtered_val[j] && !right_black) right_white++;
        else right_black++;
    }

    sensors.lr_sum = left_white + right_white;
    int32_t line_vector_val_SS = left_white - right_white;

    // Print debugging information about sensor readings
    debugPrint("watch_track_val " + String(left_white + right_white) + "\n");
    debugPrint("l,r,lb,rb = l " + String(left_white) + " r " + String(right_white) + " lb " + String(left_black) + " rb " + String(right_black) + " lr sum " + String(sensors.lr_sum) + "\n");

    return line_vector_val_SS;
}

// Function to control the robot without PID
void voditelWithoutPID(SensorData& sensors, MotorControl& motor_control) {
    int16_t vect_trass = integraciya(sensors); // Integrate sensor data
    int16_t required_servo_pos;

    // Map the line vector value to servo angle based on conditions
    if (vect_trass < -1) {
        required_servo_pos = map(sensors.line_vector_val, 11, -11, ZERO_POS - (MAX_SERVO_ANGLE - 9), ZERO_POS + (MAX_SERVO_ANGLE - 9));
    } else if (vect_trass > 1) {
        required_servo_pos = map(sensors.line_vector_val, 9, -10, ZERO_POS - (MAX_SERVO_ANGLE + 6), ZERO_POS + (MAX_SERVO_ANGLE + 6));
    } else {
        required_servo_pos = map(sensors.line_vector_val, 11, -11, ZERO_POS - MAX_SERVO_ANGLE, ZERO_POS + MAX_SERVO_ANGLE);
    }

    // Control the servo based on track conditions
    if (watch_track_val != 22 && sensors.lr_sum >= 8) servo.write(required_servo_pos);

    // Calculate the required motor speed based on line vector value
    int16_t required_motor_speed = (sensors.line_vector_val >= 0) ?
        map(sensors.line_vector_val, 0, 11, motor_control.max_speed, motor_control.min_speed) :
        map(sensors.line_vector_val, 0, -11, motor_control.max_speed, motor_control.min_speed);

    // Increase motor speed if the error is small
    if (abs(sensors.line_vector_val) < 4) required_motor_speed = motor_control.max_speed;

    // Set motor direction and speed
    digitalWrite(MOTORIN1_PIN, HIGH);
    digitalWrite(MOTORIN2_PIN, LOW);
    analogWrite(PWM_PIN, required_motor_speed);

    // Print debugging information about servo and motor control
    debugPrint(" Servo: " + String(required_servo_pos) + " Motor: " + String(required_motor_speed) + "\n");
}

// Main function to control the robot
void generalDriver(SensorData& sensors, MotorControl& motor_control) {
    readSensors(sensors); // Read sensor data
    voditelWithoutPID(sensors, motor_control); // Control the robot
}

void setup() {
    pinMode(PWM_PIN, OUTPUT);
    pinMode(MOTORIN1_PIN, OUTPUT);
    pinMode(MOTORIN2_PIN, OUTPUT);

    Serial.begin(250000);
    Serial1.begin(115200);

    servo.attach(SERVO_PIN);
    delay(1);
    servo.write(ZERO_POS);

    // Wait for the green light to start
    if (GREEN_START) {
        while (traffic_light_value != 2) {
            roadTraffic();
        }
    }

    prew_millis = millis();

    // Fast start setup
    if (FAST_START) {
        digitalWrite(MOTORIN1_PIN, HIGH);
        digitalWrite(MOTORIN2_PIN, LOW);
        digitalWrite(PWM_PIN, HIGH);
        delay(1000);
    }
}

void loop() {
    while (true) {
        generalDriver(sensors, motor_control); // Continuously drive the robot

        // Adjust motor control based on elapsed time
        if (ALGORITHM_TIME) {
            lpTime = millis() - prew_millis;
            switch (lpTime) {
                case 7500 ... INT32_MAX:
                    motor_control.min_speed = 0;
                    motor_control.max_speed = 0;
                    break;
                case 5100 ... 7499:
                    motor_control.min_speed = 230;
                    motor_control.max_speed = 255;
                    break;
                case 4600 ... 5099:
                    motor_control.min_speed = 180;
                    motor_control.max_speed = 220;
                    break;
                case 3100 ... 4599:
                    motor_control.min_speed = 170;
                    motor_control.max_speed = 215;
                    break;
                case 1700 ... 3099:
                    motor_control.min_speed = 210;
                    motor_control.max_speed = 235;
                    break;
                case 1400 ... 1699:
                    motor_control.min_speed = 190;
                    motor_control.max_speed = 235;
                    break;
            }
        }

        roadTraffic(); // Monitor traffic light or other conditions
    }
}
