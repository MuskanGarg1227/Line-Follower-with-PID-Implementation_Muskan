#include <Arduino.h>
#include <Servo.h>  // Library for controlling servos (if not needed, can be ignored)

// Pin Definitions for motor driver and sensors
#define AIN1     PB7  // Motor A direction pin 1
#define AIN2     PB8  // Motor A direction pin 2
#define PWMA     PB9  // Motor A speed control (PWM pin)
#define BIN1     PB5  // Motor B direction pin 1
#define BIN2     PB4  // Motor B direction pin 2
#define PWMB     PB3  // Motor B speed control (PWM pin)
#define STBY     PB6  // Standby pin for the motor driver
#define CALIB    PA8  // Calibration indicator pin (used to show calibration status)

// PID Variables for controlling the motor based on sensor input
float Kp = 2.0, Ki = 5.0, Kd = 1.0;  // PID constants (adjustable values)
float error, last_error, integral, derivative;  // Variables for calculating PID
float outputA, outputB;  // Output for motors A and B
float setpoint = 0;  // Desired target value (position of the line)
unsigned long lastTime = 0;  // Last timestamp used for calculating time intervals in PID

// Line Sensor Variables for detecting the line on the ground
const int numSensors = 8;  // Number of sensors used
const int threshold = 700;  // Threshold value to determine if the sensor sees the line
const int numValues = 15;  // Number of values to average sensor readings
int minValues[numSensors], maxValues[numSensors], sensorValue[numSensors];  // Arrays to store min, max, and current sensor readings
int sensorReadings[numSensors][numValues];  // Array to store recent sensor values for averaging
int ourIndex = 0;  // Index for cycling through sensor readings
bool filled = false;  // Flag to indicate if enough readings have been taken
bool onLine = true;  // Flag to check if the robot is currently on the line

// Function Declarations
void motor();  // Function to control motor speed and direction
void calibrate();  // Function to calibrate the line sensors
void calc_error();  // Function to calculate error based on sensor input
void updateSensorInput();  // Function to update sensor readings
void staticPID(float dt);  // Function to calculate PID output
void ZieglerNicholsTuning(float Ku, float Tu);  // Function to perform PID tuning using Ziegler-Nichols method

// Array that holds pin numbers for each sensor
int arraypins[8] = {PA8, PA0, PA1, PA4, PA6, PA7, PB0, PA9};  // Pin assignments for the sensors

// Setup function runs once when the Arduino is powered up or reset
void setup() {
    // Motor Control Setup: Setting motor control pins as outputs
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);  // Enable the motor driver by pulling STBY high

    // Indicate calibration process has started
    digitalWrite(CALIB, HIGH);
    
    // Calibrate the line sensors to adjust for varying lighting conditions
    calibrate();
    
    // Indicate calibration process has ended
    digitalWrite(CALIB, LOW);

    // Initialize the lastTime variable for PID calculations
    lastTime = millis();

    // Example values for Ziegler-Nichols tuning, these values need to be determined experimentally
    float Ku = 1.0;  // Ultimate gain
    float Tu = 1.0;  // Oscillation period in seconds
    
    // Perform PID tuning using Ziegler-Nichols method
    ZieglerNicholsTuning(Ku, Tu);

    // Blink an LED on PC13 to indicate setup completion
    pinMode(PC13, OUTPUT);
    for (int i = 0; i < 5; i++) {
        digitalWrite(PC13, HIGH);  // Turn LED on
        delay(100);
        digitalWrite(PC13, LOW);  // Turn LED off
        delay(100);
    }
}

// Main loop runs continuously after setup
void loop() {
    // Calculate the time difference (dt) since the last loop
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;  // Convert time difference from milliseconds to seconds
    lastTime = currentTime;  // Update the lastTime to the current time

    // Read sensor values and calculate the error
    calc_error();

    // Perform PID calculations based on the error
    staticPID(dt);

    // Control motors based on the PID output
    motor();

    // Small delay to control loop speed (adjustable)
    delay(5);  // Delay can be modified based on application requirements
}

// Motor control function that adjusts motor speeds based on PID output
void motor() {
    int motorSpeedA = constrain(255 - outputA, -255, 255);  // Constrain motor A speed between -255 and 255
    int motorSpeedB = constrain(255 - outputB, -255, 255);  // Constrain motor B speed between -255 and 255

    // Set direction and speed for motor A
    if (motorSpeedA >= 0) {
        digitalWrite(AIN1, HIGH);  // Move forward
        digitalWrite(AIN2, LOW);
    } else {
        digitalWrite(AIN1, LOW);  // Move backward
        digitalWrite(AIN2, HIGH);
        motorSpeedA = -motorSpeedA;  // Convert speed to positive for PWM
    }
    analogWrite(PWMA, motorSpeedA);  // Set motor A speed

    // Set direction and speed for motor B
    if (motorSpeedB >= 0) {
        digitalWrite(BIN1, HIGH);  // Move forward
        digitalWrite(BIN2, LOW);
    } else {
        digitalWrite(BIN1, LOW);  // Move backward
        digitalWrite(BIN2, HIGH);
        motorSpeedB = -motorSpeedB;  // Convert speed to positive for PWM
    }
    analogWrite(PWMB, motorSpeedB);  // Set motor B speed
}

// Function to calibrate sensors by finding the minimum and maximum values
void calibrate() {
    // Initialize min and max values for the sensors
    for (int i = 0; i < numSensors; i++) {
        if (i == 0 || i == 7) {
            minValues[i] = 1000 * digitalRead(arraypins[i]);  // For edge sensors, use digital readings
            maxValues[i] = 1000 * digitalRead(arraypins[i]);
        } else {
            minValues[i] = analogRead(arraypins[i]);  // For middle sensors, use analog readings
            maxValues[i] = analogRead(arraypins[i]);
        }
    }

    int csp = 40;  // Motor speed during calibration

    // Perform calibration over 10,000 iterations by moving motors and adjusting sensor readings
    for (int i = 0; i < 10000; i++) {
        analogWrite(PWMA, csp);  // Move motor A forward
        analogWrite(PWMB, -csp);  // Move motor B backward

        // Update sensor min/max values based on current readings
        for (int i = 0; i < numSensors; i++) {
            int value;
            if (i == 0 || i == 7) {
                value = 1000 * digitalRead(arraypins[i]);  // Digital read for edge sensors
            } else {
                value = analogRead(arraypins[i]);  // Analog read for middle sensors
            }
            if (value < minValues[i]) minValues[i] = value;  // Update min value
            if (value > maxValues[i]) maxValues[i] = value;  // Update max value
        }
    }

    // Stop motors after calibration
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
}

// Function to calculate the error value based on sensor readings
void calc_error() {
    updateSensorInput();  // Update sensor input before calculating error
    error = 0;  // Initialize error to 0
    for (int i = 0; i < numSensors; i++) {
        error += sensorValue[i] * (i - 4) * -1;  // Calculate weighted sum of sensor positions
    }
}

// Function to update sensor input and average the readings
void updateSensorInput() {
    onLine = false;  // Assume initially that the robot is not on the line

    // `sensorReadings` stores the last `numValues` readings of the sensors
    // `sensorValue` stores the average of the last `numValues` readings of the sensors
    for (int i = 0; i < numSensors; i++) {
        if (filled) {  // If enough values are filled, calculate the average
            sensorValue[i] = sensorValue[i] * numValues - sensorReadings[i][ourIndex];  // Subtract the oldest reading

            // For edge sensors (i == 0 or i == 7), use digitalRead, otherwise use analogRead
            if (i == 0 || i == 7) {
                // Constrain sensor reading between 0 and 1000 and map it based on min/max calibration values
                sensorReadings[i][ourIndex] = constrain(map(digitalRead(arraypins[i]), minValues[i], maxValues[i], 0, 1000), 0, 1000);
            } else {
                // Constrain analog reading for non-edge sensors
                sensorReadings[i][ourIndex] = constrain(map(analogRead(arraypins[i]), minValues[i], maxValues[i], 0, 1000), 0, 1000);
            }

            // Add new reading and calculate average over `numValues`
            sensorValue[i] = (sensorValue[i] + sensorReadings[i][ourIndex]) / numValues;
        } else {  // If the array isn't filled yet, perform averaging based on how many readings there are
            if (i == 0 || i == 7) {
                // For edge sensors, use digital reading
                sensorReadings[i][ourIndex] = constrain(map(digitalRead(arraypins[i]), minValues[i], maxValues[i], 0, 1000), 0, 1000);
            } else {
                // For non-edge sensors, use analog reading
                sensorReadings[i][ourIndex] = constrain(map(analogRead(arraypins[i]), minValues[i], maxValues[i], 0, 1000), 0, 1000);
            }

            // Calculate average value incrementally based on the number of readings so far
            sensorValue[i] = (sensorValue[i] * ourIndex + sensorReadings[i][ourIndex]) / (ourIndex + 1);
        }

        // Check if the sensor reading is above the threshold, indicating the line has been detected
        if (sensorValue[i] > threshold) {
            onLine = true;  // Set onLine to true if any sensor detects the line
        }
    }

    // Set `filled` to true if `ourIndex` has reached the end of the sensor readings array
    filled = (filled || (ourIndex == (numValues - 1)));

    // Increment `ourIndex` and wrap it around if necessary to reuse old values
    ourIndex = (ourIndex + 1) % numValues;
}

void staticPID(float dt) {
    // Calculate the error between the setpoint (desired position) and the actual position
    float current_error = setpoint - error;

    // Update integral term by adding the current error scaled by time difference `dt`
    integral += current_error * dt;

    // Calculate derivative term by determining how fast the error is changing
    derivative = (current_error - last_error) / dt;

    // PID formula: Proportional + Integral + Derivative control
    outputA = (Kp * current_error) + (Ki * integral) + (Kd * derivative);

    // For simplicity, we assume motor B has the same output as motor A
    outputB = outputA;

    // Store the current error for the next iteration to calculate derivative in future
    last_error = current_error;
}

void ZieglerNicholsTuning(float Ku, float Tu) {
    // Ziegler–Nichols tuning formulas for a PID controller based on ultimate gain `Ku` and oscillation period `Tu`
    
    // Proportional gain Kp is 60% of the ultimate gain
    Kp = 0.6 * Ku;

    // Integral gain Ki is derived from Kp and Tu (based on Ziegler-Nichols)
    Ki = 2 * Kp / Tu;

    // Derivative gain Kd is calculated using Kp and Tu
    Kd = Kp * Tu / 8;
}


