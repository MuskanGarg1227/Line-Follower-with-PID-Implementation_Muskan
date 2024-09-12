# Line-Follower-with-PID-Implementation_Muskan

Overview:
This project implements a line-following robot using PID control in Arduino. The robot follows a line on the ground by using multiple sensors and adjusting its motor speeds based on a PID controller to minimize error and ensure smooth and accurate tracking.

Features:
1. Line Following: Uses an array of sensors to detect and follow a line on the ground.
2. PID Control: Employs a PID controller to adjust motor speeds for smooth and precise line tracking.
3. Sensor Calibration: Includes a calibration routine for sensor values to adapt to different lighting conditions and surface types.
4. Modular Code: Organized into separate functions for motor control, sensor calibration, PID calculation, and error handling.

Components:
1. Arduino Board 
2. Motor Driver 
3. DC Motors
4. Line Sensors 
5. Various resistors and capacitors (depending on sensor and motor driver specifications)

Circuit Diagram:

Code Explanation:
1. setup(): Initializes pins, sets up motor control, calibrates sensors, and performs initial PID tuning.
2. loop(): Continuously reads sensor data, calculates the error, updates PID values, and adjusts motor speeds.
3. motor(): Controls motor direction and speed based on PID output.
4. calibrate(): Calibrates sensor readings to handle different environmental conditions.
5. calc_error(): Computes the error value based on sensor input.
6. updateSensorInput(): Updates sensor readings and calculates average values.
7. staticPID(): Calculates PID control values to adjust motor speeds.
8. ZieglerNicholsTuning(): Tunes PID parameters using the Ziegler-Nichols method.

Ziegler-Nichols Method
The Ziegler-Nichols method is a technique for tuning PID controllers to improve system performance. 
It involves:
1. Finding Ultimate Gain (Ku): Increase the proportional gain (Kp) until the system starts oscillating continuously.
2. Measuring Oscillation Period (Tu): Determine the period of these oscillations.
3. Calculating PID Parameters:
   1. Proportional Gain (Kp):  𝐾
𝑝
=
0.6
×
𝐾
𝑢
   2. Integral Gain (Ki):  𝐾
𝑖
=
2
×
𝐾
𝑝
/𝑇
𝑢

   3.  Derivative Gain (Kd):  𝐾
d
=
𝐾
𝑝
×
𝑇
𝑢
/8





