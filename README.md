# Line-following-robot-using-ESP32

# ESP32-Based Line Following Robot with Real-Time PID Control

##  Project Overview
This project features the design, implementation, and evaluation of a robust closed-loop control system for an autonomous line-following robot. Built on the ESP32 platform using FreeRTOS, the robot utilizes a five-sensor infrared (IR) array and PID-based control logic to accurately navigate straight paths, curves, and sharp turns in real-time.

## Hardware Components
* **Microcontroller:** ESP32 
* **Sensors:** 5x Infrared (IR) Sensor Array
* **Motor Driver:** L298N Dual H-Bridge
* **Actuators:** DC Motors (4-wheel chassis)
* **Power Supply:** 12V Battery

##  Software & Control Logic
* **Language:** C++ (Arduino IDE)
* **OS:** FreeRTOS (Multitasking for concurrent sensor reading and motor control)
* **Control System:** Proportional-Integral-Derivative (PID) Controller

### Mathematical Model
The robot calculates the error signal based on the deviation from the line center:
`e(t) = Desired Position - Measured Position`

The PID control law is applied to generate the necessary motor correction:
`u(t) = Kp*e(t) + Ki∫e(t)dt + Kd(de(t)/dt)`

*Experimental Tuning Parameters Used:*
* Proportional Gain (Kp): 1.2
* Integral Gain (Ki): 0.01
* Derivative Gain (Kd): 0.8

##  Features
* **Weighted Average Position Detection:** Smooth line tracking using a 5-sensor array.
* **Locked-Turn Logic:** Prevents sensor noise during sharp maneuvers.
* **Real-Time Processing:** FreeRTOS tasks separate sensor polling and PID calculations for optimized timing.
