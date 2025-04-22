# Intelligent-Accident-Avoidance-System

**Introduction**
This project presents an Intelligent Accident-Avoidance System designed to improve road safety by monitoring driving behavior in real time. The system uses accelerometer, gyroscope, and GPS sensors, along with machine learning, to detect and alert against unsafe driving patterns such as:

Lane Weaving

Lane Swerving

Hard Braking

Hard Cornering

Quick U-turns

Overspeeding

When such behavior is detected, the system sends alerts via a cloud-based notification system, enabling timely intervention.

**Project Overview**
Microcontroller: ESP32

Sensors: ICM20948 (Accelerometer & Gyroscope), GPS Module

Memory: 23LC512 SRAM for buffered data storage

Communication Protocols: I2C, SPI, UART, Wi-Fi

Cloud Services: AWS IoT Core, HERE API for speed limit detection

Machine Learning Model: Custom CNN model trained on IMU sensor data

**Machine Learning Overview**
Input Data: Acceleration (X, Y), Gyroscope (Z) from IMU sensor

Dataset: Collected via smartphone in real driving conditions

Driving Behaviors Detected:

Lane Weaving

Lane Swerving

Hard Braking

Hard Cornering

Quick U-turn

Model Type: Convolutional Neural Network (CNN)

Performance Metrics: Accuracy, Precision, Recall, F1 Score, ROC-AUC

**System Features**
Real-time monitoring and detection of unsafe driving behavior

Cloud-based alerts via AWS and buzzer output for driver feedback

Speed limit detection using GPS + HERE API

Buffering and reliability through dual memory management (ESP32 + SRAM)

FreeRTOS task management for sensor reading and Wi-Fi communication

**Results and Evaluation**
The CNN model achieved high accuracy in classifying driving behaviors

Real-time alerts were reliably generated for overspeeding and rash events

Integration with Power BI provided effective visualization of system performance

Field testing showed successful detection of harsh braking and swerving

Minor false positives observed in stop-and-go traffic scenarios

**Future Enhancements**
Incorporate additional sensors such as radar or camera for contextual awareness

Improve classification accuracy in congested traffic conditions

Enhance predictive analytics using historical behavior and environmental data

Explore advanced models (e.g., RNN, Reinforcement Learning) for dynamic behavior prediction

**Authors**
Akilesh C (21BLC1325)

Sushruth S (21BLC1177)

Nikesh Kumar S (21BLC1308)
