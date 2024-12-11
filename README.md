# Drone Flight Controller

Welcome to the **Drone Flight Controller** repository! This project contains the firmware for the flight controller of a quadcopter drone. The firmware is designed to read data from an MPU6050 sensor, process it using PID control and Kalman filters, and send output signals to control the motors. The system includes safety mechanisms such as an arming requirement to prevent accidental operation.

---

## Features
- Real-time sensor data processing with the MPU6050 IMU.
- PID control for stable flight (Roll, Pitch, and Yaw).
- Kalman filtering for accurate state estimation.
- Safety mechanism requiring an explicit arming signal via serial communication.
- PWM motor control with dynamic speed adjustment.

---

## Getting Started

### Hardware Requirements
- **RP2040 Microcontroller** (or Arduino-compatible board)
- **MPU6050 IMU** (Accelerometer + Gyroscope)
- Quadcopter frame, motors, ESCs, and power supply.
- USB cable for programming and serial communication.

### Software Requirements
- Arduino IDE (with necessary libraries):
  - `Wire.h`
  - `MPU6050.h`

---

## Installation
1. Clone this repository to your local machine:
   ```bash
   git clone https://github.com/yourusername/drone-flight-controller.git
   ```
2. Open the firmware code (`flight_controller.ino`) in the Arduino IDE.
3. Connect your microcontroller to your computer via USB.
4. Configure the appropriate board and port in the Arduino IDE.
5. Upload the firmware to your microcontroller.

---

## Firmware Overview

### Arming Mechanism
The drone will not operate until an explicit arming signal is sent via serial communication. This ensures safe operation and prevents accidental motor activation.

### Core Functionality
1. **Sensor Data Processing**: Reads accelerometer and gyroscope data from the MPU6050.
2. **PID Control**: Computes motor outputs to maintain stable flight.
3. **Kalman Filtering**: Provides accurate angle estimation by combining accelerometer and gyroscope data.
4. **Motor Control**: Dynamically adjusts motor speeds based on PID outputs.

### Configuration
- PID parameters for Roll, Pitch, and Yaw can be modified in the code for tuning.
- Kalman filter parameters are pre-configured for typical drone setups but can also be adjusted as needed.

---

## Usage
1. Power on the drone and connect it to your computer.
2. Use a serial communication tool (e.g., Arduino Serial Monitor) to:
   - Send the arming signal.
   - Monitor sensor data and control outputs for debugging.
3. Once armed, the drone is ready for manual or autonomous operation.

---

## Safety Precautions
- Ensure the drone is placed on a flat surface before powering on.
- Do not arm the drone near people or obstacles.
- Test motor operations without propellers initially to validate the firmware.

---

## Contributing
Contributions are welcome! Feel free to fork the repository, make improvements, and submit a pull request.

---

## License
This project is licensed under the MIT License. See the `LICENSE` file for details.

---

## Contact
For questions or support, please open an issue in this repository or contact [banumathhettiarachchi@gmail.com].

