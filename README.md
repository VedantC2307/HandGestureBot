# ✋ Hand Gesture Controlled Robot Hand 🤖

This was our robot hand gesture imitation project for the Expressive Robotics class. We created a seamless bridge between human movement and robotic response.

![Demo](demo.gif)

## Overview
This project uses MediaPipe for hand tracking through a webcam and translates finger positions into motor commands for a robotic hand. Each finger's movement is mapped to a corresponding stepper motor, imitating the movement between the human hand and the robotic hand. To ensure smooth and stable motion, we implemented Exponential Moving Average (EMA) with deadzone consideration, which helps filter out small unintentional movements while maintaining responsive control.

## 🛠️ Requirements
### Hardware
- Arduino board
- 5 Stepper motors
- Webcam


## 🚀 Setup
1. Install Python packages:
   ```bash
   pip install -r requirements.txt
   ```

2. Upload the Arduino code (`sketch_jan9b.ino`) to your Arduino board

3. Connect the stepper motors to these Arduino pins:
   - STEP pins: 4, 6, 5, 8, 7
   - DIR pins: 9, 11, 10, 13, 12

4. Run the Python script:
   ```bash
   python handGesture.py
   ```

## 👋 Usage
1. The program starts with a 10-second calibration ⏱️
2. Move your hand fully open and closed during calibration
3. After calibration, the system will track your hand in real-time
4. The robot hand will copy your finger movements
