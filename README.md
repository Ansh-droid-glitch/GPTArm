# 🎙️🤖 Voice-Controlled Vision-Guided Robotic Arm

<p align="center">
  <strong>Part of the <a href="https://github.com/Intuex">Intuex</a> Organization</strong>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Python-3.9+-3776AB?style=for-the-badge&logo=python&logoColor=white" alt="Python">
  <img src="https://img.shields.io/badge/Gemini_AI-Robotics-8E75B2?style=for-the-badge&logo=google&logoColor=white" alt="Gemini">
  <img src="https://img.shields.io/badge/Hardware-Arduino-00979D?style=for-the-badge&logo=arduino&logoColor=white" alt="Arduino">
  <img src="https://img.shields.io/badge/License-Intuex_0.1_NC-yellow.svg?style=for-the-badge" alt="License">
  <img src="https://img.shields.io/badge/Org-Intuex-blue?style=for-the-badge&logo=github&logoColor=white" alt="Intuex">
</p>

---

## 📖 Overview

A real-time perception–action loop that allows you to control a robotic arm using **natural voice commands** and **live computer vision**. This system integrates speech-to-text, vision-language modeling (Gemini), and low-level serial control.

---

## ✨ Key Features

- 🎙️ **Push-to-Talk:** Record commands by holding the `SPACE` bar
- 🧠 **Gemini Reasoning:** Multi-step task planning using vision + logic
- 👁️ **Live Vision:** Compatible with ESP32-CAM, Android IP Webcam, or CCTV
- 🗣️ **Voice Feedback:** Real-time spoken updates on what the robot is thinking
- 🔁 **Stateful Loop:** Remembers previous actions for complex, multi-stage tasks

---

## 🚀 Quick Start

### 1. Installation

```bash
git clone https://github.com/Ansh-droid-glitch/GPTArm.git
cd GPTArm
pip install -r requirements.txt
```

### 2. Configure Environment

Create a `.env` file in the root directory:

```env
ASSEMBLYAI_API_KEY=your_key_here
GOOGLE_API=your_key_here
```

### 3. Launch the System

Run the specific script for your operating system:

| Platform | Execution Command |
|----------|-------------------|
| 🪟 **Windows** | `.\run.bat` |
| 🐧 **Linux** | `sh run.sh` |
| 🍎 **macOS** | `sh run_mac.sh` |

---

## 🛠️ System Architecture

> [!TIP]
> Each iteration of the loop is one reasoning step. The robot "sees," "thinks," "speaks," and then "moves."

1. **Input:** Voice (AssemblyAI) + Vision (Live Camera Frame)
2. **Analysis:** Gemini 1.5 Pro analyzes the scene and command
3. **Planning:** AI generates text-to-speech feedback and serial commands
4. **Execution:** Arduino moves the servos based on `MOVE` or `GRIP` strings

```
┌─────────────┐     ┌──────────────┐     ┌─────────────┐     ┌──────────────┐
│   Voice     │────>│   Gemini     │────>│  TTS + UI   │────>│   Arduino    │
│  Command    │     │   Vision AI  │     │  Feedback   │     │   Control    │
└─────────────┘     └──────────────┘     └─────────────┘     └──────────────┘
       │                    │
       └────────────┬───────┘
                    │
              ┌─────▼──────┐
              │   Camera   │
              │   Stream   │
              └────────────┘
```

---

## 📂 Project Structure

```
GPTArm/
├── main.py                 # Main control script
├── arduino/
│   └── servo_control.ino   # Arduino firmware
├── images/
│   └── input.jpg          # Camera captures stored here
├── audio/
│   └── recording.wav      # Voice recordings stored here
├── .env                   # API keys (create this)
├── requirements.txt       # Python dependencies
├── run.bat               # Windows launcher
├── run.sh                # Linux launcher
└── run_mac.sh            # macOS launcher
```

---

## 📝 Code Overview

### Main Components

The system consists of three main components:

#### 1. **Voice Input** (`record_audio()`)
- Hold `SPACE` to record your voice command
- Uses `sounddevice` for real-time audio capture
- Saves to `audio/recording.wav`

#### 2. **Vision Processing**
- Fetches live image from camera URL
- Sends image + voice command to Gemini AI
- AI analyzes scene and plans robot movements

#### 3. **Robot Control**
- Parses AI response for `MOVE` and `GRIP` commands
- Sends commands via serial to Arduino
- Maintains command history for context

### Main Script (`main.py`)

```python
import os
import time
import numpy as np
import sounddevice as sd
import keyboard
from scipy.io.wavfile import write
import assemblyai as aai
import pyttsx3
from dotenv import load_dotenv
from google import genai
from google.genai import types
import serial
import requests

# Load environment variables
load_dotenv()
aai.settings.api_key = os.getenv("ASSEMBLYAI_API_KEY")
GOOGLE_API_KEY = os.getenv("GOOGLE_API")

# Initialize Gemini client
MODEL_ID = "gemini-robotics-er-1.5-preview"
client = genai.Client(api_key=GOOGLE_API_KEY)

# Camera and file paths
URL = "http://192.168.1.15:8080/shot"  # Update with your camera IP
IMAGE_PATH = "images/input.jpg"
AUDIO_PATH = "audio/recording.wav"
SAMPLE_RATE = 44100
CHANNELS = 1

# Text-to-speech setup
engine = pyttsx3.init()
engine.setProperty("rate", 170)

def speak(text):
    """Convert text to speech"""
    engine.say(text)
    engine.runAndWait()

def record_audio():
    """Record audio while SPACE is held"""
    audio_chunks = []
    recording = False

    def callback(indata, frames, time, status):
        if recording:
            audio_chunks.append(indata.copy())

    print("Hold SPACE to record")

    with sd.InputStream(
        samplerate=SAMPLE_RATE,
        channels=CHANNELS,
        dtype="int16",
        callback=callback
    ):
        while True:
            if keyboard.is_pressed("space"):
                if not recording:
                    print("Recording...")
                    recording = True
            else:
                if recording:
                    print("Stopped recording")
                    break

    audio = np.concatenate(audio_chunks, axis=0)
    write(AUDIO_PATH, SAMPLE_RATE, audio)
    print("Audio saved")

# Main control loop
commands_history = []
count = 0

while True:
    print(f"\n===== STEP {count} =====")

    # Capture image from camera
    r = requests.get(URL, timeout=5)
    if r.status_code == 200:
        with open(IMAGE_PATH, "wb") as f:
            f.write(r.content)
        print("Image updated")
    else:
        print("Image download failed")
        continue

    # Record voice command
    record_audio()

    # Transcribe audio
    transcript = aai.Transcriber().transcribe(AUDIO_PATH)
    if transcript.status == "error":
        print("Transcription failed")
        continue

    base_prompt = transcript.text
    print("You said:", base_prompt)

    speak("Processing")

    # Build prompt with or without history
    if count == 0:
        prompt = f"""
You are a robotic arm.
User request: "{base_prompt}"

First line: explain briefly what you are doing.
Next lines: commands only:
MOVE x y z
GRIP OPEN
GRIP CLOSE
"""
    else:
        prompt = f"""
You are a robotic arm.
User request: "{base_prompt}"

Previous actions:
{commands_history}

First line: explain briefly what you are doing.
Next lines: commands only:
MOVE x y z
GRIP OPEN
GRIP CLOSE
"""

    # Send to Gemini with image
    with open(IMAGE_PATH, "rb") as f:
        image_bytes = f.read()

    response = client.models.generate_content(
        model=MODEL_ID,
        contents=[
            types.Part.from_bytes(data=image_bytes, mime_type="image/jpeg"),
            prompt
        ],
        config=types.GenerateContentConfig(
            temperature=0.5,
            thinking_config=types.ThinkingConfig(thinking_budget=0)
        )
    )

    raw_text = response.text or ""
    raw_text = raw_text.strip()

    print("\nMODEL RESPONSE:\n", raw_text)

    # Parse response
    lines = [l.strip() for l in raw_text.split("\n") if l.strip()]
    spoken = []
    commands = []

    for line in lines:
        if line.startswith("MOVE") or line.startswith("GRIP"):
            commands.append(line)
        else:
            spoken.append(line)

    # Speak feedback
    if spoken:
        speak(spoken[0])

    if not commands:
        print("No commands generated")
        continue

    # Send commands to Arduino
    ser = serial.Serial("COM7", 9600, timeout=2)  # Update COM port
    time.sleep(2)

    for cmd in commands:
        print("Sending:", cmd)
        ser.write((cmd + "\n").encode())
        time.sleep(1)

    ser.close()

    # Update history
    commands_history.extend(commands)
    count += 1
```

---

## 🔌 Arduino Setup

### Required Arduino Code (`servo_control.ino`)

```cpp
#include <Servo.h>

Servo servo1, servo2, servo3, servo4;

void setup() {
  Serial.begin(9600);
  servo1.attach(9);   // Base
  servo2.attach(10);  // Shoulder
  servo3.attach(11);  // Elbow
  servo4.attach(6);   // Gripper
  
  // Set to home position
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.startsWith("MOVE")) {
      // Parse: MOVE x y z
      int x, y, z;
      sscanf(command.c_str(), "MOVE %d %d %d", &x, &y, &z);
      
      // Map coordinates to servo angles (adjust as needed)
      servo1.write(constrain(x, 0, 180));
      servo2.write(constrain(y, 0, 180));
      servo3.write(constrain(z, 0, 180));
      
      Serial.println("MOVED");
    }
    else if (command == "GRIP OPEN") {
      servo4.write(180);  // Open position
      Serial.println("OPENED");
    }
    else if (command == "GRIP CLOSE") {
      servo4.write(0);    // Close position
      Serial.println("CLOSED");
    }
  }
}
```

### Wiring Diagram

```
Arduino Uno/Mega
├── Pin 9  → Base Servo (Signal)
├── Pin 10 → Shoulder Servo (Signal)
├── Pin 11 → Elbow Servo (Signal)
├── Pin 6  → Gripper Servo (Signal)
├── 5V     → Servo Power (or external 5V supply)
└── GND    → Common Ground
```

> [!IMPORTANT]
> Use an external power supply for servos if using more than 2 servos to avoid brownouts.

---

## 📋 Detailed Setup Guide

### Step 1: Create Project Directories

```bash
mkdir GPTArm
cd GPTArm
mkdir images audio arduino
```

### Step 2: Install Python Dependencies

Create `requirements.txt`:

```txt
numpy==1.24.3
sounddevice==0.4.6
keyboard==0.13.5
scipy==1.11.3
assemblyai==0.25.0
pyttsx3==2.90
python-dotenv==1.0.0
google-genai==0.2.0
pyserial==3.5
requests==2.31.0
```

Install dependencies:

```bash
pip install -r requirements.txt
```

### Step 3: Configure Camera

#### Option A: Android IP Webcam
1. Download "IP Webcam" app from Play Store
2. Open app and click "Start server"
3. Note the IP address (e.g., `192.168.1.15:8080`)
4. Update `URL` in `main.py`:
   ```python
   URL = "http://192.168.1.15:8080/shot.jpg"
   ```

#### Option B: ESP32-CAM
1. Flash ESP32-CAM with camera server firmware
2. Connect to your WiFi
3. Note the IP address
4. Update `URL` in `main.py`:
   ```python
   URL = "http://192.168.1.20:81/stream"
   ```

### Step 4: Get API Keys

#### AssemblyAI (Speech-to-Text)
1. Go to [AssemblyAI](https://www.assemblyai.com/)
2. Sign up for free account
3. Copy your API key from dashboard

#### Google Gemini (Vision AI)
1. Go to [Google AI Studio](https://aistudio.google.com/)
2. Create API key
3. Copy the key

Create `.env` file:

```env
ASSEMBLYAI_API_KEY=your_assemblyai_key_here
GOOGLE_API=your_google_api_key_here
```

### Step 5: Upload Arduino Code

1. Open Arduino IDE
2. Copy the servo control code to a new sketch
3. Select your board (Tools → Board)
4. Select correct COM port (Tools → Port)
5. Upload the sketch

### Step 6: Find Your Serial Port

#### Windows
1. Open Device Manager
2. Expand "Ports (COM & LPT)"
3. Note the COM port (e.g., `COM7`)
4. Update in `main.py`:
   ```python
   ser = serial.Serial("COM7", 9600, timeout=2)
   ```

#### Linux/macOS
```bash
ls /dev/tty*
# Look for /dev/ttyUSB0 or /dev/ttyACM0

# Grant permissions (Linux)
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyUSB0
```

Update in `main.py`:
```python
ser = serial.Serial("/dev/ttyUSB0", 9600, timeout=2)
```

### Step 7: Test Individual Components

#### Test Camera
```python
import requests
URL = "http://192.168.1.15:8080/shot.jpg"
r = requests.get(URL)
with open("test.jpg", "wb") as f:
    f.write(r.content)
print("Image saved as test.jpg")
```

#### Test Microphone
```python
import sounddevice as sd
print(sd.query_devices())  # List all audio devices
```

#### Test Serial Connection
```python
import serial
ser = serial.Serial("COM7", 9600, timeout=2)
ser.write(b"GRIP OPEN\n")
ser.close()
```

### Step 8: Run the System

```bash
python main.py
```

---

## 🎮 Usage Workflow

### Basic Operation

1. **Start the program**: `python main.py`
2. **Wait for prompt**: "Hold SPACE to record"
3. **Give command**: Hold SPACE, speak clearly, release SPACE
4. **AI processes**: Analyzes image + voice command
5. **Robot responds**: Speaks action and executes movement
6. **Repeat**: System ready for next command

### Example Commands

| Command | What Happens |
|---------|--------------|
| "Move to the red ball" | Camera finds red ball, arm moves there |
| "Pick up the cube" | Closes gripper on detected cube |
| "Place it over there" | Moves to indicated position |
| "Go to home position" | Returns to starting pose |
| "Open the gripper" | Opens claw |

---

## 🎛️ Configuration Options

### Camera Settings

```python
# In main.py, update these variables:
URL = "http://YOUR_CAMERA_IP:PORT/endpoint"
IMAGE_PATH = "images/input.jpg"
```

### Audio Settings

```python
SAMPLE_RATE = 44100  # CD quality
CHANNELS = 1         # Mono recording
```

### Voice Settings

```python
engine.setProperty("rate", 170)   # Speech speed (100-300)
engine.setProperty("volume", 1.0) # Volume (0.0-1.0)
```

### Serial Settings

```python
ser = serial.Serial(
    port="COM7",      # Your port
    baudrate=9600,    # Must match Arduino
    timeout=2         # Read timeout in seconds
)
```

---

## 🔍 Code Explanation

### Voice Recording Function

```python
def record_audio():
    """
    Records audio while SPACE key is held down.
    Uses real-time callback to capture audio chunks.
    Saves as WAV file when recording stops.
    """
    audio_chunks = []
    recording = False

    def callback(indata, frames, time, status):
        if recording:
            audio_chunks.append(indata.copy())
    # ... rest of implementation
```

### Main Control Loop

```python
while True:
    # 1. Capture current scene
    r = requests.get(URL, timeout=5)
    
    # 2. Get voice command
    record_audio()
    
    # 3. Transcribe speech
    transcript = aai.Transcriber().transcribe(AUDIO_PATH)
    
    # 4. Send to Gemini AI with image
    response = client.models.generate_content(...)
    
    # 5. Parse commands
    commands = [line for line in response if line.startswith("MOVE") or line.startswith("GRIP")]
    
    # 6. Execute on Arduino
    ser.write((cmd + "\n").encode())
```

### Command History System

```python
commands_history = []  # Stores all previous commands

# AI uses this context to plan multi-step tasks
prompt = f"""
Previous actions:
{commands_history}

New request: "{base_prompt}"
"""
```

---

## 💻 Platform Requirements

| OS | Extra Steps |
|----|-------------|
| **Linux** | `sudo apt install portaudio19-dev` |
| **macOS** | `brew install portaudio` |
| **Windows** | None (Works out of the box) |

---

## 🤖 Robot Command Format

The system communicates with the Arduino using a simple string protocol:

```plaintext
MOVE x y z          # Move to coordinates
GRIP OPEN           # Open the claw
GRIP CLOSE          # Close the claw
```

### Example Commands

```python
# Move arm to position
"MOVE 100 50 75"

# Pick up object
"GRIP CLOSE"

# Release object
"GRIP OPEN"
```

---

## 📦 Hardware Requirements

- Arduino board (Uno, Mega, or compatible)
- 4-6 DOF robotic arm with servo motors
- Camera (ESP32-CAM, IP Webcam, or USB webcam)
- USB cable for Arduino connection
- Power supply for servos (5V or as required)

---

## 🔧 Configuration

### Camera Setup

Edit the camera stream URL in your configuration file:

```python
# For ESP32-CAM
CAMERA_URL = "http://192.168.1.100:81/stream"

# For Android IP Webcam
CAMERA_URL = "http://192.168.1.101:8080/video"
```

### Serial Port Configuration

```python
# Windows
SERIAL_PORT = "COM3"

# Linux/macOS
SERIAL_PORT = "/dev/ttyUSB0"
```

---

## 🎯 Usage Examples

### Basic Pick and Place

1. Hold `SPACE` and say: "Pick up the red cube"
2. Robot analyzes the scene and moves to object
3. Hold `SPACE` and say: "Place it on the blue plate"
4. Robot completes the task

### Multi-Step Task

1. "Sort the blocks by color"
2. Robot plans and executes multiple pick-and-place operations
3. Provides voice feedback for each step

---

## 🛡️ Safety & Disclaimer

> [!WARNING]
> This project controls physical hardware. Please observe the following safety guidelines:

- ⚠️ Keep the robot's workspace clear of obstacles
- ⚠️ Ensure your Arduino has safety limits to prevent servo damage
- ⚠️ Always supervise the robot during operation
- ⚠️ Test in a controlled environment first
- ⚠️ Implement emergency stop functionality

---

## 📚 Dependencies Explained

| Package | Purpose | Why We Need It |
|---------|---------|----------------|
| `numpy` | Numerical operations | Audio array manipulation |
| `sounddevice` | Audio recording | Capture microphone input |
| `keyboard` | Keyboard input | Detect SPACE key press |
| `scipy` | Scientific computing | Save audio as WAV file |
| `assemblyai` | Speech-to-text | Convert voice to text |
| `pyttsx3` | Text-to-speech | Robot voice feedback |
| `python-dotenv` | Environment variables | Secure API key storage |
| `google-genai` | Gemini AI | Vision + language reasoning |
| `pyserial` | Serial communication | Talk to Arduino |
| `requests` | HTTP requests | Fetch camera images |

---

## ❓ Frequently Asked Questions

### Q: Can I use a USB webcam instead of IP camera?

Yes! Replace the image capture code with:

```python
import cv2

cap = cv2.VideoCapture(0)  # 0 for default webcam
ret, frame = cap.read()
cv2.imwrite(IMAGE_PATH, frame)
cap.release()
```

### Q: The robot moves too fast/slow. How do I adjust?

In the Arduino code, add delays between servo movements:

```cpp
servo1.write(x);
delay(500);  // Wait 500ms
servo2.write(y);
delay(500);
```

### Q: Can I add more servos?

Yes! Just add more servo objects and pins:

```cpp
Servo servo5, servo6;
servo5.attach(5);
servo6.attach(3);
```

Then update the command parsing in Arduino.

### Q: Why does speech recognition fail sometimes?

Common causes:
- Background noise (use in quiet environment)
- Microphone too far away
- Speaking too fast
- Poor internet connection (AssemblyAI is cloud-based)

### Q: Can this work offline?

Partially. You need internet for:
- AssemblyAI (speech-to-text)
- Gemini AI (vision reasoning)

Local alternatives:
- Use `speech_recognition` with Sphinx for offline STT
- Use local vision models (though less capable)

### Q: How accurate is the object detection?

Gemini 1.5 Pro is very accurate for:
- Object identification
- Spatial reasoning
- Color recognition

Limitations:
- Very small objects (<1cm)
- Poor lighting conditions
- Highly reflective surfaces

### Q: Can I control multiple robots?

Yes! Open multiple serial connections:

```python
robot1 = serial.Serial("COM7", 9600)
robot2 = serial.Serial("COM8", 9600)

# Send different commands to each
robot1.write(b"MOVE 90 90 90\n")
robot2.write(b"GRIP OPEN\n")
```

### Q: How do I add custom commands?

1. Update the Arduino code to handle new commands
2. Update the Gemini prompt to include new commands
3. Example for a new "ROTATE" command:

**Arduino:**
```cpp
else if (command.startsWith("ROTATE")) {
  int angle;
  sscanf(command.c_str(), "ROTATE %d", &angle);
  servo1.write(angle);
}
```

**Prompt:**
```python
prompt = f"""
Commands available:
MOVE x y z
GRIP OPEN
GRIP CLOSE
ROTATE angle
"""
```

---

## 🐛 Troubleshooting

### Common Issues

**Camera not connecting:**
```bash
# Verify camera URL is accessible
curl http://your-camera-ip:port/stream
```

**Audio input not working:**
```bash
# Test microphone
python -m sounddevice
```

**Arduino not responding:**
- Check serial port permissions (Linux/macOS): `sudo chmod 666 /dev/ttyUSB0`
- Verify baud rate matches Arduino sketch
- Ensure correct port is selected

### Common Issues

**Camera not connecting:**
```bash
# Verify camera URL is accessible
curl http://your-camera-ip:port/shot.jpg

# Or use browser to test
# http://192.168.1.15:8080/shot.jpg
```

**Audio input not working:**
```bash
# Test microphone
python -m sounddevice

# List audio devices
python -c "import sounddevice as sd; print(sd.query_devices())"
```

**Arduino not responding:**
- Check serial port permissions (Linux/macOS): `sudo chmod 666 /dev/ttyUSB0`
- Verify baud rate matches Arduino sketch (9600)
- Ensure correct port is selected
- Try unplugging and replugging USB cable
- Check if another program is using the serial port

**Servos jittering or not moving:**
- Use external power supply (5V, 2A minimum)
- Check all ground connections
- Verify servo wire connections
- Reduce load on servos
- Add capacitors (100μF) across power lines

**"ModuleNotFoundError" errors:**
```bash
# Reinstall all dependencies
pip install -r requirements.txt --force-reinstall

# If using Anaconda
conda install -c conda-forge <package-name>
```

**Gemini API errors:**
- Check API key is valid
- Verify you're using correct model ID
- Check API quota limits
- Ensure internet connection is stable

**Voice transcription is inaccurate:**
- Speak clearly and at moderate pace
- Reduce background noise
- Move microphone closer
- Use a better quality microphone
- Check microphone input levels in system settings

---

## ⚡ Performance Optimization

### Speed Improvements

```python
# 1. Reduce image resolution before sending to AI
from PIL import Image

img = Image.open(IMAGE_PATH)
img = img.resize((640, 480))  # Smaller = faster
img.save(IMAGE_PATH)
```

```python
# 2. Use threading for parallel operations
import threading

def capture_image_async():
    threading.Thread(target=capture_image).start()
```

```python
# 3. Cache repeated AI responses
response_cache = {}
cache_key = f"{base_prompt}_{image_hash}"

if cache_key in response_cache:
    response = response_cache[cache_key]
else:
    response = client.models.generate_content(...)
    response_cache[cache_key] = response
```

### Memory Optimization

```python
# Clear audio chunks after saving
audio_chunks.clear()

# Close serial connection when not in use
ser.close()
```

### Latency Reduction

```python
# Use lower quality audio recording
SAMPLE_RATE = 16000  # Instead of 44100

# Reduce thinking budget for faster responses
thinking_config=types.ThinkingConfig(thinking_budget=0)
```

---

## 🎥 Demo Videos

### Example Use Cases

**1. Pick and Place Task**
```
User: "Pick up the red cube and place it in the box"
Robot: "I'll pick up the red cube and move it to the box"
Actions: MOVE 120 80 60 → GRIP CLOSE → MOVE 90 90 90 → GRIP OPEN
```

**2. Color Sorting**
```
User: "Sort the blocks by color"
Robot: "I'll sort the blocks, starting with blue ones"
Actions: Multiple MOVE and GRIP sequences
```

**3. Object Stacking**
```
User: "Stack the three cubes on top of each other"
Robot: "I'll stack them starting from the bottom"
Actions: Precise MOVE commands with increasing Z values
```

---

## 🚀 Advanced Features

### Adding Camera Calibration

```python
import cv2
import numpy as np

# Calibrate camera to real-world coordinates
def pixel_to_coordinates(x_pixel, y_pixel):
    # Add your calibration matrix here
    x_real = (x_pixel - 320) * 0.1
    y_real = (y_pixel - 240) * 0.1
    return x_real, y_real
```

### Implementing Safety Limits

```python
# In Arduino code
int constrain_safe(int value, int min_val, int max_val) {
  if (value < min_val) return min_val;
  if (value > max_val) return max_val;
  return value;
}

// Usage
servo1.write(constrain_safe(x, 20, 160));  // Prevent extreme angles
```

### Adding Emergency Stop

```python
import threading

def emergency_stop_listener():
    while True:
        if keyboard.is_pressed('esc'):
            ser.write(b"STOP\n")
            print("EMERGENCY STOP ACTIVATED")
            break

# Start in background
threading.Thread(target=emergency_stop_listener, daemon=True).start()
```

### Logging System

```python
import logging
from datetime import datetime

logging.basicConfig(
    filename=f'robot_log_{datetime.now().strftime("%Y%m%d")}.txt',
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# Log all commands
logging.info(f"User command: {base_prompt}")
logging.info(f"AI response: {raw_text}")
logging.info(f"Executed: {commands}")
```

---

## 🔧 Alternative Hardware Configurations

### Budget Setup (~$50)

- **Arduino Uno** - $25
- **SG90 Micro Servos (4x)** - $10
- **USB Webcam** - $15
- **3D Printed Parts** - $5 (or cardboard)

```python
# Use USB webcam instead
import cv2
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
cv2.imwrite(IMAGE_PATH, frame)
```

### Mid-Range Setup (~$150)

- **Arduino Mega** - $40
- **MG996R Servos (6x)** - $50
- **ESP32-CAM** - $10
- **5V 10A Power Supply** - $20
- **Custom PCB Shield** - $15
- **Aluminum Frame** - $15

### Professional Setup (~$500+)

- **Arduino Due / Raspberry Pi 4** - $50
- **Dynamixel Servos** - $300
- **High-res IP Camera** - $100
- **Linear Actuators** - $50+
- **CNC Machined Parts** - Variable

---

## 🔌 Wiring Best Practices

### Power Distribution

```
External 5V Supply (10A)
    ├── Servo 1 (VCC, GND)
    ├── Servo 2 (VCC, GND)
    ├── Servo 3 (VCC, GND)
    ├── Servo 4 (VCC, GND)
    └── Arduino VIN (Optional: if not USB powered)

Arduino
    ├── Digital Pin 9  → Servo 1 Signal (Yellow)
    ├── Digital Pin 10 → Servo 2 Signal (Yellow)
    ├── Digital Pin 11 → Servo 3 Signal (Yellow)
    └── Digital Pin 6  → Servo 4 Signal (Yellow)

Common Ground Connection: Arduino GND ↔ Power Supply GND
```

### Safety Circuit

```cpp
// Add voltage monitoring in Arduino
int voltage_pin = A0;

void checkPower() {
  int voltage = analogRead(voltage_pin);
  if (voltage < 200) {  // ~1V threshold
    // Stop all servos
    servo1.detach();
    servo2.detach();
    servo3.detach();
    servo4.detach();
    Serial.println("LOW POWER - STOPPED");
  }
}
```

---

## 📱 Mobile App Integration (Optional)

Create a simple web interface for remote control:

```python
from flask import Flask, render_template, request

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('control.html')

@app.route('/command', methods=['POST'])
def handle_command():
    text_command = request.json['command']
    # Process command without voice
    # ... existing logic
    return {'status': 'success'}

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
```

---

## 🧪 Testing Framework

### Unit Tests

Create `test_robot.py`:

```python
import unittest
from main import parse_commands, validate_coordinates

class TestRobotControl(unittest.TestCase):
    
    def test_command_parsing(self):
        response = "I'll move to the object\nMOVE 90 90 90\nGRIP CLOSE"
        commands = parse_commands(response)
        self.assertEqual(len(commands), 2)
        self.assertEqual(commands[0], "MOVE 90 90 90")
    
    def test_coordinate_limits(self):
        self.assertTrue(validate_coordinates(90, 90, 90))
        self.assertFalse(validate_coordinates(200, 90, 90))

if __name__ == '__main__':
    unittest.main()
```

### Integration Test

```python
def test_full_pipeline():
    # 1. Test camera
    assert capture_image() == True
    
    # 2. Test audio
    assert record_audio() == True
    
    # 3. Test transcription
    transcript = transcribe_audio()
    assert len(transcript) > 0
    
    # 4. Test AI response
    response = get_ai_response(transcript, IMAGE_PATH)
    assert response is not None
    
    # 5. Test serial (with mock)
    commands = parse_commands(response)
    assert len(commands) > 0
```

---

## 🤝 Contributing

We welcome contributions from the community! This project is part of the [Intuex](https://github.com/Intuex) organization.

### How to Contribute

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

### Contribution Guidelines

- Follow the existing code style
- Write clear commit messages
- Add tests for new features
- Update documentation as needed
- Be respectful and constructive

For more details, see the [Intuex Contributing Guidelines](https://github.com/Intuex/.github/blob/main/CONTRIBUTING.md).

### Types of Contributions Welcome

- 🐛 Bug fixes
- ✨ New features
- 📝 Documentation improvements
- 🎨 UI/UX enhancements
- 🧪 Test coverage
- 🌐 Translations
- 💡 Ideas and suggestions

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 🗺️ Roadmap

### Current Features (v1.0)
- ✅ Voice control with push-to-talk
- ✅ Real-time vision processing
- ✅ Command history and context
- ✅ Text-to-speech feedback
- ✅ Serial Arduino control

### Planned Features (v2.0)
- 🔄 Continuous voice activation ("Hey Robot...")
- 🔄 Object tracking and following
- 🔄 Multiple camera angles
- 🔄 Web-based control interface
- 🔄 Gesture recognition
- 🔄 Offline mode with local AI models

### Future Enhancements (v3.0)
- 📋 Task queue and scheduling
- 📋 Multi-robot coordination
- 📋 AR visualization overlay
- 📋 Machine learning for custom tasks
- 📋 ROS integration
- 📋 Cloud telemetry and monitoring

---

## 🙏 Acknowledgments

### Organization
- **[Intuex](https://github.com/Intuex)** - Building the future of intelligent automation
- **Open Source Community** - For continuous support and contributions

### AI & APIs
- **Gemini AI** by Google - Powerful vision-language reasoning
- **AssemblyAI** - Accurate speech-to-text transcription
- **OpenAI** - Inspiration for conversational AI interfaces

### Hardware & Libraries
- **Arduino Community** - Extensive servo control resources
- **Python Community** - Amazing open-source libraries
- **PySerial Contributors** - Reliable serial communication
- **SoundDevice Developers** - Cross-platform audio recording

### Inspiration & Resources
- **MistralRobotics** - AI-powered robotics examples
- **ROS (Robot Operating System)** - Robotics best practices
- **OpenCV Community** - Computer vision techniques

### Special Thanks
- All contributors who submitted issues and pull requests
- Beta testers who helped identify bugs
- The robotics community for continuous support

---

## 📄 License

This project is licensed under the **Intuex 0.1 License**.

### Intuex 0.1 License Summary

This license allows you to:
- ✅ Use for personal projects
- ✅ Use for educational purposes
- ✅ Use for research and academic work
- ✅ Modify and distribute
- ✅ Use any patents in the code

Requirements:
- 📝 Include license and copyright notice
- 📝 Document any changes made
- 📝 Distribute derivatives under the same license
- 📝 Make source code available

Restrictions:
- ❌ **No commercial use without separate license**
- ❌ Cannot use "Intuex" trademark for endorsement

### Full License Text

```
Intuex Open Source License v0.1

Copyright (c) 2024 Intuex Organization & Ansh

Permission is hereby granted, free of charge, to any person or organization
obtaining a copy of this software and associated documentation files (the
"Software"), to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, subject to the following conditions:

1. Attribution Requirement:
   - The above copyright notice and this permission notice shall be included
     in all copies or substantial portions of the Software.
   - Any modifications must be clearly documented and attributed.

2. Open Source Commitment:
   - Derivative works must be distributed under the same Intuex 0.1 License.
   - Source code must be made available for any distributed binary forms.

3. Patent Grant:
   - Contributors grant a perpetual, worldwide, royalty-free patent license
     to use any patent claims implemented in their contributions.

4. Trademark Protection:
   - The name "Intuex" and associated logos may not be used to endorse
     derived products without explicit written permission.

5. No Warranty:
   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
   OR IMPLIED, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT.

6. Limitation of Liability:
   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
   CLAIM, DAMAGES, OR OTHER LIABILITY ARISING FROM THE SOFTWARE.

For licensing inquiries: https://github.com/Intuex
```

---

## 🏢 Part of Intuex

This project is maintained by the [**Intuex**](https://github.com/Intuex) organization - building the future of intelligent automation.

<p align="center">
  <a href="https://github.com/Intuex">
    <img src="https://img.shields.io/badge/Explore_More-Intuex_Projects-blue?style=for-the-badge&logo=github" alt="Intuex">
  </a>
</p>

---

## 🔗 Useful Links

- 📚 [Gemini API Documentation](https://ai.google.dev/docs)
- 🎙️ [AssemblyAI Docs](https://www.assemblyai.com/docs)
- 🤖 [Arduino Reference](https://www.arduino.cc/reference/en/)
- 🐍 [Python Serial Tutorial](https://pyserial.readthedocs.io/)
- 📹 [OpenCV Python Tutorials](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html)

---

## 💬 Community & Support

### Get Help
- 🐛 [Report a Bug](https://github.com/Ansh-droid-glitch/GPTArm/issues)
- 💡 [Request a Feature](https://github.com/Ansh-droid-glitch/GPTArm/issues)
- 💬 [Join Discussions](https://github.com/Ansh-droid-glitch/GPTArm/discussions)

### Stay Updated
- ⭐ Star this repository to follow updates
- 👁️ Watch for new releases
- 🔔 Enable notifications for important changes

### Share Your Build
Made something cool with this project? We'd love to see it!
- Tag us on social media
- Submit a pull request with your modifications
- Share photos/videos in the discussions

---

## 📊 Project Stats

<p align="center">
  <img src="https://img.shields.io/github/stars/Ansh-droid-glitch/GPTArm?style=social" alt="Stars">
  <img src="https://img.shields.io/github/forks/Ansh-droid-glitch/GPTArm?style=social" alt="Forks">
  <img src="https://img.shields.io/github/watchers/Ansh-droid-glitch/GPTArm?style=social" alt="Watchers">
</p>

<p align="center">
  <img src="https://img.shields.io/github/issues/Ansh-droid-glitch/GPTArm" alt="Issues">
  <img src="https://img.shields.io/github/issues-pr/Ansh-droid-glitch/GPTArm" alt="Pull Requests">
  <img src="https://img.shields.io/github/last-commit/Ansh-droid-glitch/GPTArm" alt="Last Commit">
</p>

---

<p align="center">
  <strong>Built with ❤️ by <a href="https://github.com/Intuex">Intuex</a> • Powered by Gemini AI & AssemblyAI</strong>
</p>

<p align="center">
  <a href="https://github.com/Ansh-droid-glitch/GPTArm">⭐ Star this repo</a> • 
  <a href="https://github.com/Intuex">🚀 Explore Intuex</a> • 
  <a href="https://github.com/Ansh-droid-glitch/GPTArm/issues">🐛 Report Issues</a>
</p>

<p align="center">
  <sub>Open source robotics • AI-powered automation • Community-driven innovation</sub>
</p>