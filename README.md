# SAND-GRAIN-SIZE-EVALUATOR

A field-deployable computer vision based system for automated sand grain size measurement with real-time geo-tagging and cloud synchronization.

Demo / Working Video:  
https://drive.google.com/drive/folders/1xgOGHSTq3k1kqSckc6GRWQICgBZ23wUu

---

## Project Overview

This project estimates sand grain size using controlled image acquisition and computer vision techniques.

The system was designed specifically for irregular environments such as dune beaches, where surface unevenness makes direct calibration unreliable. A hybrid hardware and software pipeline was developed to:

- Maintain fixed imaging height  
- Ensure perpendicular camera alignment  
- Calibrate real-world area per pixel  
- Estimate grain diameter using segmentation  
- Geo-tag each measurement  
- Store data locally and push to cloud when online  

The system is designed for autonomous field operation.

---

## Core Concept

Accurate grain measurement requires:

1. Constant real-world area per pixel  
2. Stable camera height  
3. Perpendicular camera orientation  
4. Reliable area calibration  
5. Robust data storage  

This project addresses all these constraints using sensor fusion and geometric calibration.

---

## System Architecture

### 1. Height Stabilization (Virtual Plane Approximation)

Dune beaches have uneven surfaces. To approximate a virtual flat plane:

- Five ultrasonic sensors are mounted at the back.
- Distance readings are stored in an array.
- The median of readings is calculated.
- Only values within the range:
&emsp; median / 1.5 to median * 1.5 are considered valid.
- The mean of filtered readings is computed.
- Image capture is allowed only if the mean distance lies between 9 cm and 11 cm.

This ensures consistent imaging height despite irregular terrain.

---

### 2. Orientation Control (Perpendicular Constraint)

An MPU6050 accelerometer and gyroscope ensures that the camera is perpendicular to the ground.

Image capture is triggered only when:
- Device tilt is within an acceptable threshold  
- Height condition is satisfied  

This prevents perspective distortion and area miscalculation.

---

### 3. Real-World Area Calibration

With fixed height:

- Horizontal Field of View (HFOV)
- Vertical Field of View (VFOV)

are used to compute:

- Real-world image width  
- Real-world image height  
- Real-world area  

Then:
- area_per_pixel = real_world_area / total_pixels
- This enables conversion from pixel measurements to real-world units.
---

### 4. Grain Segmentation and Size Estimation

Using computer vision:

- Water-based segmentation isolates sand grains.
- Average pixel area per grain is computed.
- Equivalent grain diameter is calculated from pixel area.

This provides quantitative grain size estimation.

---

### 5. Geo-Tagging

A Neo6MV2 GPS module (u-blox chipset) retrieves:

- Latitude  
- Longitude  

Each measurement is stored with geographic coordinates.

---

### 6. Data Storage and Cloud Synchronization

All processed data is stored locally in a CSV file.

Each record includes:
- Latitude  
- Longitude  
- Mean height  
- Area per pixel  
- Estimated grain diameter  
- Timestamp  

---

## upload.py Explanation

The upload.py file handles internet connectivity and cloud synchronization.

Workflow:

1. Pings 8.8.8.8 (Google DNS server) to check internet connectivity.
2. If online:
   - Reads local CSV data
   - Pushes data to a MongoDB backend server
   - Backend is deployed on Render
3. If offline:
   - Waits
   - Periodically retries
   - Pushes data once connection is restored

This ensures no data loss and reliable field deployment.

---

## Hardware Used

- Raspberry Pi 2B+ (Main processing and control)  
- Arduino Leonardo (Sensor data acquisition)  
- 5 Ultrasonic Sensors (Height measurement)  
- MPU6050 Accelerometer/Gyroscope (Tilt detection)  
- Neo6MV2 GPS Module (Geo-tagging)  
- USB Webcam (Image capture)  
- Raspberry Pi 3.5-inch TFT Display (User interface)  

---

## Software Stack

- Python  
- OpenCV  
- NumPy  
- CSV file handling  
- MongoDB  
- Render (Cloud backend hosting)  

---

## Output

Each measurement generates:

- Grain size estimation  
- Geographic coordinates  
- Height validation  
- CSV entry  
- Cloud database entry (if online)  

---

## Key Features

- Sensor fusion for stable acquisition  
- Field-deployable design  
- Real-time geo-tagging  
- Offline data persistence  
- Automatic cloud synchronization  
- Autonomous workflow  

---

## How to Run

1. Clone the repository:

```bash
git clone https://github.com/Hemant-Khurana-007/SAND-GRAIN-SIZE-EVALUATOR.git
cd SAND-GRAIN-SIZE-EVALUATOR
```
2. Upload the Arduino code to the Arduino Leonardo using the Arduino IDE.

3. Upload and configure the Raspberry Pi code on the Raspberry Pi.

4. Connect all hardware components as shown in the circuit diagram

5. Power on the system and run main.py


---
## Future Improvements

- Deep learning based segmentation  
- Real-time map visualization  
- Automated beach classification  
- Mobile dashboard interface  
- 3D grain reconstruction  
