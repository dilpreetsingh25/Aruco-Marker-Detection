# ARUCO Detection & Tracking in ROS 2

This repository contains a ROS 2 workspace for real-time aruco detection and tracking using OpenCV and deep learning models.

## **1. Installation & Setup**
### **Prerequisites**
Ensure you have **ROS 2 Humble** installed. Then install required dependencies:
```bash
sudo apt update && sudo apt install -y \
    ros-humble-vision-opencv \
    ros-humble-image-transport \
    python3-opencv \
    python3-opencv-contrib-python
```

### **Clone and Build**
```bash
mkdir -p ~/object_tracking_ws/src
cd ~/object_tracking_ws/src
git clone (https://github.com/dilpreetsingh25/Aruco-Marker-Detection.git)
cd ~/object_tracking_ws
colcon build --symlink-install
source install/setup.bash
```

## **2. Running the Aruco Detection & Tracking System**

### **Launch the Detection Pipeline**
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### **Launch the Tracking Node**
```bash
python3 src/aruco_detect_gazebo/aruco_detector.py
```

## **3. Repository Structure**
```
object_tracking_ws/
│── src/
│   ├── object_detection/       # ROS 2 package for detection
│   │   ├── nodes/              # Python/C++ scripts for detection
│   │   ├── launch/             # Launch files
│   │   ├── config/             # Config files (if needed)
│   │   ├── CMakeLists.txt      # If using C++
│   │   ├── package.xml
│   ├── object_tracking/        # ROS 2 package for tracking
│   │   ├── nodes/
│   │   ├── launch/
│   │   ├── config/
│   │   ├── package.xml
│   ├── ...
│
│── README.md                   # Documentation
│── setup.sh                     # Optional: Script for setting up dependencies
```

## **4. How It Works**
1. **Object Detection**: Uses OpenCV or a deep-learning model (YOLO, MobileNet, etc.) to detect objects.
2. **Tracking**: Implements a tracking algorithm like Kalman Filter or SORT to track objects across frames.
3. **ROS 2 Topics**:
   - `/camera/image_raw` → Input image stream
   - `/detections` → Detected objects (bounding boxes)
   

## **5. Dependencies**
- ROS 2 Humble
- OpenCV
- NumPy
- Image Transport (for ROS 2 image handling)

## **6. Contributing**
Feel free to open issues or submit pull requests to improve this project!

## **7. License**
This project is licensed under the MIT License. See `LICENSE` for details.

---
🚀 Happy Coding!

