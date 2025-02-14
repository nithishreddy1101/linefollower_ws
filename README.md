# **Line Follower Robot using ROS 2 and OpenCV** 🏎️🤖  

![Project Banner](https://user-images.githubusercontent.com/your_image_link)   

## **📌 Project Description**
This project is a **line-following robot** that uses a **camera** to detect and follow a path using **ROS 2, OpenCV, and cv_bridge**. It processes live camera feed, detects the track, and adjusts the robot’s movement accordingly.

---

## **📦 Features**
✅ Detects and follows a black/white line  
✅ Uses OpenCV for image processing  
✅ ROS 2 `rclcpp` for real-time processing  
✅ Publishes velocity commands using `geometry_msgs/Twist`  

---

## **🚀 Installation & Setup**

### **1️⃣ Install Dependencies**
Ensure your system has the required packages:
```sh
sudo apt update
sudo apt install ros-humble-desktop libopencv-dev python3-opencv -y
```

### **2️⃣ Clone the Repository**
```sh
cd ~/ROS/linefollower_ws/src
git clone https://github.com/nithishreddy1101/linefollower_ws.git
cd ..
colcon build --packages-select line_follower
source install/setup.bash
```

### **3️⃣ Run the Line Follower Node**
Launch the camera node and the line follower:
```sh
ros2 run line_follower image_cv.py
```



## **🛠️ Usage & Customization**
- **Change the threshold values** for line detection in `image_processing.cpp`
- Modify the `Twist` velocity commands in `control.cpp` to adjust speed
- Add a **Gazebo simulation** for testing in ROS 2

---

## **🤝 Contributing**
Contributions are welcome! Feel free to:
1. **Fork** this repository
2. Create a **new branch**:  
   ```sh
   git checkout -b feature-branch
   ```
3. **Commit** your changes:  
   ```sh
   git commit -m "Added new feature"
   ```
4. **Push** to GitHub and open a **Pull Request**

---
