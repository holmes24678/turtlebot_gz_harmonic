
# Turtlebot Simulation in Gazebo Harmonic

This project uses Turtlebot3 Model to simulate it in the Gazebo Harmonic. In this project i used the following Gazebo Plugin.

Lidar Plugin | Camera Plugin | Depth Camera Plugin | Differential Drive Plugin 

### 1. Cloning the Repository 
```bash
git clone https://github.com/holmes24678/turtlebot_gz_harmonic.git

```
### 2. Building and Cloning the workspace
```bash
colcon build 
source ~/install/setup.bash
```
### 3. Launching the robot in Gazebo
```bash
ros2 launch turtlebot_sim turtlebot.launch.py
```

#### You will get the following output
![Screenshot from 2025-01-23 16-54-00](https://github.com/user-attachments/assets/0b61ccbd-140f-427e-bac3-db42df0089ec)

### 4.Controlling the robot
Select Telop Gazebo Plugin to control the Robot using Keyboard
#### You will get the following output
![Screenshot from 2025-01-23 16-54-59](https://github.com/user-attachments/assets/36ab06f9-5bd2-4644-91fb-2b2701b69a0f)

### 5. using Lidar Plugin
Add the Lidar plugin to URDF and then use Visualize Lidar to view the Laser Data
#### You will get the following output
![Screenshot from 2025-01-23 16-53-00](https://github.com/user-attachments/assets/4bafdd41-3394-4d2c-bdd7-9cee64318f75)

### 6. Using Depth Camera/Camera
Add the camera Plugin to URDF and then use Image Display to view the frames

for Depth Camera : in Camera plugin file sensor tag keep type as "depth_camera"

for Camera : in Camera plugin file sensor tag keep type as "camera"

#### You will get the following output
Depth Camera
![Screenshot from 2025-01-23 16-50-04](https://github.com/user-attachments/assets/0f8ef8c3-af20-4340-8df4-a4ee151977df)
Camera
![Screenshot from 2025-01-23 16-48-32](https://github.com/user-attachments/assets/eaa9383f-5e7a-4813-a291-dd6c95fef7c9)
