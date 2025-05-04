# Point-to-Point Robot Controller

## Visualization

### RViz2
![RViz2 View](image.png)

### Gazebo
![Gazebo View](image-1.png)

## About this project
This project is implemented using ROS 2 Foxy, RViz2, and Gazebo. It enables point-to-point control of a robot.

## How to run

### 1. Clone the repository
```bash  
git clone https://github.com/Imotechs/point-to-point-controller.git 

cd point-to-point-controller  
colcon build  

3. Launch Gazebo and RViz to spawn the robot in the environment

ros2 launch controller_bringup gazebo.launch.xml  

4. Move the robot
ros2 run robot_controller move_robot  

5. Watch the robot move...

'''