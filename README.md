CafeBot - Autonomous Delivery Robot
Cafe bot is a robot used as a substitute for butlers. THe robot can perform the task of taking the order from the user and when received it will collect the food from the kitchen and place it in the respective table. Once the order is completed the robot moves to the home position and waits for the next order. The robot is used to make the working faster.


Features
* Takes customer orders via a command-line interface.
* Navigates autonomously to the kitchen to pick up food.
* Delivers food to selected tables while allowing order confirmation.
* Runs on ROS 2 Humble using nav2_simple_commander for navigation.
Installation
Prerequisites
Ensure you have the following installed:
* Ubuntu 22.04 (or compatible system)
* ROS 2 Humble
* Nav2 (Navigation Stack 2)
* nav2_simple_commander package


-Created a world file fr_cafe.world 
Mapped the whole cafe and saved the yaml file as “my_cafe_map.yaml”
* Created a launch file “cafe.launch.py”
*contain the newly created world file






FOR CREATING A MAP WITH THE NEW WORLD FILE


LAUNCH SLAM
ros2 launch turtlebot3_navigation2 slam_launch.py use_sim_time:=True
-To save map:
ros2 run nav2_map_server map_saver_cli -f my_cafe_map(FILE_NAME)


Terminal1:
ros2 launch turtlebot3_gazebo cafe.launch.py


Terminal2:(check the code)
 ros2 launch nav2_bringup localization_launch.py map:=/path/to/yaml file/ use_sim_time:=True


Terminal3:
ros2 launch turtlebot3_navigation2 navigation2.launch.py


Initial Pose Estimation must be performed before running Navigation as this process initializes the AMCL parameters that are critical for accurate Navigation. TurtleBot3 has to be correctly located on the map with the LDS sensor data that neatly overlaps the displayed map.
1. Click the 2D Pose Estimate button in the RViz2 menu.
2. Click on the map where the actual robot is located and drag the large green arrow toward the direction where the robot is facing.
3. Repeat step 1 and 2 until the LDS sensor data is overlaid on the saved map.




Terminal4:
ros2 run my_cafe_bot cafe_bot


PLACING ORDER:


CafeBot Order System:
Press 1 for Table 1
Press 2 for Table 2
Press 3 for Table 3
Press 9 to start the robot
Enter order (or 9 to start):


USER MANUAL


To place the order press ‘1’ for table 1 ,’2’ for table 2 and ‘3’ for table 3. After placing the order press ‘9’ so the robot moves.
Press ‘0’ to deploy the order in the table.








Order Processing Logic
* The robot moves to the kitchen first.
* It then delivers food to the selected tables.
* At each table, press 0 to confirm food placement.
* If 0 is pressed at all tables → Directly returns home.
* If 0 is NOT pressed at any table → Goes back to the kitchen first, then home.
* The robot always returns home after completing deliveries.






















EXAMPLE OUTPUT