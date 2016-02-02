# ackerman_drive_teleop
ROS teleoperation scripts for robots with ackermann steering

##### ackermann_drive_keyop
+ Run the teleoperation script, with  
`rosrun ackermann_drive_teleop ackermann_drive_keyop`  
+ You can set max speed and steering angle, by giving them as arguments, when running the script.  
ie. `rosrun ackermann_drive_teleop ackermann_drive_keyop 0.5 0.8`  
+ Use the "up", "down" arrow keys to control speed and "left" and "right" arrow keys to control the steering angle.  

##### ackermann_drive_joyop
+ Run the teleoperation script, as well as the joy node using the following command:  
`roslaunch ackermann_drive_teleop ackermann_drive_joyop.launch`  
+ You can set max speed and steering angle, by giving them as arguments to the launcher.  
ie. `roslaunch ackermann_drive_teleop ackermann_drive_joyop.launch max_speed:=0.5 max_angle:=0.8`  
+ **In order to use a joystick, it must have read and right permissions.**  
You can grant such permissions by executing the following command: `sudo chmod a+rw /dev/input/js0`
