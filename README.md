# ros_raspi_car: Turn a Remote Controlled Car into an Autonomous Robot via a Raspberry Pi 4 and ROS
This repo contains the ROS packages developed or used to turn a remote controlled car into an autonomous robot! For more details visit the corresponding blog on my website [https://www.moritzboeker.de/](https://www.moritzboeker.de/2022/08/ros-raspi-car/)

## Materials
- remote controlled car
- Raspberry Pi 4
- 16 channel PWM board based on PCA9685 chip
- hall sensor
- neodymium magnets

## How to remote control your car via [ROS-Mobile App](https://github.com/ROS-Mobile/ROS-Mobile-Android)
- ROS package `cmd_vel_to_pwm` (requires ROS package ros-pwm-pca9685)
	- ROS node `cmd_vel_to_ackermann_drive`
		- Subscribes the velocity command via ROS topic `cmd_vel` as `Twist`, converts it to the Ackermann-suitable type `AckermannDriveStamped` and publishes it to `/ackermann_cmd_topic`
	- ROS node `ackermann_drive_to_pwm`
		- Subscribes to ROS topic `ackermann_cmd_topic` as `AckermannDriveStamped`, converts it to PWM values for servo motors and publishes it as `Int32MultiArray` to `/command`, from there it is sent to the PWM board and eventually to the servo motors
	- Adapt the parameters in `cmd_vel_to_pwm.launch` to your PWM board, especially the ones regarding the `pwm_pca9685_node`, e.g. device, address, frequency etc.
	- Launch the two nodes `cmd_vel_to_ackermann_drive` and `ackermann_drive_to_pwm` via `$roslaunch cmd_vel_to_pwm cmd_vel_to_pwm.launch`
- Download & install & launch ROS-Mobile App for Android on your phone
	- Enter the IP-address of your Raspberry Pi and connect
		- method 1 (at home): Raspberry Pi and your phone are connected to same local Wifi, the [Ubuntu image from Ubiquity](https://learn.ubiquityrobotics.com/noetic_pi_image_downloads) does so automatically if your local Wifi is present
		- method 2 (on field): Raspberry Pi creates its own access point and your phone connects to it, the [Ubuntu image from Ubiquity](https://learn.ubiquityrobotics.com/noetic_pi_image_downloads) does so automatically if your local Wifi is NOT present
	- Add a joystick widget
	- Remote control your car and enjoy!

## Changes to Ydlidar ROS1 driver
- The original [ROS driver for YDLidar](https://github.com/YDLIDAR/ydlidar_ros_driver) depends on the [YDLidar SDK](https://github.com/YDLIDAR/YDLidar-SDK)
- For some reason, I could not manage to install YDLidar SDK
- Therefore, I used this third-party YDLidar ROS driver from [EAIBOT](https://github.com/EAIBOT/ydlidar) which works without YDLidar SDK
- However, with this package the `LaserScan` could not be transformed to any other frame besides the `laser` frame
- But the solution was provided by a comment in this [issue](https://github.com/EAIBOT/ydlidar/issues/14)
- edit ydlidar/src/ydlidar_node.cpp by changing the following lines
  - L134: scan_msg.scan_time = scan.config.scan_time; →scan_msg.scan_time = scan.config.scan_time/1000000000ul;
  - L135: scan_msg.time_increment = scan.config.time_increment; → scan_msg.time_increment = scan.config.time_increment/1000000000ul;
- build the workspace again
- ROS should now be able to transform the LaserScan into arbitrary frames of the tf tree
- You can check this by displaying the LaserScan in Rviz with a fixed fram set e.g., to `base_link` or `odom`


## How to add odometry to a remote controlled car
- Identify the drive shaft of the rc car and determine its diameter
- Construct and 3d-print a disk with holes to insert magnets alternating with their south and north pole
- Attach the disk to the drive shaft of the rc car
- Firmly install a hall sensor to the frame of the rc car with minimal distance to the disk
- Connect the hall sensor to one of the Raspberry Pi's digital inputs
- ROS package `wheel_encoder`
	- ROS node `wheel_encoder_node`
		- Publishes the readings of the hall sensor to ROS topic `/encoder_ticks` as `Bool`: `True`, when a south and a north pole passed by the hall sensor (or the other way round); `False`, otherwise
		- Publishes the linear speed of the rc car to ROS topic `/encoder_speed` as `Float32` in [m/s] 
	- ROS node `calibrate_encoder_node`
		- Used to calibrate the wheel encoder to know how many meters you car travels for each odometry tick
		- terminal 1: `$ roslaunch wheel_encoder calibrate_encoder.launch`
		- terminal 2: `$ rosrun wheel_encoder calibrate_encoder_node.py`
		- Follow the instructions in terminal 2, e.g.:
		~~~
		Enter linear velocity in [m/s]:0.2
		Enter time period of ride in [s]:16
		Estimated distance in [m]: 3.2
		Press Enter to drive the robot...
		16.0 s left, sum odom ticks 0
		15.9 s left, sum odom ticks 2
		15.8 s left, sum odom ticks 4
		…
		0.2 s left, sum odom ticks 589
		0.1 s left, sum odom ticks 593
		-0.0 s left, sum odom ticks 597
		Measure and enter the actual distance in [m]:5.13
		For each odometry tick the robot travels 0.008593 m
		~~~
		- Copy & paste the determined value (here: 0.008593 m) into `wheel_encoder.launch` as value for the parameter `meter_per_ticks`
		- Launch the calibrate velocity with `$ roslaunch wheel_encoder wheel_encoder.launch`

