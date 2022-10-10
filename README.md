# ros_raspi_car: Turn a Remote Controlled Car into an Autonomous Robot via a Raspberry Pi 4 and ROS
This repo contains the ROS packages I used to turn a remote controlled car into an autonomous robot.
## Materials
- remote controlled car
- Raspberry Pi 4
- 16 channel PWM board
- hall sensor
- neodymium magnets

## Use and Calibrate your Custom Odometry
- Attach a disk with magnets to the drive shaft of the car model.
- Attach a hall sensor to the frame of the car close to the disk.
- Use the Raspberry Pi's digital input to read when a magnet passed by the hall sensor.
- Publish the readings [True, False] on the ROS topic `/encoder_ticks`
-  Calibrate the odometry to know how many meters you car travels for each odometry tick by
	- `$ roslaunch odometry calibrate_odom.launch`
	- `$ rosrun odometry calibrate_odom_node.py$`
	- Follow the instructions in the terminal
	- Write down the experimentally determined value

