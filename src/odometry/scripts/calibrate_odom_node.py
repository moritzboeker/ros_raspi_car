#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class Calibration():
    def __init__(self):
        self.ui_vel_lin = 0.0
        self.ui_t_dur = 0.0
        self.ui_dist = 0.0
        self.meters_per_tick = 0.0
        self.sum_encoder_ticks = 0
        self.permission = False
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.encoder_sub = rospy.Subscriber("/encoder_ticks", Bool, self.encoder_callback, queue_size=1)
        self.rate = rospy.Rate(10) # 10hz
        self.menu()

    def menu(self):
        print("This tool allows you to calibrate the odometry for your robot.")
        print("It determines how many meters you car travels during a single odometry tick.")
        print("The car drives forward with a specified linear velocity for a given period of time.")
        print("During this process it counts the odometry ticks.")
        print("You then have to enter the actual distance travelled.")
        print("A typical value is e.g., 0.01 m / tick.")
        print("At first you need to execute: $ roslaunch odometry calibrate_odom.launch")
        self.ui_vel_lin = float(input("Enter linear velocity in [m/s]:"))
        self.ui_t_dur = float(input("Enter time period of ride in [s]:"))
        print("Estimated distance in [m]: {:.1f} ".format(self.ui_vel_lin*self.ui_t_dur))
        input("Press Enter to drive the robot...")
        self.run_robot()
        self.ui_dist = float(input("Measure and enter the actual distance in [m]:"))
        try:
            self.meters_per_tick = self.ui_dist / self.sum_encoder_ticks
        except ZeroDivisionError:
            print("Sorry, no encoder ticks were registered!")
        print("For each odometry tick the robot travels {:.3f} m".format(self.meters_per_tick))

    def encoder_callback(self, msg):
        if msg.data and self.permission:
            self.sum_encoder_ticks += 1
    
    def run_robot(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = 0.0 # robot drives straight
        self.timestamp = rospy.Time.now()
        while not rospy.is_shutdown():
            time_left = rospy.Duration.to_sec(rospy.Duration.from_sec(self.ui_t_dur) - (rospy.Time.now() - self.timestamp))
            print("{:5.1f} s left, sum odom ticks {:d}".format(time_left, self.sum_encoder_ticks))
            if rospy.Time.now() - self.timestamp < rospy.Duration.from_sec(self.ui_t_dur):
                cmd_vel_msg.linear.x = self.ui_vel_lin # go
                self.cmd_vel_pub.publish(cmd_vel_msg)
                self.permission = True
            else:
                cmd_vel_msg.linear.x = 0.0 # stop
                self.cmd_vel_pub.publish(cmd_vel_msg)
                self.permission = False
                break
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node("calibrate_odom_node", anonymous=True)
    Calibration()