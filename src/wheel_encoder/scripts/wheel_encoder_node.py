#!/usr/bin/env python3

import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Bool, Float64, Int32MultiArray


class Encoder():
    def __init__(self):
        self.encoder_pin = rospy.get_param('~encoder_pin', default=13)
        self.meter_per_ticks = rospy.get_param('~meter_per_ticks', default=0.01)
        self.motor_pwm_center = rospy.get_param('~motor_pwm_center', default=5000)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.encoder_pin, GPIO.IN)
        rate = rospy.Rate(100)
        pub_ticks = rospy.Publisher('encoder_ticks', Bool, queue_size=10)
        pub_speed = rospy.Publisher('encoder_speed', Float64, queue_size=10)
        rospy.Subscriber('command', Int32MultiArray, self.motor_pwm_callback, queue_size=10)
        self.ticks_msg = Bool() # whether or not an encoder tick was registered
        self.speed_msg = Float64() # linear velocity of vehicle
        self.old_magnet_reading = False
        self.cur_magnet_reading = False
        self.direction_factor = 1.0
        self.ticks_ctr = 0 # the number of encoder ticks counted within the time period
        self.time_window = 0.25 # the duration in seconds for which encoder ticks are counted
        self.last_time = rospy.Time.now()
        while not rospy.is_shutdown():
            self.read_encoder_ticks()
            self.calc_speed()
            pub_ticks.publish(self.ticks_msg)
            pub_speed.publish(self.speed_msg)
            rate.sleep()

    def read_encoder_ticks(self):
        self.cur_magnet_reading = GPIO.input(self.encoder_pin)
        if self.old_magnet_reading ^ self.cur_magnet_reading: # when south pole and north pole passed --> one encoder tick
            self.ticks_msg.data = True
            self.ticks_ctr += 1
        else:
            self.ticks_msg.data = False
        self.old_magnet_reading = self.cur_magnet_reading

    def calc_speed(self):
        if rospy.Time.now() - self.last_time >= rospy.Duration.from_sec(self.time_window):
            ticks_per_second = self.ticks_ctr / self.time_window
            self.speed_msg.data = self.direction_factor * self.meter_per_ticks * ticks_per_second # in meters per second
            self.ticks_ctr = 0
            self.last_time = rospy.Time.now()

    def motor_pwm_callback(self, msg):
        motor_pwm = msg.data[0]
        if motor_pwm >= self.motor_pwm_center:
            self.direction_factor = 1.0 # driving forwards
        else:
            self.direction_factor = -1.0 # driving backwards


if __name__ == '__main__':
    rospy.init_node('wheel_encoder_node', anonymous=True)
    Encoder()