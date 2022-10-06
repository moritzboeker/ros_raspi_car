#!/usr/bin/env python3

import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Bool


class Encoder():
    def __init__(self):
        self.encoder_pin = rospy.get_param('~encoder_pin', default=13)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.encoder_pin, GPIO.IN)
        rate = rospy.Rate(10) # 10hz
        pub_encoder = rospy.Publisher('encoder_ticks', Bool, queue_size=10)
        self.encoder_msg = Bool()
        self.old_magnet_reading = False
        self.cur_magnet_reading = False
        while not rospy.is_shutdown():
            self.readTicks()
            pub_encoder.publish(self.encoder_msg)
            rate.sleep()

    def readTicks(self):
        self.cur_magnet_reading = GPIO.input(self.encoder_pin)
        if self.old_magnet_reading ^ self.cur_magnet_reading:
            self.encoder_msg.data = True
        else:
            self.encoder_msg.data = False
        self.old_magnet_reading = self.cur_magnet_reading    

if __name__ == '__main__':
    rospy.init_node('wheel_encoder_node', anonymous=True)
    Encoder()