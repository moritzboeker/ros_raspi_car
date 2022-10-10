#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Int32MultiArray
from ackermann_msgs.msg import AckermannDriveStamped


def ackermann_callback(data):
  global pub
  global pwm_center_speed
  global pwm_center_steering
  global pwm_max_forward
  global pwm_min_backward
  global pwm_max_left
  global pwm_min_right
  global max_steering_angle
  global max_speed

  current_steering_angle = data.drive.steering_angle
  current_speed = data.drive.speed

  pwm_speed = int(pwm_center_speed + (pwm_max_forward - pwm_min_backward) / 2 * current_speed / max_speed)
  pwm_steering = int(pwm_center_steering + (pwm_max_left - pwm_min_right) / 2 * current_steering_angle / max_steering_angle)

  msg = Int32MultiArray()
  msg.data = [pwm_speed, pwm_steering, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1] 
  pub.publish(msg)

if __name__ == '__main__': 
  try:    
    rospy.init_node('ackermann_drive_to_pwm')
    
    pwm_center_speed = rospy.get_param('~pwm_center_speed', 5000)
    pwm_center_steering = rospy.get_param('~pwm_center_steering', 6500)
    pwm_max_forward = rospy.get_param('~pwm_max_forward', 5000)
    pwm_min_backward = rospy.get_param('~pwm_min_backward', 3500)
    pwm_max_left = rospy.get_param('~pwm_max_left', 6500)
    pwm_min_right = rospy.get_param('~pwm_min_right', 3500)
    max_steering_angle = rospy.get_param('~max_steering_angle', np.radians(60.0))
    max_speed = rospy.get_param('~max_speed', 1.0)
    ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/ackermann_cmd')        
    intarr_pwm_topic = rospy.get_param('~intarr_pwm_topic', '/command')     
    rospy.Subscriber(ackermann_cmd_topic, AckermannDriveStamped, ackermann_callback, queue_size=1)
    pub = rospy.Publisher(intarr_pwm_topic, Int32MultiArray, queue_size=1)
    
    rospy.loginfo("Node 'ackermann_drive_to_pwm' started.\nListening to %s, publishing to %s.", ackermann_cmd_topic, intarr_pwm_topic)    
    rospy.spin()    
  except rospy.ROSInterruptException:
    pass