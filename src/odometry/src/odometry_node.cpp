#include <iostream>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32MultiArray.h>

int steering_idx;
int steering_pwm_center;
int steering_max_ampl_pwm;
double steering_max_ampl_angle;
double wheelbase;

double fwrd_speed = 0.0;
double steering_angle = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

void fwrd_speed_callback(const std_msgs::Float64::ConstPtr& msg){
    fwrd_speed = msg->data;
}

void pwm_callback(const std_msgs::Int32MultiArray::ConstPtr& msg){
    double rad_per_pwm = (double) steering_max_ampl_angle / steering_max_ampl_pwm;
    double steering_pwm = msg->data[steering_idx];
    if (steering_pwm > steering_pwm_center + steering_max_ampl_pwm){
      steering_pwm = steering_pwm_center + steering_max_ampl_pwm;
    }
    else if (steering_pwm < steering_pwm_center - steering_max_ampl_pwm){
      steering_pwm = steering_pwm_center - steering_max_ampl_pwm;
    }
    steering_angle = (steering_pwm - steering_pwm_center) * rad_per_pwm;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber fwrd_speed_sub = n.subscribe("encoder_speed", 100, fwrd_speed_callback);
  ros::Subscriber pwm_sub = n.subscribe("command", 100, pwm_callback);
  tf::TransformBroadcaster odom_broadcaster;

  n.param("steering_pwm_index", steering_idx, 1);
  n.param("steering_pwm_center", steering_pwm_center, 5000);
  n.param("steering_max_ampl_pwm", steering_max_ampl_pwm, 1500);
  n.param("steering_max_ampl_angle", steering_max_ampl_angle, 40.0 * M_PI / 180.0);
  n.param("wheelbase", wheelbase, 0.34);

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(50.0);
  while(n.ok()){
    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    vx = fwrd_speed * cos(steering_angle);
    vy = fwrd_speed * sin(steering_angle);
    vth = fwrd_speed / wheelbase * tan(steering_angle);

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}