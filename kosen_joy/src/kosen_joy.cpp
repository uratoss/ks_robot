#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

double x_l = 0, y_l = 0;
double x_r = 0, y_r = 0;
void callBack(const sensor_msgs::Joy::ConstPtr &joy) {
  x_l = joy->axes[0] * (-1);
  y_l = joy->axes[1];
  ROS_INFO_STREAM("LEFT  x:" << x_l << " ,y:" << y_l);
  x_r = joy->axes[2] * (-1);
  y_r = joy->axes[5];
  ROS_INFO_STREAM("RIGHT  x:" << x_r << " ,y:" << y_r);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "robocon_joy_publisher");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 50);
  ros::Subscriber sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &callBack);
  ros::Rate loop(50);
  geometry_msgs::Twist twist;
  while (ros::ok()) {
    double v = (y_r + y_l) / 2.0;
    double vth = (y_r - y_l) / (2.0 * 0.17);
    twist.linear.x = v;
    twist.angular.z = vth;
    pub.publish(twist);

    ros::spinOnce();
    loop.sleep();
  }
  return 0;
}
