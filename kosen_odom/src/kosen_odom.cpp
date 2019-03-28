#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

class kosen_odom {
public:
  kosen_odom() : rate_(100), dt_(0.1) {
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 50);
    sub_ = nh_.subscribe("speed", 50, &kosen_odom::velocityCallBack, this);
  }
  void velocityCallBack(const geometry_msgs::Twist &tw) {
    vx_ = tw.linear.x;
    vy_ = tw.linear.y;
    vth_ = tw.angular.z;
  }

  void publish_odom(const ros::TimerEvent &) {
    ros::Time current_time = ros::Time::now();

    double delta_x = (vx_ * cos(th_) - vy_ * sin(th_)) * dt_;
    double delta_y = (vx_ * sin(th_) + vy_ * cos(th_)) * dt_;
    double delta_th = vth_ * dt_;

    x_ += delta_x;
    y_ += delta_y;
    th_ += delta_th;

    ROS_INFO_STREAM("Arduino speed: vx: " << vx_ << " ,vy: " << vy_
                                          << " ,vth: " << vth_);
    ROS_INFO_STREAM("Arduino  odom: x: " << x_ << " ,y: " << y_
                                         << " ,th: " << th_);

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster_.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx_;
    odom.twist.twist.linear.y = vy_;
    odom.twist.twist.angular.z = vth_;

    odom_pub_.publish(odom);
  }

  void run() {
    ros::Timer frame_timer =
        nh_.createTimer(ros::Duration(dt_), &kosen_odom::publish_odom, this);
    while (ros::ok()) {
      ros::spinOnce();
      rate_.sleep();
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Publisher odom_pub_;
  tf::TransformBroadcaster odom_broadcaster_;
  ros::Subscriber sub_;
  double vx_, vy_, vth_;
  double x_, y_, th_;
  const double dt_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry_publisher");
  kosen_odom odom;

  odom.run();
  return 0;
}
