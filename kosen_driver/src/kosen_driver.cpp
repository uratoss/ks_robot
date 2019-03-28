#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <ros/ros.h>

#include "geometry_msgs/Twist.h"
#include "kosen_msgs/Enc.h"
#include "kosen_msgs/Op.h"

class kosen_driver {
public:
  kosen_driver()
      : rate_(50), d_(0.17), dw_(0.11), Kp_(100), Ki_(150), Kd_(0.1), dt_(0.1),
        duty_l_(0.0), duty_r_(0.0) {

    cmd_sub_ = nh_.subscribe("/cmd_vel", 10, &kosen_driver::cmd_callback, this);
    right_sub_ =
        nh_.subscribe("/enc_right", 10, &kosen_driver::right_callback, this);
    left_sub_ =
        nh_.subscribe("/enc_left", 10, &kosen_driver::left_callback, this);

    right_pub_ = nh_.advertise<kosen_msgs::Op>("/right", 10);
    left_pub_ = nh_.advertise<kosen_msgs::Op>("/left", 10);
    speed_pub_ = nh_.advertise<geometry_msgs::Twist>("/speed", 10);
  }

  // move_baseからの指令値(vx,vy,vth)
  void cmd_callback(const geometry_msgs::Twist &cmd) { cmd_ = cmd; }

  // encoderからの時間
  // 15.0の部分は減速比/2なので(1:30より30/2=15)
  // 0.15はタイヤの半径
  void right_callback(const kosen_msgs::Enc &encr) {
    if (encr.time == 0) {
      speed_r_ = 0.0;
    } else {
      speed_r_ = (1000000.0 / (encr.time * 15.0));
      speed_r_ =
          (speed_r_ / 60) * (boost::math::constants::pi<double>() * dw_ * 2);
      if (!encr.dir) {
        speed_r_ *= -1;
      }
    }
  }
  void left_callback(const kosen_msgs::Enc &encl) {
    if (encl.time == 0) {
      speed_l_ = 0.0;
    } else {
      speed_l_ = (1000000.0 / (encl.time * 15.0));
      speed_l_ =
          (speed_l_ / 60) * (boost::math::constants::pi<double>() * dw_ * 2);
      if (encl.dir) {
        speed_l_ *= -1;
      }
    }
  }
  void publish_speed_callback(const ros::TimerEvent &) {
    v_ = (speed_r_ + speed_l_) / 2.0;
    vth_ = (speed_r_ - speed_l_) / (2.0 * d_);
    speed_.linear.x = v_;
    speed_.angular.z = vth_;
    speed_pub_.publish(speed_);
  }

  void publish_cmd_callback(const ros::TimerEvent &) {
    ROS_INFO_STREAM("========================================================");
    {
      kosen_msgs::Op right;
      double cmd_r = (cmd_.linear.x + d_ * cmd_.angular.z);
      if (cmd_r < 0) {
        cmd_r *= -1;
        right.dir = 0;
      } else {
        right.dir = 1;
      }
      double speed_r = speed_r_;
      if (speed_r < 0) {
        speed_r *= -1;
      }

      double P = cmd_r - speed_r;
      I_r_ += P * dt_;
      double D = (P - pre_P_r_) / dt_;
      pre_P_r_ = P;
      duty_r_ += Kp_ * P + Ki_ * I_r_ + Kd_ * D;
      if (duty_r_ > 255) {
        duty_r_ = 255;
      } else if (duty_r_ < 0) {
        duty_r_ = 0;
      }
      ROS_INFO_STREAM("cmd_r : " << cmd_r << " ,speed_r : " << speed_r_
                                 << " ,duty_r : " << duty_r_);
      right.duty = duty_r_;
      right_pub_.publish(right);
    }
    {
      kosen_msgs::Op left;
      double cmd_l = (cmd_.linear.x - d_ * cmd_.angular.z);
      ROS_INFO_STREAM(
          "-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-");
      if (cmd_l < 0) {
        cmd_l *= -1;
        left.dir = 1;
      } else {
        left.dir = 0;
      }
      double speed_l = speed_l_;
      if (speed_l < 0) {
        speed_l *= -1;
      }
      double P = cmd_l - speed_l;
      I_l_ += P * dt_;
      double D = (P - pre_P_l_) / dt_;
      pre_P_l_ = P;
      duty_l_ += Kp_ * P + Ki_ * I_l_ + Kd_ * D;
      if (duty_l_ > 255) {
        duty_l_ = 255;
      } else if (duty_l_ < 0) {
        duty_l_ = 0;
      }
      ROS_INFO_STREAM("cmd_l : " << cmd_l << " ,speed_l : " << speed_l_
                                 << " ,duty_l : " << duty_l_);
      left.duty = duty_l_;
      left_pub_.publish(left);
    }
    ROS_INFO_STREAM("========================================================");
  }

  void run() {
    ros::Timer frame_timer1 = nh_.createTimer(
        ros::Duration(dt_), &kosen_driver::publish_cmd_callback, this);
    ros::Timer frame_timer2 = nh_.createTimer(
        ros::Duration(0.01), &kosen_driver::publish_speed_callback, this);
    while (ros::ok()) {
      ros::spinOnce();
      rate_.sleep();
    }
  }

private:
  // ros
  ros::NodeHandle nh_;
  ros::Rate rate_;
  // move_baseのsub
  ros::Subscriber cmd_sub_;
  // move_baseからの指令値
  geometry_msgs::Twist cmd_;
  // arduinoのsub
  ros::Subscriber right_sub_;
  ros::Subscriber left_sub_;
  // arduinoへのpub
  ros::Publisher right_pub_;
  ros::Publisher left_pub_;
  // 左右車輪の速度
  // m/s
  double speed_l_, speed_r_;
  // ロボットの速度
  double v_, vth_;
  // ロボットの速度のpub
  ros::Publisher speed_pub_;
  geometry_msgs::Twist speed_;

  const double Kp_, Ki_, Kd_;
  const double dt_;
  double pre_P_r_, pre_P_l_;
  double I_r_, I_l_;
  int duty_r_, duty_l_;

  // 中心からタイヤまでの長さ
  const double d_;
  // タイヤの半径
  const double dw_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "driver");
  kosen_driver driver;

  driver.run();
  return 0;
}
