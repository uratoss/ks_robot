#include <fstream>
#include <ros/ros.h>
#include <string>

#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

class kosen_waypoint_maker {
public:
  kosen_waypoint_maker(std::string filename)
      : rate_(50), id_(0), filename_(filename) {
    std::ofstream waypoint_file(filename_);
    clicked_sub_ = nh_.subscribe("/move_base_simple/goal", 10,
                                 &kosen_waypoint_maker::clicked_callback, this);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/markers", 10);
  }
  kosen_waypoint_maker() : kosen_waypoint_maker("waypoint_list.txt") {}

  void clicked_callback(const geometry_msgs::PoseStamped &point) {
    ROS_INFO_STREAM("header:" << point.header.frame_id);
    ROS_INFO("x:%f ,y:%f ,z:%f", point.pose.position.x, point.pose.position.y,
             point.pose.position.z);
    ROS_INFO("rx:%f ,ry:%f ,rz:%f ,rw:%f", point.pose.orientation.x,
             point.pose.orientation.y, point.pose.orientation.z,
             point.pose.orientation.w);
    make_maker(point);
  }
  void make_maker(const geometry_msgs::PoseStamped &point) {
    std::ofstream waypoint_file(filename_, std::ios::out | std::ios::app);
    visualization_msgs::Marker marker;
    marker.header = point.header;
    marker.ns = "waypoint_shape";
    marker.id = id_++;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();
    marker.scale.x = 0.5;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.pose.position = point.pose.position;
    marker.pose.orientation = point.pose.orientation;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    // marker_array_.markers.push_back(marker);
    // marker_pub_.publish(marker_array_);
    waypoint_file << marker.header.seq << " " << marker.header.frame_id << " "
                  << marker.pose.position.x << " " << marker.pose.position.y
                  << " " << marker.pose.position.z << " "
                  << marker.pose.orientation.x << " "
                  << marker.pose.orientation.y << " "
                  << marker.pose.orientation.z << " "
                  << marker.pose.orientation.w << std::endl;

    ROS_INFO_STREAM("save to " << filename_);
    marker_pub_.publish(marker);
    marker.id = id_++;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker_pub_.publish(marker);
  }

  void run() {
    // ros::Timer frame_timer1 = nh_.createTimer(
    //     ros::Duration(dt_), &kosen_driver::publish_cmd_callback, this);
    while (ros::ok()) {
      ros::spinOnce();
      rate_.sleep();
    }
  }

private:
  // ros
  ros::NodeHandle nh_;
  ros::Rate rate_;
  // clicked_pointのsub
  ros::Subscriber clicked_sub_;
  ros::Publisher marker_pub_;
  visualization_msgs::MarkerArray marker_array_;
  int id_;
  // std::ofstream waypoint_file_;
  std::string filename_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "waypoint_maker");
  kosen_waypoint_maker waypoint;

  waypoint.run();
  return 0;
}
