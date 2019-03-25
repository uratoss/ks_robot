#include <fstream>
#include <ros/ros.h>
#include <string>

#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

class kosen_waypoint_manager {
public:
  kosen_waypoint_manager(std::string filename)
      : rate_(50), filename_(filename) {
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/marker_array", 1, true);
    load_waypoint();
    marker_pub_.publish(marker_array_);
  }

  kosen_waypoint_manager() : kosen_waypoint_manager("waypoint_list.txt") {}

  void load_waypoint() {
    std::ifstream waypoint_file(filename_);
    visualization_msgs::Marker marker;
    int id = 0;
    marker.ns = "waypoint_shape";
    marker.lifetime = ros::Duration();
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    while (true) {
      if (waypoint_file.eof()) {
        break;
      }
      waypoint_file >> marker.header.seq >> marker.header.frame_id >>
          marker.pose.position.x >> marker.pose.position.y >>
          marker.pose.position.z >> marker.pose.orientation.x >>
          marker.pose.orientation.y >> marker.pose.orientation.z >>
          marker.pose.orientation.w;
      marker.action = visualization_msgs::Marker::ADD;
      marker.header.stamp = ros::Time::now();
      marker.type = visualization_msgs::Marker::ARROW;
      marker.scale.x = 0.5;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.id = id++;
      marker_array_.markers.push_back(marker);
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.scale.x = 0.25;
      marker.scale.y = 0.25;
      marker.scale.z = 0.25;
      marker.id = id++;
      marker_array_.markers.push_back(marker);
    }
    ROS_INFO_STREAM(marker_array_);
  }

  // void publish_marker_callback(const ros::TimerEvent &) {
  //   ;
  //   ;
  // }
  void run() {
    // ros::Timer frame_timer1 =
    //     nh_.createTimer(ros::Duration(1),
    //                     &kosen_waypoint_manager::publish_marker_callback,
    //                     this);
    while (ros::ok()) {
      ros::spinOnce();
      rate_.sleep();
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Publisher marker_pub_;
  visualization_msgs::MarkerArray marker_array_;
  std::string filename_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "waypoint_manager");
  kosen_waypoint_manager waypoint;

  waypoint.run();
  return 0;
}
