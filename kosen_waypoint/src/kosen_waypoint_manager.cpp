#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <fstream>
#include <string>

#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

class kosen_waypoint_manager {
 public:
  using MoveBaseClient =
      actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

  kosen_waypoint_manager(std::string filename)
      : rate_(50), filename_(filename), action_("move_base", true) {
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
      // marker.type = visualization_msgs::Marker::CYLINDER;
      // marker.scale.x = 0.25;
      // marker.scale.y = 0.25;
      // marker.scale.z = 0.25;
      // marker.id = id++;
      // marker_array_.markers.push_back(marker);
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.text = std::to_string(id / 3);
      marker.id = id++;
      marker_array_.markers.push_back(marker);
    }
    // ROS_INFO_STREAM(marker_array_);
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
    while (!action_.waitForServer(ros::Duration(5.0))) {
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal goal;
    // goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.frame_id = "map";
    for (auto it = marker_array_.markers.begin();
         it != marker_array_.markers.end(); it += 2) {
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose = it->pose;

      ROS_INFO_STREAM("Sending goal" << goal.target_pose.pose);
      ROS_INFO_STREAM(*it);

      action_.sendGoal(goal);

      action_.waitForResult();

      if (action_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved 1 meter forward");
      } else {
        ROS_INFO("The base failed to move forward 1 meter for some reason");
      }
      it->action = visualization_msgs::Marker::DELETE;
      (it + 1)->action = visualization_msgs::Marker::DELETE;
      //(it + 2)->action = visualization_msgs::Marker::DELETE;
      marker_pub_.publish(marker_array_);
    }
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
  MoveBaseClient action_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "waypoint_manager");
  kosen_waypoint_manager waypoint;

  waypoint.run();
  return 0;
}
