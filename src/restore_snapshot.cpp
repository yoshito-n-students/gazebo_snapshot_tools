#include <stdexcept>
#include <string>

#include <gazebo_snapshot_tools/gazebo_snapshot_tools.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "restore_snapshot");
  ros::NodeHandle nh;

  const std::string bag_path = ros::param::param< std::string >("~snapshot_bag", "snapshot.bag");
  const ros::WallDuration timeout(ros::param::param("~timeout", 10.));

  try {
    gazebo_snapshot_tools::restoreSnapshot(bag_path, timeout);
    ROS_INFO_STREAM("Restored a snapshot at " << bag_path);
  } catch (const std::runtime_error &error) {
    ROS_ERROR_STREAM(error.what());
    return 1;
  }

  return 0;
}