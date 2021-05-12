#include <string>

#include <gazebo_snapshot_tools/gazebo_snapshot_tools.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/spinner.h>
#include <ros/subscriber.h>

namespace gst = gazebo_snapshot_tools;
namespace rp = ros::param;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "create_snapshot_by_joy");
  ros::NodeHandle nh;

  // load params
  const std::string bag_path = rp::param<std::string>("~snapshot_bag", "snapshot.bag");
  const ros::WallDuration timeout(rp::param("~timeout", 10.));
  const int button_id = rp::param("~button", 2); // circle button of ps4 joystick

  gst::JoyButtonHandler button_handler(button_id, [&bag_path, &timeout]() {
    gst::createSnapshot(bag_path, timeout);
    ROS_INFO_STREAM("Created a snapshot bag at " << bag_path);
  });
  const ros::Subscriber joy_sub =
      nh.subscribe("joy", 1, &gst::JoyButtonHandler::handleJoy, &button_handler);

  ros::spin();

  return 0;
}