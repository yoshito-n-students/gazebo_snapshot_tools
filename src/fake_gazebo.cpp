#include <gazebo_msgs/GetPhysicsProperties.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/SetLinkState.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/service_server.h>
#include <ros/service_traits.h>
#include <ros/spinner.h>
#include <ros/timer.h>
#include <std_srvs/Empty.h>

#include <boost/bind.hpp>

namespace gm = gazebo_msgs;
namespace rst = ros::service_traits;

template < typename Service >
bool handle(typename Service::Request &, typename Service::Response &) {
  ROS_INFO_STREAM("called " << rst::datatype< Service >());
  return true;
}

template <>
bool handle< gm::GetPhysicsProperties >(gm::GetPhysicsProperties::Request &,
                                        gm::GetPhysicsProperties::Response &response) {
  ROS_INFO_STREAM("called " << ros::service_traits::datatype< gm::GetPhysicsProperties >());
  response.pause = true;
  response.success = true;
  return true;
}

template <>
bool handle< gm::SetLinkState >(gm::SetLinkState::Request &, gm::SetLinkState::Response &response) {
  ROS_INFO_STREAM("called " << ros::service_traits::datatype< gm::SetLinkState >());
  response.success = true;
  return true;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "fake_gazebo");
  ros::NodeHandle nh;

  const ros::ServiceServer
      get_physics_properties_c =
          nh.advertiseService("/gazebo/get_physics_properties", handle< gm::GetPhysicsProperties >),
      set_link_state_c = nh.advertiseService("/gazebo/set_link_state", handle< gm::SetLinkState >),
      pause_physics_c = nh.advertiseService("/gazebo/pause_physics", handle< std_srvs::Empty >),
      unpause_physics_c = nh.advertiseService("/gazebo/unpause_physics", handle< std_srvs::Empty >);
  const ros::Publisher link_states_p = nh.advertise< gm::LinkStates >("/gazebo/link_states", 1);
  void (ros::Publisher::*const link_states_cb)(const gm::LinkStates &) const =
      &ros::Publisher::publish< gm::LinkStates >;
  const ros::Timer link_states_t = nh.createTimer(
      ros::Duration(1.), boost::bind(link_states_cb, &link_states_p, gm::LinkStates()));

  ros::spin();

  return 0;
}