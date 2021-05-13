#ifndef GAZEBO_SNAPSHOT_TOOLS_HPP
#define GAZEBO_SNAPSHOT_TOOLS_HPP

#include <algorithm>
#include <functional>
#include <stdexcept>
#include <string>

#include <gazebo_msgs/GetPhysicsProperties.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/SetLinkState.h>
#include <ros/callback_queue.h>
#include <ros/duration.h>
#include <ros/message_event.h>
#include <ros/message_traits.h>
#include <ros/node_handle.h>
#include <ros/service.h>
#include <ros/service_client.h>
#include <ros/time.h>
#include <rosbag/bag.h>
#include <rosbag/message_instance.h>
#include <rosbag/query.h>
#include <rosbag/view.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>

namespace gazebo_snapshot_tools {

namespace internal {

/////////////////
// Service utils
/////////////////

template < typename > struct ServiceOf;
template < typename > struct ResponseOf;
#define GST_DEFINE_REQUEST_TRAITS(Service)                                                         \
  template <> struct ServiceOf< Service::Request > { using Type = Service; };                      \
  template <> struct ResponseOf< Service::Request > { using Type = Service::Response; }
GST_DEFINE_REQUEST_TRAITS(gazebo_msgs::GetPhysicsProperties);
GST_DEFINE_REQUEST_TRAITS(gazebo_msgs::SetLinkState);
GST_DEFINE_REQUEST_TRAITS(std_srvs::Empty);

// creates a service client ready to be called within the given timeout
// (i.e. the existence of the service has been confirmed)
template < typename Service >
static inline ros::ServiceClient createCallableClient(const std::string &service_name,
                                                      const ros::WallTime &abs_timeout) {
  ros::ServiceClient client =
      ros::service::createClient< Service >(service_name, /* persistent = */ true);
  const ros::WallDuration timeout = abs_timeout - ros::WallTime::now();
  if (!client.waitForExistence(ros::Duration(timeout.sec, timeout.nsec))) {
    throw std::runtime_error("createCallableClient(): " + client.getService() + " does not exist");
  }
  return client;
}

// calls the service associated to the given client with the given request within the given timeout
template < typename Request >
static inline typename ResponseOf< Request >::Type
call(ros::ServiceClient *const client, const Request &request, const ros::WallTime &abs_timeout) {
  if (ros::WallTime::now() > abs_timeout) {
    throw std::runtime_error("call(): Reached timeout");
  }
  typename ResponseOf< Request >::Type response;
  if (!client->call(request, response)) {
    throw std::runtime_error("call(): Failed to call " + client->getService());
  }
  return response;
}

// calls the given service with the given request within the given timeout
template < typename Request >
static inline typename ResponseOf< Request >::Type
call(const std::string &service_name, const Request &request, const ros::WallTime &abs_timeout) {
  ros::ServiceClient client =
      createCallableClient< typename ServiceOf< Request >::Type >(service_name, abs_timeout);
  return call(&client, request, abs_timeout);
}

///////////////
// Topic utils
///////////////

// receives the first message via the given topic within the given timeout
template < typename Message >
static inline typename ros::MessageEvent< const Message >
receive(const std::string &topic_name, const ros::WallTime &abs_timeout) {
  ros::NodeHandle nh;
  ros::CallbackQueue cb_queue;
  nh.setCallbackQueue(&cb_queue);

  using Event = typename ros::MessageEvent< const Message >;
  Event event;
  void (Event::*const copy_event_f)(const Event &) = &Event::operator=;
  ros::Subscriber sub = nh.subscribe(topic_name, 1, copy_event_f, &event);

  while (true) {
    cb_queue.callOne(abs_timeout - ros::WallTime::now());
    if (event.getMessage() && event.getReceiptTime() > ros::TIME_MIN) {
      return event;
    }
    if (ros::WallTime::now() >= abs_timeout) {
      throw std::runtime_error("receive(): no valid message via " + sub.getTopic());
    }
  }
}

/////////////
// Bag utils
/////////////

// writes the given message event to the given bagfile
template < typename Message >
static inline void write(const std::string &bag_path,
                         const typename ros::MessageEvent< Message > &event) {
  rosbag::Bag bag(bag_path, rosbag::bagmode::Write);
  if (!bag.isOpen()) {
    throw std::runtime_error("write(): cannot open a bag at " + bag_path);
  }

  const ros::M_string &header = event.getConnectionHeader();
  const ros::M_string::const_iterator topic_name = header.find("topic");
  bag.write(topic_name != header.end() ? topic_name->second : "", event);
}

// reads the first message event from the given bagfile
template < typename Message >
static inline typename ros::MessageEvent< Message > read(const std::string &bag_path) {
  rosbag::Bag bag(bag_path, rosbag::bagmode::Read);
  if (!bag.isOpen()) {
    throw std::runtime_error("read(): cannot open a bag at " + bag_path);
  }

  const std::string datatype = ros::message_traits::datatype< Message >();
  rosbag::View view(bag, rosbag::TypeQuery(datatype));
  if (view.size() <= 0) {
    throw std::runtime_error("read(): no " + datatype + " in " + bag_path);
  }

  const rosbag::MessageInstance first = *view.begin();
  return ros::MessageEvent< Message >(first.instantiate< Message >(), first.getConnectionHeader(),
                                      first.getTime());
}

} // namespace internal

// receives the latest link states and write it to the given bagfile
static inline void createSnapshot(const std::string &bag_path, const ros::WallDuration &timeout) {
  // use WallTime rather than Time because Time may not be advancing if gazebo is paused
  const ros::WallTime abs_timeout = ros::WallTime::now() + timeout;

  bool should_unpause_pause;
  {
    const gazebo_msgs::GetPhysicsProperties::Response response =
        internal::call("/gazebo/get_physics_properties",
                       gazebo_msgs::GetPhysicsProperties::Request(), abs_timeout);
    if (!response.success) {
      throw std::runtime_error("createSnapshot(): Failed to get physics properties: " +
                               response.status_message);
    }
    should_unpause_pause = response.pause;
  }

  if (should_unpause_pause) {
    internal::call("/gazebo/unpause_physics", std_srvs::Empty::Request(), abs_timeout);
  }

  const ros::MessageEvent< gazebo_msgs::LinkStates > link_states =
      internal::receive< gazebo_msgs::LinkStates >("/gazebo/link_states", abs_timeout);

  internal::write(bag_path, link_states);

  if (should_unpause_pause) {
    internal::call("/gazebo/pause_physics", std_srvs::Empty::Request(), abs_timeout);
  }
}

// reads link states from the given bagfile and set it to gazebo
static inline void restoreSnapshot(const std::string &bag_path, const ros::WallDuration &timeout) {
  const ros::WallTime abs_timeout = ros::WallTime::now() + timeout;

  bool should_pause_unpause;
  {
    const gazebo_msgs::GetPhysicsProperties::Response response =
        internal::call("/gazebo/get_physics_properties",
                       gazebo_msgs::GetPhysicsProperties::Request(), abs_timeout);
    if (!response.success) {
      throw std::runtime_error("restoreSnapshot(): Failed to get physics properties: " +
                               response.status_message);
    }
    should_pause_unpause = !response.pause;
  }

  if (should_pause_unpause) {
    internal::call("/gazebo/pause_physics", std_srvs::Empty::Request(), abs_timeout);
  }

  const gazebo_msgs::LinkStatesConstPtr link_states =
      internal::read< gazebo_msgs::LinkStates >(bag_path).getConstMessage();

  {
    ros::ServiceClient client = internal::createCallableClient< gazebo_msgs::SetLinkState >(
        "/gazebo/set_link_state", abs_timeout);
    const int n_links = std::min(std::min(link_states->name.size(), link_states->pose.size()),
                                 link_states->twist.size());
    for (int i = 0; i < n_links; ++i) {
      gazebo_msgs::SetLinkState::Request request;
      request.link_state.link_name = link_states->name[i];
      request.link_state.pose = link_states->pose[i];
      request.link_state.twist = link_states->twist[i];
      internal::call(&client, request, abs_timeout);
    }
  }

  if (should_pause_unpause) {
    internal::call("/gazebo/unpause_physics", std_srvs::Empty::Request(), abs_timeout);
  }
}

// a helper class that dispatches the given callback when the given button is newly pressed
class JoyButtonHandler {
public:
  JoyButtonHandler(const int button_id, const std::function< void() > &callback)
      : button_id_(button_id), callback_(callback), was_pressed_(false) {}

  void handleJoy(const sensor_msgs::JoyConstPtr &joy) {
    const bool is_pressed =
        (button_id_ < joy->buttons.size() ? joy->buttons[button_id_] > 0 : false);

    if (!was_pressed_ && is_pressed) {
      try {
        callback_();
      } catch (const std::exception &error) {
        ROS_ERROR_STREAM(error.what());
      }
    }

    was_pressed_ = is_pressed;
  }

private:
  const int button_id_;
  const std::function< void() > callback_;
  bool was_pressed_;
};
} // namespace gazebo_snapshot_tools

#endif