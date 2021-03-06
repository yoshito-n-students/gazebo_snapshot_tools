# gazebo_snapshot_tools

creates and restores a snapshot bag of link states from/to Gazebo simulator

## create_snapshot

### Sequence

<img src="https://github.com/yoshito-n-students/gazebo_snapshot_tools/raw/images/images/create_sequence.png" alt="create sequence" width="400" >

### Parameters
___~snapshot_bag___ (string, default: "snapshot.bag")
* path to snapshot bag to write

___~timeout___ (double, default: 10.0)
* creation timeout in sec

## restore_snapshot

### Sequence

<img src="https://github.com/yoshito-n-students/gazebo_snapshot_tools/raw/images/images/restore_sequence.png" alt="restore sequence" width="400" >

### Parameters
___~snapshot_bag___ (string, default: "snapshot.bag")
* path to snapshot bag to read

___~timeout___ (double, default: 10.0)
* restoration timeout in sec

## create_snapshot_by_joy
Sequence and parameters are same as create_snapshot

### Additional subscribed topics
___joy___ (sensor_msgs/Joy)

### Additional parameters
___~button___ (int, default: 2)
* button id to trigger creation
* defaults to circle button of PS4 joystick

## restore_snapshot_by_joy
Sequence and parameters are same as restore_snapshot

### Additional subscribed topics
___joy___ (sensor_msgs/Joy)

### Additional parameters
___~button___ (int, default: 1)
* button id to trigger restoration
* defaults to cross button of PS4 joystick
