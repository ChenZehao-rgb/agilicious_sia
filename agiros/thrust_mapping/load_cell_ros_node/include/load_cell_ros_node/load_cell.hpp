/*
 * load_cell.h
 *
 *  Created on: Jul 25, 2013
 *      Author: ffontana
 */

#ifndef LOAD_CELL_H_
#define LOAD_CELL_H_

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>

#include "boost/thread.hpp"
#include "fcntl.h"  // F_SETFL O_NONBLOCK fcntl
#include "geometry_msgs/WrenchStamped.h"
#include "ros/ros.h"
#include "std_msgs/Empty.h"

namespace load_cell {

namespace commands {
const unsigned short STOP_STREAMING = 0x000;
const unsigned short START_HIGH_SPEED_REAL_TIME_STREAMING = 0x0002;
const unsigned short START_HIGH_SPEED_BUFFERED_STREAMING = 0x0003;
const unsigned short START_MULTI_UNIT_STREAMING = 0x0004;
const unsigned short RESET_THRESHOLD_LATCH = 0x0041;
const unsigned short SET_SOFTWARE_BIAS = 0x0042;
}  // namespace commands

class LoadCell {
 public:
  LoadCell();
  virtual ~LoadCell();

  bool initUdpConnection();

  void sendCommand(const unsigned short command);

  void startStreamingCallback(std_msgs::Empty);
  void stopStreamingCallback(std_msgs::Empty);
  void setBiasCallback(std_msgs::Empty);

  void receiveLoadCellDataThread();
  void loadParameters();

  geometry_msgs::WrenchStamped convertRawDataToRosMsg(unsigned char* data_raw);

  ros::NodeHandle nh_;
  ros::Publisher data_pub_;
  ros::Subscriber start_streaming_sub_;
  ros::Subscriber stop_streaming_sub_;
  ros::Subscriber set_bias_sub_;

  int socket_handle_;

  boost::thread data_thread_;
  std::string node_name_;

  bool streaming_is_on_;
  std::string load_cell_ip_addr_;
  double counts_per_force_;
  double counts_per_torque_;
};

}  // namespace load_cell

#endif /* LOAD_CELL_H_ */
