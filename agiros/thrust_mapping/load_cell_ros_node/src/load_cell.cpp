#include "load_cell_ros_node/load_cell.hpp"

namespace load_cell {

LoadCell::LoadCell() : nh_(), streaming_is_on_(false) {
  node_name_ = ros::this_node::getName();
  loadParameters();
  initUdpConnection();

  data_pub_ =
    nh_.advertise<geometry_msgs::WrenchStamped>(node_name_ + "/data", 10);
  start_streaming_sub_ = nh_.subscribe(node_name_ + "/start_streaming", 1,
                                       &LoadCell::startStreamingCallback, this);
  stop_streaming_sub_ = nh_.subscribe(node_name_ + "/stop_streaming", 1,
                                      &LoadCell::stopStreamingCallback, this);
  set_bias_sub_ = nh_.subscribe(node_name_ + "/set_bias", 1,
                                &LoadCell::setBiasCallback, this);

  data_thread_ = boost::thread(&LoadCell::receiveLoadCellDataThread, this);
}

LoadCell::~LoadCell() { close(socket_handle_); }

void LoadCell::startStreamingCallback(std_msgs::Empty) {
  streaming_is_on_ = true;
  ROS_INFO("[%s] start streaming", node_name_.c_str());
  sendCommand(commands::START_HIGH_SPEED_REAL_TIME_STREAMING);
}

void LoadCell::stopStreamingCallback(std_msgs::Empty) {
  streaming_is_on_ = false;
  ROS_INFO("[%s] stop streaming", node_name_.c_str());
  sendCommand(commands::STOP_STREAMING);
}

void LoadCell::setBiasCallback(std_msgs::Empty) {
  ROS_INFO("[%s] loadcell bias is set", node_name_.c_str());
  sendCommand(commands::SET_SOFTWARE_BIAS);
}

bool LoadCell::initUdpConnection() {
  struct addrinfo hints, *res;

  // retrieve ip address
  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_DGRAM;
  getaddrinfo(load_cell_ip_addr_.c_str(), "49152", &hints, &res);

  // make a socket:
  socket_handle_ = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
  fcntl(socket_handle_, F_SETFL, O_NONBLOCK);

  // connect
  if (connect(socket_handle_, res->ai_addr, res->ai_addrlen) == -1) {
    ROS_ERROR("[%s] Could not connect to load cell with ip addr %s ",
              node_name_.c_str(), load_cell_ip_addr_.c_str());
    return false;
  }
  ROS_INFO("[%s] Connected to load cell with ip addr %s ", node_name_.c_str(),
           load_cell_ip_addr_.c_str());

  return true;
}

void LoadCell::sendCommand(const unsigned short command) {
  unsigned short header = 0x1234;
  unsigned short cmd;
  unsigned short sample_cnt = 0;

  unsigned char command_raw[8];

  *(unsigned short *)&command_raw[0] = htons(header);
  *(unsigned short *)&command_raw[2] = htons(command);
  *(unsigned int *)&command_raw[4] = htonl(sample_cnt);

  send(socket_handle_, (const char *)command_raw, 8, 0);
}

void LoadCell::receiveLoadCellDataThread() {
  while (ros::ok()) {
    unsigned char data_raw[36];
    int bytes_received = recv(socket_handle_, (char *)data_raw, 36, 0);
    if (bytes_received == 36) {
      geometry_msgs::WrenchStamped ros_msg;
      ros_msg = convertRawDataToRosMsg(data_raw);
      data_pub_.publish(ros_msg);
    } else {
      ros::Duration(0.00001).sleep();
    }
    // only sleep when we are not streaming
    if (!streaming_is_on_) {
      ros::Duration(0.001).sleep();
    }
  }
}

geometry_msgs::WrenchStamped LoadCell::convertRawDataToRosMsg(
  unsigned char *data_raw) {
  geometry_msgs::WrenchStamped ros_msg;

  ros_msg.header.stamp = ros::Time::now();
  ros_msg.header.seq = ntohl(*(unsigned int *)&data_raw[0]);

  //  resp.rdt_sequence = ntohl(*(unsigned int*)&data_raw[0]);
  //  resp.ft_sequence = ntohl(*(unsigned int*)&data_raw[4]);
  //  resp.status = ntohl(*(unsigned int*)&data_raw[8]);

  ros_msg.wrench.force.x =
    ((int)ntohl(*(int *)&data_raw[12])) / counts_per_force_;
  ros_msg.wrench.force.y =
    ((int)ntohl(*(int *)&data_raw[16])) / counts_per_force_;
  ros_msg.wrench.force.z =
    ((int)ntohl(*(int *)&data_raw[20])) / counts_per_force_;

  ros_msg.wrench.torque.x =
    ((int)ntohl(*(int *)&data_raw[24])) / counts_per_torque_;
  ros_msg.wrench.torque.y =
    ((int)ntohl(*(int *)&data_raw[28])) / counts_per_torque_;
  ros_msg.wrench.torque.z =
    ((int)ntohl(*(int *)&data_raw[32])) / counts_per_torque_;

  return ros_msg;
}

void LoadCell::loadParameters() {
  ros::param::param<std::string>("load_cell_ip_addr", load_cell_ip_addr_,
                                 std::string("192.168.1.1"));
  ros::param::param<double>("counts_per_force", counts_per_force_, 1000000.0);
  ros::param::param<double>("counts_per_torque", counts_per_torque_, 1000000.0);
}

}  // namespace load_cell

int main(int argc, char **argv) {
  ros::init(argc, argv, "load_cell");
  load_cell::LoadCell loadcell;

  ros::spin();
  return 0;
}
