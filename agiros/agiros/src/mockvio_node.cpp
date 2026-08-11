#include <ros/ros.h>

#include "agilib/estimator/mock_vio/mock_vio.hpp"
#include "agiros/ros_eigen.hpp"
#include "agiros_msgs/QuadState.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Empty.h"

using namespace agi;

class MockVioNode : protected MockVio {
 public:
  MockVioNode(const ros::NodeHandle &nh = ros::NodeHandle(),
              const ros::NodeHandle &pnh = ros::NodeHandle("~"));

 private:
  static Quadrotor loadQuad(const ros::NodeHandle &nh);
  static std::shared_ptr<MockVioParams> loadParams(const ros::NodeHandle &nh);
  void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
  void imuCallback(const sensor_msgs::ImuConstPtr &msg);
  void resetCallback(const std_msgs::EmptyConstPtr &);


  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber reset_sub_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber imu_sub_;

  ros::Publisher odometry_pub_;
  ros::Publisher imu_pub_;

  bool velocity_in_bodyframe_{false};
};

MockVioNode::MockVioNode(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
  : MockVio(loadQuad(pnh), loadParams(pnh)), nh_(nh), pnh_(pnh) {
  odometry_pub_ =
    nh_.advertise<agiros_msgs::QuadState>("mock_vio/odometry_out", 10);
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("mock_vio/imu_out", 10);

  odometry_sub_ =
    nh_.subscribe("mock_vio/odometry_in", 10, &MockVioNode::odometryCallback,
                  this, ros::TransportHints().tcpNoDelay());
  imu_sub_ = nh_.subscribe("mock_vio/imu_in", 10, &MockVioNode::imuCallback,
                           this, ros::TransportHints().tcpNoDelay());

  reset_sub_ = nh_.subscribe("mock_vio/reset", 10, &MockVioNode::resetCallback,
                             this, ros::TransportHints().tcpNoDelay());

  pnh_.getParam("velocity_in_bodyframe", velocity_in_bodyframe_);
}

Quadrotor MockVioNode::loadQuad(const ros::NodeHandle &nh) {
  Quadrotor quad;

  std::string quadrotor_file;
  if (!nh.getParam("quadrotor_file", quadrotor_file)) {
    ROS_FATAL("Could not get quadrotor file name.");
    ros::shutdown();
    return quad;
  }

  if (!quad.load(quadrotor_file)) {
    ROS_FATAL("Could not load quadrotor.");
    ros::shutdown();
    return quad;
  }

  return quad;
}

std::shared_ptr<MockVioParams> MockVioNode::loadParams(
  const ros::NodeHandle &nh) {
  std::shared_ptr<MockVioParams> params = std::make_shared<MockVioParams>();

  std::string params_file;
  if (!nh.getParam("params_file", params_file)) {
    ROS_FATAL("Could not get parameter file name.");
    ros::shutdown();
    return params;
  }

  if (!params->load(params_file)) {
    ROS_FATAL("Could not load parameters.");
    ros::shutdown();
    return params;
  }

  return params;
}

void MockVioNode::odometryCallback(const nav_msgs::OdometryConstPtr &msg) {
  QuadState state;
  state.setZero();
  state.t = msg->header.stamp.toSec();
  state.p = fromRosVec3(msg->pose.pose.position);
  state.q(Quaternion(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                     msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z));
  state.v = fromRosVec3(msg->twist.twist.linear);
  state.w = fromRosVec3(msg->twist.twist.angular);
  if (velocity_in_bodyframe_) state.v = state.R() * state.v;

  this->addState(state);

  QuadState mock_state;
  mock_state.setZero();
  if (!this->getAt(state.t, &mock_state)) {
    logger_.warn("Could not get state at %1.6g\n", state.t);
    return;
  }

  agiros_msgs::QuadState mock_msg;
  mock_msg.t = state.t;
  mock_msg.header.frame_id = "world";
  mock_msg.header.stamp = msg->header.stamp;
  mock_msg.pose.position = toRosPoint(mock_state.p);
  mock_msg.pose.orientation = toRosQuaternion(mock_state.q());
  mock_msg.velocity.linear = toRosVector(mock_state.v);
  mock_msg.velocity.angular = toRosVector(mock_state.w);
  mock_msg.acceleration.linear = toRosVector(mock_state.a);
  mock_msg.acceleration.angular = toRosVector(mock_state.tau);
  mock_msg.acc_bias = toRosVector(mock_state.ba);
  mock_msg.gyr_bias = toRosVector(mock_state.bw);
  odometry_pub_.publish(mock_msg);
}

void MockVioNode::imuCallback(const sensor_msgs::ImuConstPtr &msg) {
  this->addImu(ImuSample(msg->header.stamp.toSec(),
                         fromRosVec3(msg->linear_acceleration),
                         fromRosVec3(msg->angular_velocity)));

  sensor_msgs::Imu mock_msg;
  mock_msg.header.stamp = msg->header.stamp;
  mock_msg.linear_acceleration = toRosVector(this->last_imu_sample_.acc);
  mock_msg.angular_velocity = toRosVector(this->last_imu_sample_.omega);
  imu_pub_.publish(mock_msg);
}

void MockVioNode::resetCallback(const std_msgs::EmptyConstPtr &) { reset(); }

int main(int argc, char **argv) {
  ros::init(argc, argv, "mockvio_node");

  MockVioNode mock_vio;
  ros::spin();

  return 0;
}
