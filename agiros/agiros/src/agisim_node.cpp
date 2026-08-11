#include <ros/ros.h>

#include "agiros/agisim.hpp"
#include "agiros_msgs/QuadState.h"
#include "nav_msgs/Odometry.h"
#include "rosgraph_msgs/Clock.h"

using namespace agi;

static inline SimulatorParams loadParams(const ros::NodeHandle &nh) {
  std::string simulator_config;
  const bool got_config = nh.getParam("simulator_config", simulator_config);

  std::string agi_param_dir;
  const bool got_directory = nh.getParam("agi_param_dir", agi_param_dir);

  std::string ros_param_dir;
  nh.getParam("ros_param_dir", ros_param_dir);

  ROS_WARN_STREAM("Simulator Config:        " << simulator_config);
  ROS_WARN_STREAM("Agi Param Directory: " << agi_param_dir);
  ROS_WARN_STREAM("ROS Param Directory: " << ros_param_dir);

  if (!got_config) ROS_FATAL("No parameter directory given!");
  if (!got_directory) ROS_FATAL("No simulator config file given!");
  if (!got_config || !got_directory) ros::shutdown();
  ROS_INFO("Loading Simulator Params from %s in %s", simulator_config.c_str(),
           ros_param_dir.c_str());

  return SimulatorParams(fs::path(ros_param_dir) / fs::path(simulator_config),
                         agi_param_dir, ros_param_dir);
}

AgiSim::AgiSim(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
  : nh_(nh), pnh_(pnh), sim_params_(loadParams(pnh_)), simulator_(sim_params_) {
  // Logic subscribers
  reset_sub_ = pnh_.subscribe("reset_sim", 1, &AgiSim::resetCallback, this);

  // Publishers
  odometry_publisher_ =
    pnh_.advertise<nav_msgs::Odometry>("groundtruth/odometry", 1);
  state_publisher_ =
    pnh_.advertise<agiros_msgs::QuadState>("groundtruth/state", 1);
  clock_publisher_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1);

  pnh_.getParam("real_time_factor", real_time_factor_);

  QuadState quad_state;
  sim_thread_ = std::thread(&AgiSim::simLoop, this);
}

AgiSim::~AgiSim() {
  if (sim_thread_.joinable()) sim_thread_.join();
}

void AgiSim::resetCallback(const std_msgs::BoolConstPtr &msg) {
  ROS_INFO("Resetting simulator!");
  simulator_.setCommand(Command(ros::Time::now().toSec(), Vector<4>::Zero()));
  simulator_.reset(msg->data);
}

void AgiSim::simLoop() {
  while (ros::ok()) {
    ros::WallTime t_start_sim = ros::WallTime::now();
    QuadState quad_state;
    simulator_.getState(&quad_state);
    rosgraph_msgs::Clock curr_time;
    curr_time.clock.fromSec(quad_state.t);
    clock_publisher_.publish(curr_time);
    ros_pilot_.getPilot().odometryCallback(quad_state);
    ros_pilot_.getPilot().guardOdometryCallback(quad_state);
    ros_pilot_.getPilot().motorSpeedCallback(quad_state.mot);
    const Command cmd = ros_pilot_.getCommand();
    if (cmd.valid()) {
      simulator_.setCommand(cmd);
    } else {
      Command zero_cmd;
      zero_cmd.t = quad_state.t;
      zero_cmd.thrusts.setZero();
      simulator_.setCommand(zero_cmd);
    }
    if (!simulator_.run(sim_dt_)) ROS_WARN_THROTTLE(1.0, "Simulation failed!");

    // handle other pending callbacks and publishing tasks
    ros::spinOnce();

    Scalar sleep_time = 1.0 / real_time_factor_ * sim_dt_ -
                        (ros::WallTime::now() - t_start_sim).toSec();
    ros::WallDuration(std::max(sleep_time, 0.0)).sleep();
  }
}

void AgiSim::publishStates(const QuadState &state) {
  agiros_msgs::QuadState msg;
  msg.t = state.t;
  msg.pose.position = toRosPoint(state.p);
  msg.pose.orientation = toRosQuaternion(state.q());
  msg.velocity.linear = toRosVector(state.v);
  msg.velocity.angular = toRosVector(state.w);
  msg.acceleration.linear = toRosVector(state.a);
  msg.acceleration.angular = toRosVector(state.tau);

  nav_msgs::Odometry msg_odo;
  msg_odo.header.frame_id = "world";
  msg_odo.header.stamp = ros::Time(state.t);
  msg_odo.pose.pose = msg.pose;
  msg_odo.twist.twist = msg.velocity;

  odometry_publisher_.publish(msg_odo);
  state_publisher_.publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "agisim_node");

  AgiSim agi_sim;
  ros::spin();

  return 0;
}
