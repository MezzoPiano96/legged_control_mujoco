/*******************************************************************************
 * BSD 3-Clause License
 * MuJoCo hardware interface for legged robots
 *******************************************************************************/

#pragma once

#include <deque>
#include <unordered_map>
#include <memory>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <legged_common/hardware_interface/ContactSensorInterface.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>

#include <atomic>

#include <mujoco/mujoco.h>

namespace legged {

struct HybridJointData {
  hardware_interface::JointHandle joint_;
  double posDes_{}, velDes_{}, kp_{}, kd_{}, ff_{};
};

struct HybridJointCommand {
  ros::Time stamp_;
  double posDes_{}, velDes_{}, kp_{}, kd_{}, ff_{};
};

struct ImuData {
  int bodyId_{-1};  // MuJoCo body ID the IMU is attached to (resolved once in setupImu)
  double ori_[4];            // quaternion [x, y, z, w]
  double oriCov_[9];         // orientation covariance
  double angularVel_[3];     // angular velocity
  double angularVelCov_[9];  // angular velocity covariance
  double linearAcc_[3];      // linear acceleration
  double linearAccCov_[9];   // linear acceleration covariance
  // cacc is not reliably populated without an explicit MuJoCo <sensor>, so
  // linear acceleration is obtained by finite-differencing world-frame
  // velocity instead (see LeggedHWMujoco::readImu).
  double linVelWorldPrev_[3]{0., 0., 0.};
  bool hasPrevVel_{false};
};

struct ContactData {
  std::vector<int> geomIds_;  // MuJoCo geom IDs for foot
  bool inContact_{false};
};

class LeggedHWMujoco : public hardware_interface::RobotHW {
 public:
  LeggedHWMujoco();
  ~LeggedHWMujoco() override;

  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override;

 private:
  bool loadModel(const std::string& model_path);
  void setupJoints(ros::NodeHandle& robot_hw_nh);
  void setupImu(ros::NodeHandle& robot_hw_nh);
  void setupContacts(ros::NodeHandle& robot_hw_nh);
  
  void readJoints();
  void readImu(double dt);
  void readContacts();
  // Gazebo provides true base pose/twist via libgazebo_ros_p3d.so on
  // /ground_truth/state (world frame, bodyName "base"); LeggedCheaterController
  // needs that topic. MuJoCo has no such plugin, so publish the equivalent
  // ourselves straight from the simulated free joint - same topic, same frame.
  void publishGroundTruth(const ros::Time& time);

  void writeJoints(const ros::Time& time, const ros::Duration& period);
  // Resets the *physical* simulation back to the "home" keyframe. Restarting
  // a ros_control controller only reinitializes its policy/estimator - it
  // never touches MuJoCo state, so after a fall the robot stays fallen until
  // something calls this (the viewer's own GUI reset button only resets the
  // viewer's separate, disconnected mjData copy - it does nothing here).
  bool resetCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  // MuJoCo objects
  mjModel* model_{nullptr};
  mjData* data_{nullptr};

  // Hardware interfaces
  HybridJointInterface hybridJointInterface_;
  hardware_interface::ImuSensorInterface imuSensorInterface_;
  ContactSensorInterface contactSensorInterface_;
  
  hardware_interface::JointStateInterface jointStateInterface_;
  hardware_interface::EffortJointInterface effortJointInterface_;

  // Joint data
  std::vector<std::string> jointNames_;
  std::vector<int> jointIds_;     // MuJoCo joint IDs
  std::vector<int> actuatorIds_;  // MuJoCo actuator IDs (NOT the same order as jointIds_!)
  std::vector<double> jointPositions_;
  std::vector<double> jointVelocities_;
  std::vector<double> jointEfforts_;
  std::vector<double> jointEffortCommands_;
  // "home" keyframe position per joint (jointIds_ order). Used as a low-level
  // PD hold target whenever no controller has set a (kp,kd) yet - e.g. right
  // after init or after /mujoco_hw/reset - so the robot doesn't immediately
  // collapse under gravity while waiting for the real controller to engage.
  std::vector<double> jointHomePos_;

  std::list<HybridJointData> hybridJointDatas_;
  std::unordered_map<std::string, std::deque<HybridJointCommand>> cmdBuffer_;

  // IMU data
  std::list<ImuData> imuDatas_;

  // Contact data
  std::unordered_map<std::string, ContactData> contactDatas_;

  // Parameters
  double delay_{0.0};
  double controlFreq_{1000.0};  // Hz

  int trunkBodyId_{-1};
  ros::Publisher groundTruthPub_;
  ros::ServiceServer resetService_;
  // Set by resetCallback() (runs on an AsyncSpinner thread), consumed by
  // read() (runs on the main control-loop thread). The actual mj_reset* call
  // must never run concurrently with the main loop's mj_step - both threads
  // touch the same mjData with no other locking.
  std::atomic<bool> resetRequested_{false};
  void applyResetIfRequested();
};

}  // namespace legged
