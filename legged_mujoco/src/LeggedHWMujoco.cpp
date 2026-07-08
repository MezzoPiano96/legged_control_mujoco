/*******************************************************************************
 * MuJoCo hardware interface implementation
 *******************************************************************************/

#include "legged_mujoco/LeggedHWMujoco.h"
#include <pluginlib/class_list_macros.hpp>

namespace legged {

LeggedHWMujoco::LeggedHWMujoco() = default;

LeggedHWMujoco::~LeggedHWMujoco() {
  if (data_) mj_deleteData(data_);
  if (model_) mj_deleteModel(model_);
}

bool LeggedHWMujoco::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  // Load MuJoCo model
  std::string model_path;
  if (!robot_hw_nh.getParam("mujoco_model_path", model_path)) {
    ROS_ERROR("Failed to get param 'mujoco_model_path'");
    return false;
  }

  if (!loadModel(model_path)) {
    return false;
  }

  // Get parameters
  robot_hw_nh.param("delay", delay_, 0.0);
  robot_hw_nh.param("control_frequency", controlFreq_, 1000.0);

  // Setup interfaces
  setupJoints(robot_hw_nh);
  setupImu(robot_hw_nh);
  setupContacts(robot_hw_nh);

  trunkBodyId_ = mj_name2id(model_, mjOBJ_BODY, "trunk");
  groundTruthPub_ = root_nh.advertise<nav_msgs::Odometry>("/ground_truth/state", 10);
  resetService_ = root_nh.advertiseService("/mujoco_hw/reset", &LeggedHWMujoco::resetCallback, this);

  ROS_INFO("MuJoCo hardware interface initialized successfully");
  return true;
}

bool LeggedHWMujoco::resetCallback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/) {
  resetRequested_ = true;
  return true;
}

void LeggedHWMujoco::applyResetIfRequested() {
  if (!resetRequested_.exchange(false)) {
    return;
  }

  int homeKeyId = mj_name2id(model_, mjOBJ_KEY, "home");
  if (homeKeyId >= 0) {
    mj_resetDataKeyframe(model_, data_, homeKeyId);
  } else {
    mj_resetData(model_, data_);
  }
  mj_forward(model_, data_);

  // write() unconditionally re-applies the last jointEffortCommands_ to
  // data_->ctrl every cycle, controller running or not. Without zeroing
  // both, last cycle's pre-reset (e.g. fall-recovery) torques immediately
  // re-disturb the freshly reset pose on the very next control cycle.
  std::fill(jointEffortCommands_.begin(), jointEffortCommands_.end(), 0.0);
  for (int i = 0; i < model_->nu; ++i) {
    data_->ctrl[i] = 0.0;
  }
  for (auto& entry : cmdBuffer_) {
    entry.second.clear();
  }
  // A controller that was running when it got stopped (e.g. by the safety
  // checker) leaves its last (posDes_, kp_, kd_, ff_) sitting in these
  // handles - they are untouched by stopping a ros_control controller.
  // Zero them too, otherwise writeJoints() sees a nonzero (kp_, kd_) and
  // never falls back to the home-pose hold below.
  for (auto& jointData : hybridJointDatas_) {
    jointData.posDes_ = 0.0;
    jointData.velDes_ = 0.0;
    jointData.kp_ = 0.0;
    jointData.kd_ = 0.0;
    jointData.ff_ = 0.0;
  }

  ROS_INFO("MuJoCo simulation reset to 'home' keyframe via /mujoco_hw/reset");
}

bool LeggedHWMujoco::loadModel(const std::string& model_path) {
  char error[1000];
  model_ = mj_loadXML(model_path.c_str(), nullptr, error, 1000);
  
  if (!model_) {
    ROS_ERROR_STREAM("Failed to load MuJoCo model: " << error);
    return false;
  }

  data_ = mj_makeData(model_);
  if (!data_) {
    ROS_ERROR("Failed to create MuJoCo data");
    return false;
  }

  // Reset to initial state. Prefer the model's "home" keyframe (standing
  // pose, feet on the ground) over mj_resetData's default spawn pose, which
  // for this model is straight-legged and ~0.16m above standing height -
  // that combination causes a violent drop+catch transient the WBC can't
  // recover from.
  int homeKeyId = mj_name2id(model_, mjOBJ_KEY, "home");
  if (homeKeyId >= 0) {
    mj_resetDataKeyframe(model_, data_, homeKeyId);
    ROS_INFO("Reset MuJoCo state to 'home' keyframe");
  } else {
    mj_resetData(model_, data_);
    ROS_WARN("No 'home' keyframe found in MuJoCo model, using default reset");
  }
  mj_forward(model_, data_);

  ROS_INFO_STREAM("Loaded MuJoCo model: " << model_path);
  ROS_INFO_STREAM("  DOF: " << model_->nv);
  ROS_INFO_STREAM("  Actuators: " << model_->nu);
  
  return true;
}

void LeggedHWMujoco::setupJoints(ros::NodeHandle& robot_hw_nh) {
  // Get joint names from parameter server
  XmlRpc::XmlRpcValue joint_list;
  if (!robot_hw_nh.getParam("joints", joint_list)) {
    ROS_ERROR("Failed to get param 'joints'");
    return;
  }

  ROS_ASSERT(joint_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  
  // URDF -> MuJoCo joint name mapping
  std::map<std::string, std::string> joint_name_map;
  joint_name_map["LF_HAA"] = "FL_hip_joint";
  joint_name_map["LF_HFE"] = "FL_thigh_joint";
  joint_name_map["LF_KFE"] = "FL_calf_joint";
  joint_name_map["RF_HAA"] = "FR_hip_joint";
  joint_name_map["RF_HFE"] = "FR_thigh_joint";
  joint_name_map["RF_KFE"] = "FR_calf_joint";
  joint_name_map["LH_HAA"] = "RL_hip_joint";
  joint_name_map["LH_HFE"] = "RL_thigh_joint";
  joint_name_map["LH_KFE"] = "RL_calf_joint";
  joint_name_map["RH_HAA"] = "RR_hip_joint";
  joint_name_map["RH_HFE"] = "RR_thigh_joint";
  joint_name_map["RH_KFE"] = "RR_calf_joint";
  
  int nJoints = joint_list.size();
  jointNames_.resize(nJoints);
  jointIds_.resize(nJoints);
  actuatorIds_.resize(nJoints);
  jointPositions_.resize(nJoints, 0.0);
  jointVelocities_.resize(nJoints, 0.0);
  jointEfforts_.resize(nJoints, 0.0);
  jointEffortCommands_.resize(nJoints, 0.0);
  jointHomePos_.resize(nJoints, 0.0);

  int setupHomeKeyId = mj_name2id(model_, mjOBJ_KEY, "home");

  // Map ROS joint names to MuJoCo joint IDs
  for (int i = 0; i < nJoints; ++i) {
    std::string rosJointName = static_cast<std::string>(joint_list[i]);
    jointNames_[i] = rosJointName;
    
    // Find corresponding MuJoCo name
    std::string mjJointName;
    if (joint_name_map.find(rosJointName) != joint_name_map.end()) {
      mjJointName = joint_name_map[rosJointName];
    } else {
      mjJointName = rosJointName;
    }
    
    int jointId = mj_name2id(model_, mjOBJ_JOINT, mjJointName.c_str());
    if (jointId < 0) {
      ROS_ERROR_STREAM("Joint " << mjJointName << " (ROS: " << rosJointName << ") not found in MuJoCo model");
      continue;
    }
    jointIds_[i] = jointId;
    if (setupHomeKeyId >= 0) {
      int qposAdr = model_->jnt_qposadr[jointId];
      jointHomePos_[i] = model_->key_qpos[setupHomeKeyId * model_->nq + qposAdr];
    }

    // The actuator order in the MJCF does NOT match this ROS joint order
    // (e.g. MJCF declares FR_* before FL_*, but the ROS "joints" param lists
    // LF_* first), so actuator/force arrays must be indexed by a lookup,
    // never by the loop index i. Actuator names don't match joint names
    // (e.g. actuator "FL_hip" drives joint "FL_hip_joint"), so search by
    // the joint id each actuator's transmission targets instead.
    int actuatorId = -1;
    for (int a = 0; a < model_->nu; ++a) {
      if (model_->actuator_trnid[2 * a] == jointId) {
        actuatorId = a;
        break;
      }
    }
    if (actuatorId < 0) {
      ROS_ERROR_STREAM("Actuator for joint " << mjJointName << " (ROS: " << rosJointName << ") not found in MuJoCo model");
      continue;
    }
    actuatorIds_[i] = actuatorId;

    // Register joint state interface
    hardware_interface::JointStateHandle stateHandle(
        rosJointName, &jointPositions_[i], &jointVelocities_[i], &jointEfforts_[i]);
    jointStateInterface_.registerHandle(stateHandle);

    // Register effort joint interface
    hardware_interface::JointHandle effortHandle(stateHandle, &jointEffortCommands_[i]);
    effortJointInterface_.registerHandle(effortHandle);

    // Register hybrid joint interface
    hybridJointDatas_.push_back(HybridJointData{.joint_ = effortHandle});
    HybridJointData& jointData = hybridJointDatas_.back();
    
    HybridJointHandle hybridHandle(
        jointData.joint_, 
        &jointData.posDes_, 
        &jointData.velDes_, 
        &jointData.kp_, 
        &jointData.kd_, 
        &jointData.ff_);
    hybridJointInterface_.registerHandle(hybridHandle);

    // Initialize command buffer
    cmdBuffer_[rosJointName] = std::deque<HybridJointCommand>();
    
    ROS_INFO_STREAM("Registered joint: " << rosJointName 
                    << " -> MuJoCo: " << mjJointName 
                    << " (ID: " << jointId << ")");
  }

  registerInterface(&jointStateInterface_);
  registerInterface(&effortJointInterface_);
  registerInterface(&hybridJointInterface_);
}

void LeggedHWMujoco::setupImu(ros::NodeHandle& robot_hw_nh) {
  XmlRpc::XmlRpcValue imu_config;
  if (!robot_hw_nh.getParam("imus", imu_config)) {
    ROS_WARN("No IMU configuration found");
    return;
  }

  ROS_ASSERT(imu_config.getType() == XmlRpc::XmlRpcValue::TypeStruct);

  for (auto it = imu_config.begin(); it != imu_config.end(); ++it) {
    std::string imuName = it->first;
    
    if (!it->second.hasMember("frame_id")) {
      ROS_ERROR_STREAM("IMU " << imuName << " has no frame_id");
      continue;
    }

    std::string frameId = it->second["frame_id"];
    
    // Resolve the frame to a body ID directly (a site's body via
    // site_bodyid, or the body itself). Previously this stashed a body ID
    // into a "site ID" field and later indexed site_bodyid[] with it - an
    // out-of-bounds read whenever the frame_id matched a body, not a site.
    int bodyId = -1;
    int siteId = mj_name2id(model_, mjOBJ_SITE, frameId.c_str());
    if (siteId >= 0) {
      bodyId = model_->site_bodyid[siteId];
    } else {
      bodyId = mj_name2id(model_, mjOBJ_BODY, frameId.c_str());
      if (bodyId < 0) {
        ROS_ERROR_STREAM("IMU frame " << frameId << " not found in MuJoCo model");
        continue;
      }
    }

    // Get covariances
    XmlRpc::XmlRpcValue oriCov = it->second["orientation_covariance_diagonal"];
    XmlRpc::XmlRpcValue angCov = it->second["angular_velocity_covariance"];
    XmlRpc::XmlRpcValue linCov = it->second["linear_acceleration_covariance"];

    imuDatas_.push_back(ImuData{
        .bodyId_ = bodyId,
        .ori_ = {1., 0., 0., 0.},
        .oriCov_ = {static_cast<double>(oriCov[0]), 0., 0., 
                    0., static_cast<double>(oriCov[1]), 0., 
                    0., 0., static_cast<double>(oriCov[2])},
        .angularVel_ = {0., 0., 0.},
        .angularVelCov_ = {static_cast<double>(angCov[0]), 0., 0., 
                           0., static_cast<double>(angCov[1]), 0., 
                           0., 0., static_cast<double>(angCov[2])},
        .linearAcc_ = {0., 0., 0.},
        .linearAccCov_ = {static_cast<double>(linCov[0]), 0., 0., 
                          0., static_cast<double>(linCov[1]), 0., 
                          0., 0., static_cast<double>(linCov[2])}
    });

    ImuData& imuData = imuDatas_.back();
    
    hardware_interface::ImuSensorHandle imuHandle(
        imuName, frameId,
        imuData.ori_, imuData.oriCov_,
        imuData.angularVel_, imuData.angularVelCov_,
        imuData.linearAcc_, imuData.linearAccCov_);
    
    imuSensorInterface_.registerHandle(imuHandle);
    
    ROS_INFO_STREAM("Registered IMU: " << imuName << " at frame " << frameId);
  }

  registerInterface(&imuSensorInterface_);
}

void LeggedHWMujoco::setupContacts(ros::NodeHandle& robot_hw_nh) {
  XmlRpc::XmlRpcValue contact_config;
  if (!robot_hw_nh.getParam("contacts", contact_config)) {
    ROS_WARN("No contact configuration found");
    return;
  }

  ROS_ASSERT(contact_config.getType() == XmlRpc::XmlRpcValue::TypeArray);

  // URDF -> MuJoCo contact name mapping
  std::map<std::string, std::string> contact_name_map;
  contact_name_map["LF_FOOT"] = "FL_FOOT";
  contact_name_map["RF_FOOT"] = "FR_FOOT";
  contact_name_map["LH_FOOT"] = "RL_FOOT";
  contact_name_map["RH_FOOT"] = "RR_FOOT";

  for (int i = 0; i < contact_config.size(); ++i) {
    std::string rosContactName = static_cast<std::string>(contact_config[i]);
    
    // Find corresponding MuJoCo name
    std::string mjContactName;
    if (contact_name_map.find(rosContactName) != contact_name_map.end()) {
      mjContactName = contact_name_map[rosContactName];
    } else {
      mjContactName = rosContactName;
    }
    
    // Find geom ID using MuJoCo name
    int geomId = mj_name2id(model_, mjOBJ_GEOM, mjContactName.c_str());
    if (geomId < 0) {
      ROS_ERROR_STREAM("Contact geom " << mjContactName << " (ROS: " << rosContactName << ") not found");
      continue;
    }

    ContactData contactData;
    contactData.geomIds_.push_back(geomId);
    contactData.inContact_ = false;
    
    contactDatas_[rosContactName] = contactData;
    
    ContactSensorHandle contactHandle(rosContactName, &contactDatas_[rosContactName].inContact_);
    contactSensorInterface_.registerHandle(contactHandle);
    
    ROS_INFO_STREAM("Registered contact: " << rosContactName 
                    << " -> MuJoCo: " << mjContactName 
                    << " (geom ID: " << geomId << ")");
  }

  registerInterface(&contactSensorInterface_);
}

void LeggedHWMujoco::read(const ros::Time& time, const ros::Duration& period) {
  applyResetIfRequested();
  readJoints();
  readImu(period.toSec());
  readContacts();
  publishGroundTruth(time);
}

void LeggedHWMujoco::publishGroundTruth(const ros::Time& time) {
  if (trunkBodyId_ < 0) return;

  // Mirrors Gazebo's libgazebo_ros_p3d.so config in gazebo.xacro: bodyName
  // "base", frameName "world" - i.e. world-frame pose AND world-frame twist
  // (p3d does not rotate twist into the body frame, unlike REP-103/105).
  nav_msgs::Odometry odom;
  odom.header.stamp = time;
  odom.header.frame_id = "world";

  odom.pose.pose.position.x = data_->qpos[0];
  odom.pose.pose.position.y = data_->qpos[1];
  odom.pose.pose.position.z = data_->qpos[2];
  // MuJoCo qpos quat is (w,x,y,z); ROS geometry_msgs/Quaternion is (x,y,z,w).
  odom.pose.pose.orientation.w = data_->qpos[3];
  odom.pose.pose.orientation.x = data_->qpos[4];
  odom.pose.pose.orientation.y = data_->qpos[5];
  odom.pose.pose.orientation.z = data_->qpos[6];

  mjtNum* cvel = data_->cvel + 6 * trunkBodyId_;  // [angular(3); linear(3)], world-aligned axes
  odom.twist.twist.angular.x = cvel[0];
  odom.twist.twist.angular.y = cvel[1];
  odom.twist.twist.angular.z = cvel[2];
  odom.twist.twist.linear.x = cvel[3];
  odom.twist.twist.linear.y = cvel[4];
  odom.twist.twist.linear.z = cvel[5];

  groundTruthPub_.publish(odom);
}

void LeggedHWMujoco::readJoints() {
  for (size_t i = 0; i < jointIds_.size(); ++i) {
    int jointId = jointIds_[i];
    int qpos_adr = model_->jnt_qposadr[jointId];
    int qvel_adr = model_->jnt_dofadr[jointId];
    
    jointPositions_[i] = data_->qpos[qpos_adr];
    jointVelocities_[i] = data_->qvel[qvel_adr];

    jointEfforts_[i] = data_->actuator_force[actuatorIds_[i]];
  }
}

void LeggedHWMujoco::readImu(double dt) {
  for (auto& imu : imuDatas_) {
    int bodyId = imu.bodyId_;

    // MuJoCo stores xquat as (w, x, y, z); hardware_interface::ImuSensorHandle
    // (and LeggedHWSim/Eigen::Quaternion::coeffs()) expect (x, y, z, w).
    mjtNum* quat = data_->xquat + 4 * bodyId;
    imu.ori_[0] = quat[1];
    imu.ori_[1] = quat[2];
    imu.ori_[2] = quat[3];
    imu.ori_[3] = quat[0];

    // cvel is CoM-based but expressed with WORLD-aligned axes, not the body
    // frame. An IMU reads body-frame rates, so rotate world -> body using
    // the orientation conjugate (inverse rotation). Verified empirically
    // against mj_objectVelocity(..., flg_local=1).
    mjtNum quatConj[4] = {quat[0], -quat[1], -quat[2], -quat[3]};

    mjtNum* cvel = data_->cvel + 6 * bodyId;
    mjtNum angVelWorld[3] = {cvel[0], cvel[1], cvel[2]};
    mju_rotVecQuat(imu.angularVel_, angVelWorld, quatConj);

    // data_->cacc is only populated when the model has a <sensor> that
    // requires it (e.g. accelerometer) - otherwise it silently stays zero
    // even in free fall. Finite-differencing cvel's linear part avoids that
    // trap entirely.
    mjtNum linVelWorld[3] = {cvel[3], cvel[4], cvel[5]};
    mjtNum linAccWorld[3] = {0, 0, 0};
    if (imu.hasPrevVel_ && dt > 1e-9) {
      linAccWorld[0] = (linVelWorld[0] - imu.linVelWorldPrev_[0]) / dt;
      linAccWorld[1] = (linVelWorld[1] - imu.linVelWorldPrev_[1]) / dt;
      linAccWorld[2] = (linVelWorld[2] - imu.linVelWorldPrev_[2]) / dt;
    }
    imu.linVelWorldPrev_[0] = linVelWorld[0];
    imu.linVelWorldPrev_[1] = linVelWorld[1];
    imu.linVelWorldPrev_[2] = linVelWorld[2];
    imu.hasPrevVel_ = true;

    mjtNum linAccBody[3];
    mju_rotVecQuat(linAccBody, linAccWorld, quatConj);

    mjtNum gravity_world[3] = {0, 0, -9.81};
    mjtNum gravity_body[3];
    mju_rotVecQuat(gravity_body, gravity_world, quatConj);

    // Specific force (what an accelerometer reads): kinematic accel - gravity, in body frame.
    imu.linearAcc_[0] = linAccBody[0] - gravity_body[0];
    imu.linearAcc_[1] = linAccBody[1] - gravity_body[1];
    imu.linearAcc_[2] = linAccBody[2] - gravity_body[2];
  }
}

void LeggedHWMujoco::readContacts() {
  for (auto& contact : contactDatas_) {
    contact.second.inContact_ = false;
  }

  for (int i = 0; i < data_->ncon; ++i) {
    const mjContact& con = data_->contact[i];
    
    for (auto& contactPair : contactDatas_) {
      for (int geomId : contactPair.second.geomIds_) {
        if (con.geom1 == geomId || con.geom2 == geomId) {
          contactPair.second.inContact_ = true;
          break;
        }
      }
    }
  }
}

void LeggedHWMujoco::write(const ros::Time& time, const ros::Duration& period) {
  writeJoints(time, period);
  mj_step(model_, data_);
}

void LeggedHWMujoco::writeJoints(const ros::Time& time, const ros::Duration& period) {
  size_t jointIdx = 0;
  for (auto& jointData : hybridJointDatas_) {
    std::string jointName = jointData.joint_.getName();
    auto& buffer = cmdBuffer_[jointName];
    
    if (time == ros::Time(period.toSec())) {
      buffer.clear();
    }

    while (!buffer.empty() && buffer.back().stamp_ + ros::Duration(delay_) < time) {
      buffer.pop_back();
    }

    buffer.push_front(HybridJointCommand{
        .stamp_ = time,
        .posDes_ = jointData.posDes_,
        .velDes_ = jointData.velDes_,
        .kp_ = jointData.kp_,
        .kd_ = jointData.kd_,
        .ff_ = jointData.ff_
    });

    const auto& cmd = buffer.back();

    double pos = jointData.joint_.getPosition();
    double vel = jointData.joint_.getVelocity();
    double torque;
    if (cmd.kp_ == 0.0 && cmd.kd_ == 0.0 && cmd.ff_ == 0.0) {
      // No controller has issued a hybrid command yet (e.g. right after
      // init or /mujoco_hw/reset) - hold the home pose with a fixed PD
      // instead of falling under gravity with zero torque while the real
      // controller's MPC is still computing its first policy.
      static constexpr double kHoldKp = 250.0;
      static constexpr double kHoldKd = 8.0;
      torque = kHoldKp * (jointHomePos_[jointIdx] - pos) - kHoldKd * vel;
    } else {
      torque = cmd.kp_ * (cmd.posDes_ - pos) +
               cmd.kd_ * (cmd.velDes_ - vel) +
               cmd.ff_;
    }

    jointData.joint_.setCommand(torque);
    ++jointIdx;
  }

  for (size_t i = 0; i < jointEffortCommands_.size(); ++i) {
    data_->ctrl[actuatorIds_[i]] = jointEffortCommands_[i];
  }
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedHWMujoco, hardware_interface::RobotHW)
