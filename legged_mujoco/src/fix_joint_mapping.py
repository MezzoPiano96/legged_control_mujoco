#!/usr/bin/env python3
import re

# 读取原文件
with open('LeggedHWMujoco.cpp', 'r') as f:
    content = f.read()

# 定义新的 setupJoints 函数内容
new_setup_joints = '''void LeggedHWMujoco::setupJoints(ros::NodeHandle& robot_hw_nh) {
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
  jointPositions_.resize(nJoints, 0.0);
  jointVelocities_.resize(nJoints, 0.0);
  jointEfforts_.resize(nJoints, 0.0);
  jointEffortCommands_.resize(nJoints, 0.0);

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
}'''

# 找到并替换 setupJoints 函数
# 匹配从函数开始到下一个函数定义之前
pattern = r'void LeggedHWMujoco::setupJoints\(ros::NodeHandle& robot_hw_nh\) \{.*?\n\}\n\nvoid LeggedHWMujoco::setupImu'

if re.search(pattern, content, re.DOTALL):
    content = re.sub(pattern, new_setup_joints + '\n\nvoid LeggedHWMujoco::setupImu', content, flags=re.DOTALL)
    print("✅ Successfully replaced setupJoints function")
else:
    print("❌ Pattern not found, trying alternative...")
    # 尝试更宽松的匹配
    pattern2 = r'void LeggedHWMujoco::setupJoints\([^)]+\)[^{]*\{[^}]*(?:\{[^}]*\}[^}]*)*\}'
    if re.search(pattern2, content, re.DOTALL):
        # 手动处理
        print("⚠️  Please manually edit the file")
    else:
        print("❌ Could not find setupJoints function")

# 写回文件
with open('LeggedHWMujoco.cpp', 'w') as f:
    f.write(content)

print("Done!")
