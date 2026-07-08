bool LeggedHWMujoco::loadModel(const std::string& model_path) {
  // 解析 ROS package 路径
  std::string resolved_path = model_path;
  
  // 处理 $(find package_name) 格式
  size_t find_pos = resolved_path.find("$(find ");
  if (find_pos != std::string::npos) {
    size_t end_pos = resolved_path.find(")", find_pos);
    if (end_pos != std::string::npos) {
      std::string package_part = resolved_path.substr(find_pos + 7, end_pos - find_pos - 7);
      size_t slash_pos = package_part.find('/');
      std::string package_name, rest_path;
      
      if (slash_pos != std::string::npos) {
        package_name = package_part.substr(0, slash_pos);
        rest_path = package_part.substr(slash_pos);
      } else {
        package_name = package_part;
        rest_path = "";
      }
      
      // 使用 rospack 获取包路径
      std::string package_path = ros::package::getPath(package_name);
      if (package_path.empty()) {
        ROS_ERROR_STREAM("Package " << package_name << " not found");
        return false;
      }
      
      resolved_path = package_path + rest_path;
    }
  }
  
  ROS_INFO_STREAM("Loading MuJoCo model from: " << resolved_path);
  
  char error[1000];
  model_ = mj_loadXML(resolved_path.c_str(), nullptr, error, 1000);
  
  if (!model_) {
    ROS_ERROR_STREAM("Failed to load MuJoCo model: " << error);
    return false;
  }

  data_ = mj_makeData(model_);
  if (!data_) {
    ROS_ERROR("Failed to create MuJoCo data");
    return false;
  }

  mj_resetData(model_, data_);
  mj_forward(model_, data_);

  ROS_INFO_STREAM("Loaded MuJoCo model: " << resolved_path);
  ROS_INFO_STREAM("  DOF: " << model_->nv);
  ROS_INFO_STREAM("  Actuators: " << model_->nu);
  
  return true;
}
