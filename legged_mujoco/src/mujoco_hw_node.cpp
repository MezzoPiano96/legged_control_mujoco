#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <legged_mujoco/LeggedHWMujoco.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "legged_hw_mujoco_node");
    ros::NodeHandle nh;
    ros::NodeHandle robot_hw_nh("~");
    
    // 创建硬件接口
    legged::LeggedHWMujoco hw;
    if (!hw.init(nh, robot_hw_nh)) {
        ROS_ERROR("Failed to initialize MuJoCo hardware interface");
        return 1;
    }
    
    // 创建控制器管理器
    controller_manager::ControllerManager cm(&hw, nh);
    
    // 设置循环频率
    double loop_hz;
    robot_hw_nh.param("loop_frequency", loop_hz, 1000.0);
    ros::Rate rate(loop_hz);
    
    ROS_INFO("MuJoCo hardware interface started at %.0f Hz", loop_hz);
    
    ros::AsyncSpinner spinner(2);
    spinner.start();
    
    ros::Time last_time = ros::Time::now();
    
    while (ros::ok()) {
        ros::Time current_time = ros::Time::now();
        ros::Duration period = current_time - last_time;
        last_time = current_time;
        
        // 读取传感器数据
        hw.read(current_time, period);
        
        // 更新控制器
        cm.update(current_time, period);
        
        // 写入控制命令
        hw.write(current_time, period);
        
        rate.sleep();
    }
    
    spinner.stop();
    return 0;
}
