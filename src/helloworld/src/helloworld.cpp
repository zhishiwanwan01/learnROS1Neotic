#include "ros/ros.h"

int main(int argc, char** argv) {
  // ROS 节点初始化
  ros::init(argc, argv, "helloworld_node");
  // 创建节点句柄(非必需)
  ros::NodeHandle nh;
  // 控制台输出 hello world
  ROS_INFO("Hello, World!");

  return 0;
}
