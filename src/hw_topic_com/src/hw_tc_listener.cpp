// 实现流程:
//     1.包含头文件
//     2.初始化 ROS 节点:命名(唯一)
//     3.实例化 ROS 句柄
//     4.实例化 订阅者 对象
//     5.处理订阅的消息(回调函数)
//     6.设置循环调用回调函数

#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  // 回调函数，用于处理接收到的消息
  ROS_INFO("接收到的消息：%s", msg->data.c_str());
}

int main(int argc, char** argv) {
  setlocale(LC_ALL, "");  // 设置中文环境

  ros::init(argc, argv,
            "hw_tc_listener");  // 初始化ROS节点，节点名称为hw_tc_listener
  ros::NodeHandle nh;           // 创建节点句柄，封装了与ROS系统通信的功能

  // 创建一个订阅者对象，订阅std_msgs::String类型的消息
  // 这行代码注册了一个订阅者，告诉 ROS：
  // “我这个节点要订阅名为 /chatter 的话题，
  // 消息类型是 std_msgs::String，
  // 每当有新消息到达时，请帮我调用函数 chatterCallback。”
  ros::Subscriber sub =
      nh.subscribe<std_msgs::String>("chatter", 10, chatterCallback);
  ros::spin();  // 循环调用回调函数，处理接收到的消息
  return 0;
}
