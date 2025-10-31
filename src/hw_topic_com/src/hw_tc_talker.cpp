#include <sstream>  // For std::ostringstream
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

int main(int argc, char** argv) {
  // 1. 设置中文环境
  setlocale(LC_ALL, "");
  // 2. 初始化ROS节点
  // 第三个参数要和launch文件中node定义里的name一致
  ros::init(argc, argv, "hw_tc_talker");
  // 3. 创建节点句柄，这个类封装了与ROS系统通信的所有功能
  ros::NodeHandle nh;
  // 4. 创建一个发布者对象
  /**
   * @brief 创建一个发布者对象，用于发送字符串消息。
   * @tparam M 消息类型的模板参数，此处为`std_msgs::String`。
   * @param topic 发布的话题名称，当前为`"chatter"`。
   * @param queue_size 发布时缓存的最大消息数量，这里设置为`10`。
   * - 消息类型: std_msgs::String - 泛型类型，指定发布的消息类型
   * - 话题名称: "chatter" - 指定发布消息的话题
   * - 队列长度: 10 - 消息传输过程中允许缓存的最大消息数量
   */
  ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 10);

  // ==================== 重要知识点：第一条消息丢失问题 ====================
  /**
   * @问题: 为什么订阅时第一条数据会丢失？
   * @原因: ROS的发布-订阅是异步通信机制
   *   1. advertise() 创建publisher后，需要时间向 roscore 注册
   *   2. roscore 需要时间将publisher信息广播给所有节点
   *   3. subscriber 需要时间发现publisher并建立TCP连接
   *   4. 如果在连接建立完成前就发布消息，subscriber还未准备好接收   *
   * @解决方案:
   *   【方案1】固定延迟(简单，学习推荐)
   *     ros::Duration(3.0).sleep();  // 等待3秒确保注册完成
   *
   *   【方案2】动态检查订阅者(生产环境推荐)
   *     while(ros::ok() && pub.getNumSubscribers() == 0) {
   *         ros::Duration(0.1).sleep();  // 每100ms检查一次
   *     }
   *     // 至少有一个订阅者时才开始发布
   *
   *   【方案3】Latch模式(特定场景)
   *     ros::Publisher pub = nh.advertise<...>(..., true);  // 第3个参数=true
   *     // 会缓存最后一条消息，新订阅者连接时立即收到
   *
   * @实验验证:
   *   1. 注释掉下面的 sleep，先启动listener再启动talker
   *      → listener收不到第一条消息(No.0)
   *   2. 恢复 sleep，同样顺序启动
   *      → listener能收到完整消息，包括No.0
   */
  // ros::Duration(3.0).sleep();  // 等待publisher在roscore注册完成
  // ========================================================================

  // 5. 组织被发布的数据，并编写发布数据
  // 5.1
  // 定义一个对象数据（动态组织），msg里有一个string类型的成员变量data，用法：msg.data
  // = "hello world";
  std_msgs::String msg;
  std::string msg_front{"hello, 你好！"};

  int count = 0;
  // 5.2 频率（1Hz/1秒1次）
  // 这里 rate(1) 是调用了构造函数，创建了一个 ros::Rate
  // 对象，指定循环频率为1Hz，可以用列表初始化的方式写作 ros::Rate rate{1};
  ros::Rate rate(1);
  // 节点不死就一直发布数据
  while (ros::ok()) {
    // 使用 stringstream 拼接字符串与编号
    // std::stringstream ss;
    // ss << msg_front << "_No." << count++;
    // msg.data = ss.str();

    // 5.3 拼接字符串。上面方法较老旧，C++11之后可以使用更简洁的方式
    msg.data = msg_front + "_No." + std::to_string(count++);
    // 5.4 发布数据
    pub.publish(msg);
    // 6. 输出日志信息
    // ROS_INFO 是ROS中打印日志信息的宏，类似于C语言中的printf
    // 因此需要使用c_str()把std::string转换为C风格字符串
    ROS_INFO("发布的消息：%s", msg.data.c_str());
    // 7. 按照循环频率延时
    rate.sleep();
    // 8. 处理回调函数(这个talker节点没有回调函数，可以省略)
    ros::spinOnce();
  }

  return 0;
}
