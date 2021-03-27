#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include "std_msgs/msg/char.hpp"

#include <memory>
#include <utility>

#define INT_W 119
#define INT_A 97
#define INT_S 115
#define INT_D 100

class Teleop : public rclcpp::Node
{
private:
  rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr key_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;

  bool flag;
  double fatorAngular = 0.2, fatorLinear = 0.3, linearX = 0, angularZ = 0;

  void dollyControl(const std_msgs::msg::Char::SharedPtr keyMsg)
  {
    auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();

    switch (keyMsg->data)
    {
    case INT_A:
      if (angularZ < -1)
        angularZ = 0;
      angularZ += 1;
      break;
    case INT_D:
      if (angularZ > 1)
        angularZ = 0;
      angularZ += -1;
      break;
    case INT_W:
      if (linearX < -1)
        linearX = 0;
      linearX += 1;
      break;
    case INT_S:
      if (linearX > 1)
        linearX = 0;
      linearX += -1;
      break;
    default:
      linearX = angularZ = 0;
    }

    cmd_msg->angular.z = angularZ * fatorAngular;
    cmd_msg->linear.x = linearX * fatorLinear;
    cmd_pub->publish(std::move(cmd_msg));
  }

public:
  Teleop()
      : Node("Teleop")
  {
    flag = true;
    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    key_sub = this->create_subscription<std_msgs::msg::Char>(
        "dollyControl", 10, std::bind(&Teleop::dollyControl, this, std::placeholders::_1));
    cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel", default_qos);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Teleop>();

  printf("Controle iniciado!\n");
  printf("Input > ");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

/*
#define LINEAR_CONST 0.2
#define ANGULAR_CONST 0.2
#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

using namespace std::chrono;

class Teleop : public rclcpp::Node
{
private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
  rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr cmd_sub;

  double linearX, scaleY;
  int count = 0;
  char ch;

  void dollyControl(const std_msgs::msg::Char::SharedPtr keyMsg)
  {
    auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
    bool flag = false;
    switch (keyMsg->data)
    {
    case KEYCODE_L:
      //angular_ = 1.0;
      flag = true;
      break;
    case KEYCODE_R:
      //angular_ = -1.0;
      flag = true;
      break;
    case KEYCODE_U:
      //linear_ = 1.0;
      flag = true;
      break;
    case KEYCODE_D:
      //linear_ = -1.0;
      flag = true;
      break;
    }

    if (flag)
    {
      cmd_msg->linear.x = count * LINEAR_CONST;
      cmd_msg->angular.z = count * ANGULAR_CONST;

      cmd_pub->publish(std::move(cmd_msg));
      count++;
    }
  }
public:
  Teleop()
      : Node("teleop")
  {
    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

    cmd_sub = this->create_subscription<std_msgs::msg::Char>(
      "readKey", 10,
      std::bind(&Teleop::dollyControl, this, std::placeholders::_1));

    cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", default_qos);
  }
};

*/