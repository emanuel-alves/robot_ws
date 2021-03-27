#include "std_msgs/msg/char.hpp"
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <utility>

#include <termios.h>
#include <stdio.h>

#include <iostream>

using namespace std::chrono;

namespace newGet
{
  static struct termios old, current;
  void initTermios(const int &echo)
  {
    tcgetattr(0, &old);         
    current = old;              
    current.c_lflag &= ~ICANON; 
    if (echo)
    {
      current.c_lflag |= ECHO; 
    }
    else
    {
      current.c_lflag &= ~ECHO;
    }
    tcsetattr(0, TCSANOW, &current); 
  }


  char getch(int echo)
  {
    char ch;
    initTermios(echo);
    ch = getchar();
    tcsetattr(0, TCSANOW, &old);
    return ch;
  }

}

class KeyControl : public rclcpp::Node
{
private:
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr key_pub;

  void dollyKey()
  {
    auto key_msg = std::make_unique<std_msgs::msg::Char>();
    key_msg->data = newGet::getch(0);
    key_pub->publish(std::move(key_msg));
  }

public:
  KeyControl()
      : Node("KeyControl")
  {

    key_pub = this->create_publisher<std_msgs::msg::Char>("dollyControl", 10);
    timer = this->create_wall_timer(
        500ms, std::bind(&KeyControl::dollyKey, this));
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyControl>();

  printf("Controle iniciado!\n");
  printf("Input > ");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
