#include <iostream>
#include <iomanip>
#include <memory>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std;
using std::placeholders::_1;

class PrintLocale : public rclcpp::Node
{
public:
  PrintLocale()
      : Node("dolly_print_locale")
  {
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>( // CHANGE
        "dolly/odom", 10, std::bind(&PrintLocale::topic_callback, this, _1));
  }

private:
  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr locale) const // CHANGE
  {
    cout << "\nPosition:\n";
    cout <<   "   Escalar:\n";
    cout <<   "     x: " << locale->pose.pose.position.x << endl;
    cout <<   "     y: " << locale->pose.pose.position.y << endl;
    cout <<   "     z: " << locale->pose.pose.position.z << endl;
    cout <<   "   angle: " << std::atan2(
        2 * (locale->pose.pose.orientation.w * locale->pose.pose.orientation.z + locale->pose.pose.orientation.x * locale->pose.pose.orientation.y),
        1 - 2 * (locale->pose.pose.orientation.y * locale->pose.pose.orientation.y + locale->pose.pose.orientation.z * locale->pose.pose.orientation.z)) << endl;
    cout <<   "   Quartenios:\n";
    cout <<   "     x: " << locale->pose.pose.orientation.x << " -> " <<(2*acos(locale->pose.pose.orientation.x))*(180/M_PI) << endl;    
    cout <<   "     y: " << locale->pose.pose.orientation.y << " -> " <<(2*acos(locale->pose.pose.orientation.y))*(180/M_PI) << endl;
    cout <<   "     z: " << locale->pose.pose.orientation.z << " -> " <<(2*acos(locale->pose.pose.orientation.z))*(180/M_PI) << endl;
    cout <<   "     w: " << locale->pose.pose.orientation.w << " -> " <<(2*acos(locale->pose.pose.orientation.w))*(180/M_PI) << endl; //Este aqui

  }
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_; // CHANGE
};

int main(int argc, char *argv[])
{
  cout << setprecision(4) << fixed;
  cout << "\nOdometry iniciada...\n";
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PrintLocale>());
  rclcpp::shutdown();
  return 0;
}