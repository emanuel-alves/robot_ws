#include <memory>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <dolly_data/srv/dolly_position.hpp>
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace dolly_data::srv;
using namespace std;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;

struct Position
{
  float x, y, z;
} position;
struct Orientation
{
  float x, y, z, w;
} orientation;

class DollyLocale : public rclcpp::Node
{
private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr dollyGPS;

public:
  DollyLocale() : Node("dolly_auto")
  {
    dollyGPS = this->create_subscription<nav_msgs::msg::Odometry>("dolly/odom", 10, std::bind(&DollyLocale::updatePosition, this, _1));
  }

  void updatePosition(const nav_msgs::msg::Odometry::SharedPtr locale)
  {
    position.x = locale->pose.pose.position.x;
    position.y = locale->pose.pose.position.y;
    position.z = locale->pose.pose.position.z;

    orientation.x = locale->pose.pose.orientation.x;
    orientation.y = locale->pose.pose.orientation.y;
    orientation.z = locale->pose.pose.orientation.z;
    orientation.w = locale->pose.pose.orientation.w;
  }
};

class DollyAuto : public rclcpp::Node
{
private:
  rclcpp::Service<DollyPosition>::SharedPtr dollyServer;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr dollySetPosition;
  float velLinX, velAngZ, angle, angleDolly, errorLin, errorAng;

  float getAngle(const float &x1, const float &y1, const float &x2, const float &y2)
  {
    float c1 = (y2 - y1);
    float c2 = (x2 - x1);
    angle = atan2(c1, c2);

    return angle;
  }
  float getAngleDolly()
  {

    //float siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y);
    //float cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z);
    //return std::atan2(siny_cosp, cosy_cosp);
    return std::atan2(
        2 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z));
  }

public:
  DollyAuto() : Node("dolly_auto")
  {
    velLinX = 1;
    velAngZ = 0.2;
    errorAng = 0.1;
    errorLin = 0.2;
    dollyServer = this->create_service<DollyPosition>("dolly_position", std::bind(&DollyAuto::dolly_control, this, _1, _2, _3));

    dollySetPosition = this->create_publisher<geometry_msgs::msg::Twist>(
        "/dolly/cmd_vel", 10);
  }

  void dolly_control(const shared_ptr<rmw_request_id_t> request_header, const shared_ptr<DollyPosition::Request> request, shared_ptr<DollyPosition::Response> response)
  {
    (void)request_header;
    float angleDolly;
    auto sendPos = geometry_msgs::msg::Twist();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Movimentação iniciada!");
    // Movimentação angular
    angle = getAngle(position.x, position.y, request->x, request->y);

    sendPos.angular.z = velAngZ * (angle > getAngleDolly() ? 1 : -1);
    dollySetPosition->publish(sendPos);
    sendPos = geometry_msgs::msg::Twist();

    do
    {
      angleDolly = getAngleDolly();
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%f != %f -> %f", angle, angleDolly, abs(angle - angleDolly));
    } while (abs(angle - angleDolly) >= errorAng);

    // Movimento Linear
    sendPos.angular.z = 0;
    sendPos.linear.x = velLinX;
    dollySetPosition->publish(sendPos);
    sendPos = geometry_msgs::msg::Twist();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Movimentação linear!");
    while ((abs(position.x - request->x) > errorLin || abs(position.y - request->y) > errorLin))
    {
      angleDolly = getAngleDolly();
      angle = getAngle(position.x, position.y, request->x, request->y);

      if (abs(angle - angleDolly) > errorAng)
      {
        sendPos.angular.z = velAngZ * (angle > getAngleDolly() ? 1 : -1);
        sendPos.linear.x = velLinX;
        dollySetPosition->publish(sendPos);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Angulação Regulada (-velAngZ)");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "angle: %f <>  getangle: %f", angle, angleDolly);
      }
      else if (sendPos.angular.z != 0)
      {
        sendPos.angular.z = 0;
        sendPos.linear.x = velLinX;
        dollySetPosition->publish(sendPos);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Angulação Regulada (0)");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "angle: %f <>  getangle: %f", angle, angleDolly);
      }
    }
    sendPos = geometry_msgs::msg::Twist();
    sendPos.linear.x = 0;
    dollySetPosition->publish(sendPos);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Solicitação concluida!");
    response->status = 1;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto nodePosition = std::make_shared<DollyLocale>();
  auto nodeAuto = std::make_shared<DollyAuto>();
  executor.add_node(nodePosition);
  executor.add_node(nodeAuto);

  executor.spin();
  rclcpp::shutdown();
}