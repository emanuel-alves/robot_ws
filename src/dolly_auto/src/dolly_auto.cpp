#include <memory>
#include <utility>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <dolly_data/srv/dolly_position.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#define DIRECTION_ANGLE(x1, y1, x2, y2) (y2 - y1) * (x1) - (x2 - x1) * (y1)

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
sensor_msgs::msg::LaserScan laser_data;

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

class DollySensor : public rclcpp::Node
{
private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
  double inf;

public:
  DollySensor() : Node("dolly_sensor")
  {
    inf = std::numeric_limits<double>::infinity();
    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "dolly/laser_scan", default_qos,
        std::bind(&DollySensor::OnSensorMsg, this, _1));
  }
  void OnSensorMsg(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    laser_data = *msg;
  }
};

class DollyAuto : public rclcpp::Node
{
private:
  rclcpp::Service<DollyPosition>::SharedPtr dollyServer;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr dollySetPosition;
  float velLinX, velAngZ, angle, angleDolly, errorLin, errorAng;
  double min_dist;

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
  bool readSensor(int &min, int &max)
  {
    min = max = -1;
    bool status = 1;
    for (int a = 0; a < laser_data.ranges.size(); a++)
    {
      if (laser_data.ranges[a] <= 4)
      {
        if (min == -1)
          min = a;
        max = a;
        if (status && laser_data.ranges[a] <= min_dist)
          status = 0;
      }
    }
    return !status;
  }

public:
  DollyAuto() : Node("dolly_auto")
  {
    velLinX = 1;
    velAngZ = 0.2;
    errorAng = 0.1;
    errorLin = 0.2;
    min_dist = 3.0;
    dollyServer = this->create_service<DollyPosition>("dolly_position", std::bind(&DollyAuto::dolly_control, this, _1, _2, _3));

    dollySetPosition = this->create_publisher<geometry_msgs::msg::Twist>(
        "/dolly/cmd_vel", 10);
  }
  void dolly_control(const shared_ptr<rmw_request_id_t> request_header, const shared_ptr<DollyPosition::Request> request, shared_ptr<DollyPosition::Response> response)
  {
    (void)request_header;
    if (abs(position.x - request->x) <= errorLin && abs(position.y - request->y) <= errorLin)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pera ai... você está bem proximo do ponto, acho que não preciso me movimentar...");
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pos: x = %f, y = %f, erro = %f", position.x, position.y, sqrt(pow(position.x - request->x, 2) + pow(position.y - request->y, 2)));
      response->status = 1;
      return;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Movimentação iniciada!");
    auto sendPos = geometry_msgs::msg::Twist();
    /*
    angle = getAngle(position.x, position.y, request->x, request->y);
    sendPos.angular.z = velAngZ * (angle > getAngleDolly() ? 1 : -1);
    dollySetPosition->publish(sendPos);

    do
    {
      angleDolly = getAngleDolly();
      //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%f != %f -> %f", angle, angleDolly, abs(angle - angleDolly));
    } while (abs(angle - angleDolly) >= errorAng);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%f != %f -> %f", angle, angleDolly, abs(angle - angleDolly));
    sendPos.angular.z = 0;
    sendPos.linear.x = velLinX;
    dollySetPosition->publish(sendPos);
    */
    sendPos.angular.z = 0;
    sendPos.linear.x = velLinX;
    dollySetPosition->publish(sendPos);

    do
    {
      int minPos, maxPos;
      while (readSensor(minPos, maxPos))
      {
        float aux = velAngZ * (minPos > laser_data.ranges.size() - maxPos ? -1 : 1);
        if (aux != sendPos.angular.z)
        {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Corrigindo rota...");
          sendPos.angular.z = aux;
          sendPos.linear.x = velLinX;
          dollySetPosition->publish(sendPos);
        }
      }
      /*
        if (readSensor(minPos, maxPos))
        {
          sendPos.angular.z = velAngZ * (minPos > laser_data.ranges.size() - maxPos ? -1 : 1);
          sendPos.linear.x = velLinX;
          dollySetPosition->publish(sendPos);
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Foram encontrado obstaculos!");
          do
          {
            float aux = velAngZ * (minPos > laser_data.ranges.size() - maxPos ? -1 : 1);
            if (sendPos.angular.z != aux)
            {
              sendPos.angular.z = aux;
              sendPos.linear.x = velLinX;
              dollySetPosition->publish(sendPos);

              RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Corrigindo rota...");
            }
          } while (readSensor(minPos, maxPos));
          sendPos.angular.z = 0;
          sendPos.linear.x = velLinX;
          dollySetPosition->publish(sendPos);

          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Obstaculos resolvidos!");
        }
        */
      angleDolly = getAngleDolly();
      angle = getAngle(position.x, position.y, request->x, request->y);

      float directionAngle = DIRECTION_ANGLE(cos(angleDolly), sin(angleDolly), cos(angle), sin(angle));
      if (directionAngle != sendPos.angular.z)
      {

        sendPos.angular.z = directionAngle;
        dollySetPosition->publish(sendPos);
      }
      /*
      
      if (abs(angle - angleDolly) > errorAng*2)
      {
        sendPos.angular.z = velAngZ * (angle > angleDolly ? 1 : -1);
        sendPos.linear.x = velLinX;
        dollySetPosition->publish(sendPos);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Correção: ang=%f, dol=%f, Z=%f", angle, angleDolly, sendPos.angular.z);
      }
      else if (sendPos.angular.z != 0)
      {
        sendPos.angular.z = 0;
        sendPos.linear.x = velLinX;
        dollySetPosition->publish(sendPos);
      }
      */

    } while ((abs(position.x - request->x) > errorLin || abs(position.y - request->y) > errorLin));

    sendPos.linear.x = sendPos.angular.z = 0;
    dollySetPosition->publish(sendPos);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Solicitação concluida...");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pos: x = %f, y = %f, erro = %f", position.x, position.y, sqrt(pow(position.x - request->x, 2) + pow(position.y - request->y, 2)));
    response->status = 1;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto nodePosition = std::make_shared<DollyLocale>();
  auto nodeAuto = std::make_shared<DollyAuto>();
  auto nodeFollow = std::make_shared<DollySensor>();
  executor.add_node(nodePosition);
  executor.add_node(nodeAuto);
  executor.add_node(nodeFollow);

  executor.spin();
  rclcpp::shutdown();
}
