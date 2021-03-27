#include <chrono>
#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "dolly_data/srv/dolly_position.hpp"

using namespace dolly_data::srv;
using namespace std::chrono_literals;
using namespace std;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 4)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "erro na quantidade de parametros, e");
    return 1;
  }

  shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("dolly_position_client");

  rclcpp::Client<DollyPosition>::SharedPtr client =
      node->create_client<DollyPosition>("dolly_position");

  auto request = std::make_shared<DollyPosition::Request>();
  request->x = atoll(argv[1]);
  request->y = atoll(argv[2]);
  request->z = 0;

  while (!client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Aguarde... Estamos tentando conectar com o Dolly.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vish... Não foi possível conectar...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    if (result.get()->status)
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Você chegou ao seu destino");
    else
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Você não chegou ao seu destino");
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Vish... Deu ruim...");
  }

  rclcpp::shutdown();
  return 0;
}