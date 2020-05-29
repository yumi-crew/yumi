#include <rws_clients/robot_manager_client.hpp>


namespace rws_clients
{

// namespaced helper functions..........................................................................................
unsigned int random_char()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, 255);
  return dis(gen);
}

std::string generate_hex(const unsigned int len)
{
  std::stringstream ss;
  for (size_t i = 0; i < len; i++)
  {
    const auto rc = random_char();
    std::stringstream hexstream;
    hexstream << std::hex << rc;
    auto hex = hexstream.str();
    ss << (hex.length() < 2 ? '0' + hex : hex);
  }
  return ss.str();
}
// .....................................................................................................................


RobotManagerClient::RobotManagerClient(std::shared_ptr<rclcpp::Node> node)
:
node_(node)
{}


bool RobotManagerClient::init()
{
  return true;
}


bool RobotManagerClient::robot_is_ready()
{
  using IsReady  = yumi_robot_manager_interfaces::srv::IsReady;
  using namespace std::chrono_literals;

  // temp node which will handle communication with the service server
  auto temp_node = std::make_unique<rclcpp::Node>("temp_"+generate_hex(8));
  auto client = temp_node->create_client<IsReady>("/IsReady");           
  
  // loop logic
  unsigned int retryCount = 0;
  constexpr unsigned int maxRetries = 10;

  while (retryCount < maxRetries)
  {
    client->wait_for_service(1.5s);
    if (!client->service_is_ready()) 
    {
      retryCount++;
      RCLCPP_ERROR(node_->get_logger(), 
        "IsReady service failed to start, check that service server is launched. Retries left: %d", 
        maxRetries - retryCount);
      continue;
    }

    auto req = std::make_shared<yumi_robot_manager_interfaces::srv::IsReady::Request>();
    auto resp = client->async_send_request(req);
    
    auto spin_status = rclcpp::spin_until_future_complete(temp_node->get_node_base_interface(), resp, 3s);
    if (spin_status != rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      retryCount++;
      RCLCPP_ERROR(node_->get_logger(), 
        "IsReady service failed to execute (spin failed). Retries left: %d", maxRetries - retryCount);
      continue;
    }

    auto status = resp.wait_for(1s);
    if (status != std::future_status::ready)
    {
      retryCount++;
      RCLCPP_ERROR(node_->get_logger(), 
        "Client service request failed to execute. Retries left: %d", maxRetries - retryCount);
      continue;
    }
   
    if(resp.get()->is_ready)
    {
      return true;
    }
    else
    {
      return false;
    }
    
  }

  RCLCPP_ERROR(node_->get_logger(), "failed to communicate with service server");
  return false;
}



bool RobotManagerClient::stop_egm()
{
  using StopEgm  = yumi_robot_manager_interfaces::srv::StopEgm;
  using namespace std::chrono_literals;

  // temp node which will handle communication with the service server
  auto temp_node = std::make_unique<rclcpp::Node>("temp_"+generate_hex(8));
  auto client = temp_node->create_client<StopEgm>("/StopEgm");           
  
  // loop logic
  unsigned int retryCount = 0;
  constexpr unsigned int maxRetries = 10;

  while (retryCount < maxRetries)
  {
    client->wait_for_service(1.5s);
    if (!client->service_is_ready()) 
    {
      retryCount++;
      RCLCPP_ERROR(node_->get_logger(), 
        "StopEgm service failed to start, check that service server is launched. Retries left: %d", 
        maxRetries - retryCount);
      continue;
    }

    auto req = std::make_shared<yumi_robot_manager_interfaces::srv::StopEgm::Request>();
    auto resp = client->async_send_request(req);
    
    auto spin_status = rclcpp::spin_until_future_complete(temp_node->get_node_base_interface(), resp, 3s);
    if (spin_status != rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      retryCount++;
      RCLCPP_ERROR(node_->get_logger(), 
        "StopEgm service failed to execute (spin failed). Retries left: %d", maxRetries - retryCount);
      continue;
    }

    auto status = resp.wait_for(1s);
    if (status != std::future_status::ready)
    {
      retryCount++;
      RCLCPP_ERROR(node_->get_logger(), 
        "Client service request failed to execute. Retries left: %d", 
        maxRetries - retryCount);
      continue;
    }

   
    if( resp.get()->is_stopped )
    {
      RCLCPP_INFO(node_->get_logger(), "EGM control is stopped");
      return true;
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to stop EGM control");
      return false;
    }
    
  }

  RCLCPP_ERROR(node_->get_logger(), "failed to communicate with service server");
  return false;
}
  

bool RobotManagerClient::start_egm()
{
  using StartEgm = yumi_robot_manager_interfaces::srv::StartEgm;
  using namespace std::chrono_literals;

  // temp node which will handle communication with the service server
  auto temp_node = std::make_unique<rclcpp::Node>("temp_"+generate_hex(8));
  auto client = temp_node->create_client<StartEgm>("/StartEgm");           
  
  // loop logic
  unsigned int retryCount = 0;
  constexpr unsigned int maxRetries = 10;

  while (retryCount < maxRetries)
  {
    client->wait_for_service(1.5s);
    if (!client->service_is_ready()) 
    {
      retryCount++;
      RCLCPP_ERROR(node_->get_logger(), 
        "StartEgm service failed to start, check that service server is launched. Retries left: %d", 
        maxRetries - retryCount);
      continue;
    }

    auto req = std::make_shared<yumi_robot_manager_interfaces::srv::StartEgm::Request>();
    auto resp = client->async_send_request(req);
    
    auto spin_status = rclcpp::spin_until_future_complete(temp_node->get_node_base_interface(), resp, 3s);
    if (spin_status != rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      retryCount++;
      RCLCPP_ERROR(node_->get_logger(), 
        "StartEgm service failed to execute (spin failed). Retries left: %d", maxRetries - retryCount);
      continue;
    }

    auto status = resp.wait_for(1s);
    if (status != std::future_status::ready)
    {
      retryCount++;
      RCLCPP_ERROR(node_->get_logger(), 
        "Client service request failed to execute. Retries left: %d", 
        maxRetries - retryCount);
      continue;
    }

   
    if(resp.get()->is_started)
    {
      RCLCPP_INFO(node_->get_logger(), "EGM control is started");
      return true;
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to start EGM control");
      return false;
    }
  }

  RCLCPP_ERROR(node_->get_logger(), "Failed to communicate with service server");
  return false;
}


bool RobotManagerClient::stop_motors()
{
  using StopMotors = yumi_robot_manager_interfaces::srv::StopMotors;
  using namespace std::chrono_literals;

  // temp node which will handle communication with the service server
  auto temp_node = std::make_unique<rclcpp::Node>("temp_"+generate_hex(8));
  auto client = temp_node->create_client<StopMotors>("/StopMotors");           
  
  // loop logic
  unsigned int retryCount = 0;
  constexpr unsigned int maxRetries = 10;

  while (retryCount < maxRetries)
  {
    client->wait_for_service(1.5s);
    if (!client->service_is_ready()) 
    {
      retryCount++;
      RCLCPP_ERROR(node_->get_logger(), 
        "StopMotors service failed to start, check that service server is launched. Retries left: %d", 
        maxRetries - retryCount);
      continue;
    }

    auto req = std::make_shared<yumi_robot_manager_interfaces::srv::StopMotors::Request>();
    auto resp = client->async_send_request(req);
    
    auto spin_status = rclcpp::spin_until_future_complete(temp_node->get_node_base_interface(), resp, 3s);
    if (spin_status != rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      retryCount++;
      RCLCPP_ERROR(node_->get_logger(), 
        "StopMotors service failed to execute (spin failed). Retries left: %d", maxRetries - retryCount);
      continue;
    }

    auto status = resp.wait_for(1s);
    if (status != std::future_status::ready)
    {
      retryCount++;
      RCLCPP_ERROR(node_->get_logger(), 
        "Client service request failed to execute. Retries left: %d", 
        maxRetries - retryCount);
      continue;
    }

    if(resp.get()->motors_off)
    {
      RCLCPP_INFO(node_->get_logger(), "Stopping Motors");
      return true;
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Failed to turn off motors");
      return false;
    }
  }

  RCLCPP_ERROR(node_->get_logger(), "Failed to communicate with service server");
  return false;
}

} // end namespace rws_clients