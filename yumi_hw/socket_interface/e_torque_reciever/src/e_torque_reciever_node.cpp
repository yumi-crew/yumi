#include <e_torque_reciever/e_torque_reciever.hpp>

std::shared_ptr<socket_interface::ETorqueReciever> etorque_reciever;

// Ctr+C handler
void signal_callback_handler(int signum)
{
  std::cout << "Caught signal " << signum << std::endl;
  // Stop streams
  etorque_reciever->stop_streams();
  // Disconenct
  etorque_reciever->disconnect();
  // Terminate ros node
  rclcpp::shutdown();
  // Terminate script
  exit(signum);
}

int main(int argc, char *argv[])
{
  // Ctrl+C handler
  signal(SIGINT, signal_callback_handler);

  rclcpp::init(argc, argv);
  etorque_reciever = std::make_shared<socket_interface::ETorqueReciever>("e_torque_reciever", "192.168.125.1", 2020);

  std::cout << "before connect()" << std::endl;
  int num_retries = 10;
  if (!etorque_reciever->establish_connection(num_retries))
  {
    std::cout << "[ERROR] connect() failed after " << num_retries << " retries." << std::endl;
    return -1;
  }

  std::cout << "before start_streams()" << std::endl;
  etorque_reciever->start_streams(false);


  while (1)
  {
    sleep(1);
  }
  return 0;
}