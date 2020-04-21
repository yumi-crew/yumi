#include <e_torque_reciever/e_torque_reciever.hpp>


//auto etorque_recieve = std::make_shared<socket_interface::ETorqueReciever>("ETorqueReciever", 2020, 2021);

// Ctr+C handler
void signal_callback_handler(int signum)
{
  std::cout << "Caught signal " << signum << std::endl;
  // Stop stream
  //etorque_recieve->stop_stream();
  // Terminate ros node
  rclcpp::shutdown();
  // Terminate program
  exit(signum);
}


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // Ctrl+C handler
  signal(SIGINT, signal_callback_handler);

  socket_interface::ETorqueReciever etorque_recieve("ETorqueReciever", 2021, 2020);
  

  std::cout << "before establish_connection()" << std::endl;
  if(!etorque_recieve.establish_connection())
  {
    std::cout << "establish_connection() failed" << std::endl;
    return -1;
  }

  std::cout << "before recieve_stream() thread" << std::endl;
  // std::thread thread1([&etorque_recieve]()
  // {
  //   while(1) etorque_recieve.debug_print();
  // });
  etorque_recieve.recieve_stream();
  // std::thread([&etorque_recieve]()
  // {
  //   etorque_recieve.recieve_stream(true);
  // });
  // while(1) sleep(1);

  // std::string in;
  // std::cout << "press q to stop: ";
  // std::cin >> in;

  // etorque_recieve.stop_stream();
  // std::cout << "test completed" << std::endl;
  return 0;
}