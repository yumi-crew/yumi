#include <e_torque_reciever/e_torque_reciever.hpp>


namespace socket_interface
{
  
ETorqueReciever::ETorqueReciever(std::string node_name, uint port_left, uint port_right)
:
connected_{false}, stop_sign_{false}
{
  socket_left_.comm_socket = socket(AF_INET, SOCK_STREAM, 0);
  socket_left_.servaddr.sin_family = AF_INET;
  socket_left_.servaddr.sin_port = htons(port_left);
  inet_pton(AF_INET, "192.168.125.1", &(socket_left_.servaddr.sin_addr));
  socket_left_.connected = false,

  socket_right_.comm_socket = socket(AF_INET, SOCK_STREAM, 0);
  socket_right_.servaddr.sin_family = AF_INET;
  socket_right_.servaddr.sin_port = htons(port_right);
  inet_pton(AF_INET, "192.168.125.1", &(socket_right_.servaddr.sin_addr));
  socket_right_.connected = true;

  node_ = std::make_shared<rclcpp::Node>(node_name);
}


bool ETorqueReciever::establish_connection(int retries)
{
  if((connect(socket_right_.comm_socket,(struct sockaddr*)&(socket_right_.servaddr),sizeof(socket_right_.servaddr))==0) 
      && 
    (connect(socket_left_.comm_socket,(struct sockaddr*)&(socket_left_.servaddr),sizeof(socket_left_.servaddr))==0))
  {
    socket_left_.connected = true;
    socket_right_.connected = true;
    connected_ = true;
  }

  int retries_left = retries;
  while(!connected_) 
  {
    if(retries_left)
    {
      if((connect(socket_left_.comm_socket,(struct sockaddr*)&(socket_left_.servaddr),sizeof(socket_left_.servaddr))==0) 
          && 
         (connect(socket_right_.comm_socket,(struct sockaddr*)&(socket_right_.servaddr),sizeof(socket_right_.servaddr))==0))
      {
        socket_left_.connected = true;
        socket_right_.connected = true;
        connected_ = true;
      }
    }
    else
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Unable to establish socket connection with YuMi." 
      << "\nleft_socket connected: " << std::boolalpha << socket_left_.connected 
      << "  right_socket connected: " << std::boolalpha << socket_left_.connected);
      return false;
    }
    
  }
  return true;
}

void ETorqueReciever::recieve_stream(bool debug)
{
  std::cout << " ** entering recieve_stream()" << std::endl;
  char buffer_l[100];
  char buffer_r[100];

   
  std::cout << " ** before spinning out right thread" << std::endl;
  // Right socket thread
  std::thread right_thread([this, &buffer_r]()
  {
    int ret_r;
    while (!stop_sign_ && connected_)
    {
      thread_right_mutex_.lock();
      bzero(buffer_r, sizeof(buffer_r));
      ret_r = recv(socket_right_.comm_socket, buffer_r, sizeof(buffer_r), 0);
      thread_right_mutex_.unlock();
      if(ret_r >= 0) 
      {
        socket_right_.subsequent_read_fails_counter = 0;
      }
      else
      {
        if(socket_right_.subsequent_read_fails_counter < 5)
        {
          socket_right_.subsequent_read_fails_counter++;
          continue;
        }
        else 
        {
          RCLCPP_ERROR_STREAM(node_->get_logger(), "read (right) failed");
          break;
        }
      }
    }
  });


  std::cout << " ** before spinning out left thread" << std::endl;
  // Left socket thread
  std::thread left_thread([this, &buffer_l]()
  {
    int ret_l;
    while (!stop_sign_ && connected_)
    {
      thread_left_mutex_.lock();
      bzero(buffer_l, sizeof(buffer_l));
      ret_l = recv(socket_left_.comm_socket, buffer_l, sizeof(buffer_l), 0);
      thread_left_mutex_.unlock();
      if(ret_l >= 0) 
      {
        socket_left_.subsequent_read_fails_counter = 0;
      }
      else
      {
        if(socket_left_.subsequent_read_fails_counter < 5)
        {
          socket_left_.subsequent_read_fails_counter++;
          continue;
        }
        else 
        {
          RCLCPP_ERROR_STREAM(node_->get_logger(), "read (left) failed");
          break;
        }
      }
    }
  });

  std::cout << " ** before while (parsing)" << std::endl;
  // Parse and print
  while(!stop_sign_ && connected_)
  {
    thread_left_mutex_.lock();
    thread_right_mutex_.lock();
    //parse(buffer_l, buffer_r, debug);    
    thread_left_mutex_.unlock();
    thread_right_mutex_.unlock();
    if(stop_sign_) break;
  }

  left_thread.join();
  right_thread.join();
  RCLCPP_INFO_STREAM(node_->get_logger(), "Threads stopped successfully");
}


void ETorqueReciever::parse(std::string data_left, std::string data_right, bool debug)
{
  size_t pos = 0;
  int i = 0;
  std::string token;

  // Parsing the first part of data_left, containing the e_torques of the left arm
  while ((pos = data_left.find(";")) != std::string::npos)
  {
    token = data_left.substr(0, pos);
    e_torques_[i] = std::stod(token);
    data_left.erase(0, pos + 1);
    i++;
  }
  
  // Parsing the first part of data_right, containing the e_torques of the right arm
  i++; pos = 0;
  while ((pos = data_right.find(";")) != std::string::npos)
  {
    token = data_right.substr(0, pos);
    e_torques_[i] = std::stod(token);
    data_right.erase(0, pos + 1);
    i++;
  }

  if(debug) debug_print();
}

void ETorqueReciever::debug_print()
{
  std::cout << "{" << std::endl;
  std::cout << "  Left arm" << std::endl;
  for(int j = 0; j<7; j++) 
  {
    if(j == 2) std::cout << "\tjoint 7: " << e_torques_[j] << std::endl;
    else if(j > 2)  std::cout << "\tjoint " << j << ": " << e_torques_[j] << std::endl;
    else std::cout << "\tjoint " << j+1 << ": " << e_torques_[j] << std::endl;
  }

  std::cout << "  Right arm" << std::endl;
  for(int j = 7; j<14; j++) 
  {
    if(j == 9) std::cout << "\tjoint 7: " << e_torques_[j] << std::endl;
    else if(j > 9)  std::cout << "\tjoint " << j-7 << ": " << e_torques_[j] << std::endl;
    else std::cout << "\tjoint " << j-7+1 << ": " << e_torques_[j] << std::endl;
  }
  std::cout << "}" << std::endl;
}





} // end namespace socket_interface