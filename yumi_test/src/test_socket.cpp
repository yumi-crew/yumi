#include <pthread.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <iostream>
#include <chrono>
#include <array>
#include <vector>
#include <thread>

using namespace std::chrono;
char data_str_l[100];
char data_str_r[100];
std::array<double, 7> data; 
bool connected = false;
bool stop_sign = false;

void parse(std::string data_l, std::string data_r, bool debug_print = false)
{
  size_t pos = 0;
  int i = 0;
  std::string token;

  // while ((pos = data_l.find(";")) != std::string::npos)
  // {
  //   token = data_l.substr(0, pos);
  //   data[i] = std::stof(token);
  //   data_l.erase(0, pos + 1);
  //   i++;
  // }
  // i++;

  std::cout << data_r << std::endl;
  pos = 0;
  for(int i = 0; i < 7; i++)
  {
    pos = data_r.find(";"); 
    if(pos == std::string::npos) break;

    token = data_r.substr(0, pos);
    data[i] = std::stod(token);
    data_r.erase(0, pos + 1);
  }


  i = 0;
  if(debug_print)
  {
    for(auto e : data)
    {
      std::cout << "joint " << i+1 << ": " << e << std::endl;
      i++;
    }
  }
}

int main()
{
  
  struct sockaddr_in servaddr_l, servaddr_r;

  //set up communication socket
  int comm_socket_l, comm_socket_r;
  comm_socket_l = socket(AF_INET, SOCK_STREAM, 0);
  comm_socket_r = socket(AF_INET, SOCK_STREAM, 0);

  servaddr_l.sin_family = AF_INET;
  servaddr_l.sin_port = htons(2021);
  servaddr_r.sin_family = AF_INET;
  servaddr_r.sin_port = htons(2020);

  inet_pton(AF_INET, "192.168.125.1", &(servaddr_l.sin_addr));
  inet_pton(AF_INET, "192.168.125.1", &(servaddr_r.sin_addr));

  if(connect(comm_socket_r, (struct sockaddr *)&servaddr_r, sizeof(servaddr_r)) == 0)
  {
    connected = true;
  }    
  std::cout << "connected : " << connected << std::endl;
 
 
  int counter_r = 0; int counter_l  = 0;
  auto start = high_resolution_clock::now();

  // Left socket thread
  // std::thread left_thread([&comm_socket_l, &counter_l]()
  // {
  //   while (!stop_sign && connected)
  //   {
  //     if(read(comm_socket_l, data_str_l, 100) > 0) 
  //     {
  //       counter_l++;
  //       //std::cout << "LEFT" << std::endl;
  //     }
  //   }
  // });

  // Right socket thread
  // std::thread right_thread([&comm_socket_r, &counter_r]()
  // {
  //   while (!stop_sign && connected)
  //   {

  //     if(read(comm_socket_r, data_str_r, 100) > 0) 
  //     {
  //       counter_r++;
  //       //std::cout << "RIGHT" << std::endl;
  //     }
  //   }
  // });

  // Parse and print
  while(!stop_sign)
  {
    read(comm_socket_r, data_str_r, 100);
    std::string temp = data_str_r;
    parse(data_str_l, temp, true);
    //std::cout << "left: " << counter_l << "right: " << counter_r << std::endl;
    // if(counter_l > 10000 && counter_r > 10000)
    // {
      // stop_sign = true;
      // //left_thread.join();
      // right_thread.join();
      // break;
    //}
  }

  auto stop = high_resolution_clock::now();
  auto duration = duration_cast<microseconds>(stop - start);
  double avg = duration.count() / 10000.0;
  std::cout << "Latency (milliseconds) of one message (average of 10000 transmissions): " << avg / 1000 << std::endl;

  return 0;
}