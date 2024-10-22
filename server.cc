#include <arpa/inet.h>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <unistd.h>
#include <iostream>


#define PORT 8080

int
main()
{
  int server_fd, new_socket;
  struct sockaddr_in address;
  int addrlen = sizeof(address);
  char buffer[1024] = {0};

  // 创建套接字
  if((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
  {
    perror("socket failed");
    exit(EXIT_FAILURE);
  }

  // 设置地址和端口
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(PORT);

  // 绑定套接字
  if(bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0)
  {
    perror("bind failed");
    exit(EXIT_FAILURE);
  }

  // 监听端口
  if(listen(server_fd, 3) < 0)
  {
    perror("listen");
    exit(EXIT_FAILURE);
  }

  std::cout << "Server listening on port " << PORT << std::endl;

  // 等待客户端连接
  if((new_socket =
          accept(server_fd, (struct sockaddr*)&address, (socklen_t*)&addrlen)) <
     0)
  {
    perror("accept");
    exit(EXIT_FAILURE);
  }

  // 发送请求到客户端（假设请求格式为脚本名及参数）
  std::string request =
      "/apollo/modules/tools/mock_routing/mock_routing_request.py";
  send(new_socket, request.c_str(), request.size(), 0);
  std::cout << "Request sent: " << request << std::endl;

  // 读取客户端返回的数据
  int valread = read(new_socket, buffer, 1024);
  if(valread > 0)
  {
    std::string response(buffer, valread);
    std::cout << "Client response: " << response << std::endl;
  }
  else
  {
    std::cerr << "Failed to receive response from client or response is empty."
              << std::endl;
  }

  // 关闭套接字
  close(new_socket);
  close(server_fd);

  return 0;
}
