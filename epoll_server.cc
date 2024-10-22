#include <iostream>
#include <sys/epoll.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <fcntl.h>
#include <string>

#define PORT 8080
#define MAX_EVENTS 10
#define BUFFER_SIZE 1024

void
setNonBlocking(int sock)
{
  int flags = fcntl(sock, F_GETFL, 0);
  fcntl(sock, F_SETFL, flags | O_NONBLOCK);
}

void
handleClient(int client_fd)
{
  char buffer[BUFFER_SIZE] = {0};
  int bytes_read = read(client_fd, buffer, sizeof(buffer) - 1);
  if(bytes_read > 0)
  {
    std::cout << "Received from client: " << buffer << std::endl;
  }
  else if(bytes_read == 0)
  {
    std::cout << "Client disconnected." << std::endl;
    close(client_fd);
  }
  else
  {
    perror("Read error");
  }
}

void
sendRequestToClient(int client_fd, const std::string& message)
{
  send(client_fd, message.c_str(), message.size(), 0);
}

int
main()
{
  int server_fd, epoll_fd;
  struct sockaddr_in address;
  struct epoll_event event, events[MAX_EVENTS];

  // 创建服务器套接字
  if((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
  {
    perror("Socket failed");
    exit(EXIT_FAILURE);
  }

  // 绑定地址和端口
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(PORT);

  if(bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0)
  {
    perror("Bind failed");
    close(server_fd);
    exit(EXIT_FAILURE);
  }

  // 监听端口
  if(listen(server_fd, 10) < 0)
  {
    perror("Listen failed");
    close(server_fd);
    exit(EXIT_FAILURE);
  }

  // 设置 epoll
  epoll_fd = epoll_create1(0);
  if(epoll_fd == -1)
  {
    perror("epoll_create1 failed");
    close(server_fd);
    exit(EXIT_FAILURE);
  }

  // 设置服务器套接字为非阻塞
  setNonBlocking(server_fd);
  event.events = EPOLLIN;
  event.data.fd = server_fd;
  epoll_ctl(epoll_fd, EPOLL_CTL_ADD, server_fd, &event);

  // 将标准输入加入到 epoll 监听
  int stdin_fd = fileno(stdin);
  setNonBlocking(stdin_fd);
  event.events = EPOLLIN;
  event.data.fd = stdin_fd;
  epoll_ctl(epoll_fd, EPOLL_CTL_ADD, stdin_fd, &event);

  std::vector<int> clients;
  std::cout << "Server listening on port " << PORT << std::endl;

  while(true)
  {
    int event_count = epoll_wait(epoll_fd, events, MAX_EVENTS, -1);
    for(int i = 0; i < event_count; ++i)
    {
      if(events[i].data.fd == server_fd)
      {
        // 接受新连接
        int client_fd;
        struct sockaddr_in client_address;
        socklen_t client_len = sizeof(client_address);
        if((client_fd = accept(server_fd,
                               (struct sockaddr*)&client_address,
                               &client_len)) >= 0)
        {
          setNonBlocking(client_fd);
          event.events = EPOLLIN | EPOLLET;
          event.data.fd = client_fd;
          epoll_ctl(epoll_fd, EPOLL_CTL_ADD, client_fd, &event);
          clients.push_back(client_fd);
          std::cout << "New client connected, FD: " << client_fd << std::endl;
        }
      }
      else if(events[i].data.fd == stdin_fd)
      {
        // 处理键盘输入
        std::string input;
        std::getline(std::cin, input);

        // 根据输入向所有客户端发送不同的消息
        for(int client_fd : clients)
        {
          if(input == "1")
          {
            sendRequestToClient(client_fd, "1");
          }
          else if(input == "2")
          {
            sendRequestToClient(client_fd, "2");
          }
          else if(input == "exit")
          {
            std::cout << "Exiting server..." << std::endl;
            close(server_fd);
            for(int fd : clients)
            {
              close(fd);
            }
            return 0;
          }
          else
          {
            sendRequestToClient(client_fd, "Unknown command");
          }
        }
      }
      else
      {
        // 处理客户端的消息
        handleClient(events[i].data.fd);
      }
    }
  }

  close(server_fd);
  return 0;
}
