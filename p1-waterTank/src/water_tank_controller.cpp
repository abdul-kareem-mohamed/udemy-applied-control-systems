#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <algorithm>
#include <string>
#include <csignal>

#include "pid_controller.hpp"
#include <spdlog/spdlog.h>
#include <fmt/core.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

volatile sig_atomic_t keep_running = 1;
int server_fd = -1;


void sigint_handler(int signum)
{
  keep_running = 0;
  if (server_fd != -1) {
    close(server_fd);
  }
}


double PID_Controller(double value, double target=0.0, double kp=1000, double ki=0.0, double kd=0.0) 
{
  double error = target - value;
  double pid = std::max(0.0, kp * error);
  return pid;
}


int main() {
  spdlog::set_level(spdlog::level::debug); 

  // Register signal handler for KeyboardInterrupt (Ctrl+C)
  std::signal(SIGINT, sigint_handler);

  const std::string HOST_CTRL = "127.0.0.1";
  const int PORT_CTRL = 5000;

  int server_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd < 0) { spdlog::error("Failed to create socket"); return 1;}

  int opt = 1;
  setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  sockaddr_in server_addr{};
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(PORT_CTRL);
  inet_pton(AF_INET, HOST_CTRL.c_str(), &server_addr.sin_addr);

  // Bind and Listen
  if (bind(server_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
  {
    spdlog::error("Bind failed");
    close(server_fd);
    return  1;
  }

  if (listen(server_fd, 1) < 0)
  {
    spdlog::error("Listen failed");
    close(server_fd);
    return  1;    
  } 

  spdlog::info("Controller running...");

  while (keep_running) {
    // Accept connection
    sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);

    int client_fd = accept(server_fd, (struct sockaddr *)&client_addr, (socklen_t *)&client_len);
    
    if (client_fd < 0) {
      spdlog::error("Accept failed");
      continue;
    }

    std::string buffer = "";
    char read_buf[1024];

    while(keep_running) {
      ssize_t bytes_read = recv(client_fd, read_buf, sizeof(read_buf)-1, 0);
      if (bytes_read <= 0) {
        if (bytes_read == 0) {
          spdlog::error("No data received");
        } else {
          spdlog::error("Connection error");
        }
        break;
      }

      read_buf[bytes_read] = '\0';
      buffer += read_buf;
      std::string msg = fmt::format("Raw Data: {}", buffer);
      spdlog::debug(msg);
      
      size_t pos;
      while ((pos = buffer.find('\n')) != std::string::npos) {
        std::string line = buffer.substr(0, pos);
        buffer.erase(0, pos + 1);
        std::string msg = fmt::format("Extracted Data: {}", line);
        spdlog::debug(msg);

        try {
          json msg = json::parse(line);

          double target = msg.value("target", 1.0);
          double volume = msg.value("volume", 0.0);
          
          double m_dot = PID_Controller(volume, target);

          json reply;
          reply["m_dot"] = m_dot;
          std::string reply_str = reply.dump() + "\n";

          send(client_fd, reply_str.c_str(), reply_str.length(), 0);

          spdlog::debug("Sent: m_dot: {:.2f}", m_dot);

        } catch (const json::exception& e) {
          spdlog::error("Invalid JSON: {}", line); 
        }
      }
    }
    close(client_fd);
  }
}