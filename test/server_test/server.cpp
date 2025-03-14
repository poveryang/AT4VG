#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <vector>
#include <thread>

#include "cam_cap.h"
#include "at_interface.h"

static constexpr int SERVER_PORT = 8080;

bool ReceiveCommand(int client_socket, std::string &cmd) {
    // 简单协议：首先接收4字节命令长度，再接收命令字符串
    uint32_t cmd_len_network = 0;
    ssize_t ret = recv(client_socket, &cmd_len_network, sizeof(cmd_len_network), MSG_WAITALL);
    if (ret <= 0) return false;
    uint32_t cmd_len = ntohl(cmd_len_network);
    if (cmd_len == 0 || cmd_len > 1024) return false;

    cmd.resize(cmd_len);
    ret = recv(client_socket, &cmd[0], cmd_len, MSG_WAITALL);
    if (ret <= 0) return false;

    return true;
}

bool SendImage(const int client_socket, const cv::Mat &image) {
    std::vector<uchar> image_buffer;
    if (!cv::imencode(".png", image, image_buffer)) {
        std::cerr << "Error: Failed to encode image." << std::endl;
        return false;
    }

    const auto size = (uint32_t) image_buffer.size();
    const uint32_t size_network = htonl(size);
    if (send(client_socket, &size_network, sizeof(size_network), 0) <= 0) {
        std::cerr << "Error: Failed to send image size." << std::endl;
        return false;
    }

    size_t total_sent = 0;
    while (total_sent < image_buffer.size()) {
        const ssize_t sent = send(client_socket, image_buffer.data() + total_sent, image_buffer.size() - total_sent, 0);
        if (sent < 0) {
            std::cerr << "Error: Failed to send image data." << std::endl;
            return false;
        }
        total_sent += sent;
    }

    return true;
}

int main(int argc, char *argv[]) {
    const int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        std::cerr << "Error: cannot create socket" << std::endl;
        return -1;
    }

    constexpr int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(server_fd, (struct sockaddr *) &server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Error: cannot bind" << std::endl;
        close(server_fd);
        return -1;
    }

    if (listen(server_fd, 1) < 0) {
        std::cerr << "Error: listen failed" << std::endl;
        close(server_fd);
        return -1;
    }
    std::cout << "Server listening on port " << SERVER_PORT << std::endl;

    int client_socket = accept(server_fd, nullptr, nullptr);
    if (client_socket < 0) {
        std::cerr << "Error: accept failed" << std::endl;
        close(server_fd);
        return -1;
    }
    std::cout << "Client connected." << std::endl;

    CamCapture cam_capture;
    CameraParams cam_params;
    cam_params.exp_time = 200;
    cam_params.exp_gain = 100;
    cam_params.lights = {1, 1, 1, 1};
    cam_params.focus_pos = 30;
    cam_capture.SetCamParams(cam_params);
    std::cout << "Camera initialized." << std::endl;

    at::CamConf cam_conf = {1000, 800, {0,0,0,0},
        2000, 1, 0,
        20, 500, 1, 255, 1.0,
        0, 1, 1024,
        0, 1000, 100, {1, 1, 1, 1}, 30};
    at::ATInterface at_obj(cam_conf);

    while (true) {
        std::string cmd;
        if (!ReceiveCommand(client_socket, cmd)) {
            std::cerr << "Client disconnected or receive error." << std::endl;
            break;
        }
        std::cout << "Received command: " << cmd << std::endl;

        if (cmd == "GET_FRAME") {
            // Send image to client
            if (cv::Mat image = cam_capture.CapImg(); !SendImage(client_socket, image)) {
                std::cerr << "Failed to send image to client." << std::endl;
                break;
            }
        } else if (cmd.rfind("ADJUST_BRIGHTNESS:", 0) == 0) {
            // 命令格式: ADJUST_BRIGHTNESS:<target_brightness>
            std::string params_str = cmd.substr(strlen("ADJUST_BRIGHTNESS:"));
            const int target_brightness = std::stoi(params_str);
            at_obj.Init(target_brightness);
            bool adjust_done = false;
            while (!adjust_done) {
                cv::Mat image = cam_capture.CapImg();
                adjust_done = at_obj.RunWithTargetBrt(image);
                at::CamParams next_params = at_obj.GetNextParams();
                cam_params.exp_time = next_params.exp_time;
                cam_params.exp_gain = next_params.exp_gain;
                cam_capture.SetCamParams(cam_params);
            }
        } else {
            std::cerr << "Unknown command: " << cmd << std::endl;
        }
    }

    close(client_socket);
    close(server_fd);

    return 0;
}
