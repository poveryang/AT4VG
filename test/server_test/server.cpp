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

    CamCapture cam;
    CamParams cam_params;
    cam_params.exp_time = 1000;
    cam_params.exp_gain = 100;
    cam_params.lights = {1, 1, 1, 1};
    cam.SetCamParams(cam_params);
    std::cout << "Camera initialized." << std::endl;


    // 1. Create an AutoExposure object
    int min_exp_time = 20;
    int max_exp_time = sm_get_max_expt();
    int min_gain = 1;
    int max_gain = sm_get_max_gain();
    AutoExposure auto_exposure(min_exp_time, max_exp_time, min_gain, max_gain);
    std::cout << "Auto exposure initialized." << std::endl;


    // 示例逻辑：根据客户端的命令做出相应
    // 协议假定：
    // "GET_FRAME" 请求返回一帧图像
    while (true) {
        std::string cmd;
        if (!ReceiveCommand(client_socket, cmd)) {
            std::cerr << "Client disconnected or receive error." << std::endl;
            break;
        }
        std::cout << "Received command: " << cmd << std::endl;

        if (cmd == "GET_FRAME") {
            // // 采图前打开灯光
            // CamParams params;
            // params.lights = {1,1,1,1}; // 假设有4路灯光，如果是单路就 {1}
            // cam.SetCamParams(params);

            // 采图并发送
            cv::Mat image = cam.CapImg();
            if (!SendImage(client_socket, image)) {
                std::cerr << "Failed to send image to client." << std::endl;
                break;
            }

            // // 采图完成后关灯
            // params.lights = {0,0,0,0}; // 假设有4路灯光，如果是单路就 {0}
            // cam.SetCamParams(params);
        } else if (cmd.rfind("ADJUST_EXPOSURE:", 0) == 0) {
            // 命令格式: ADJUST_EXPOSURE:<limited_exp_time>:<target_brightness>
            std::string params_str = cmd.substr(strlen("ADJUST_EXPOSURE:"));
            // 分割这两个参数
            if (size_t colon_pos = params_str.find(':'); colon_pos != std::string::npos) {
                const int limited_exp_time = std::stoi(params_str.substr(0, colon_pos));
                const int target_brightness = std::stoi(params_str.substr(colon_pos + 1));
                std::cout << "Limited exposure time: " << limited_exp_time << ", target brightness: " << target_brightness << std::endl;


                // 2. Initialize the auto exposure adjustment
                int cur_gain = sm_get_cur_gain();
                auto_exposure.Init(cur_gain, target_brightness, limited_exp_time);

                // 3. Set the initial exposure parameters
                int exp_time, exp_gain;
                auto_exposure.GetExposureParams(exp_time, exp_gain);
                cam_params.exp_time = exp_time;
                cam_params.exp_gain = exp_gain;
                cam_params.lights = {1, 1, 1, 1};
                cam.SetCamParams(cam_params);

                // 4. Adjust exposure to target brightness
                cv::Mat image;
                bool adjust_done = false;
                while (!adjust_done) {
                    // Note: Drop the first two images
                    for (int i = 0; i <= 2; ++i) {
                        image = cam.CapImg();
                        double brightness = AutoExposure::CalcImageBrightness(image);
                        std::cout << "Img_" << i + 1 << " brt: " << brightness  << std::endl;
                    }

                    // 4.1. Adjust exposure parameters
                    adjust_done = auto_exposure.AdjustExposure(image);

                    // 4.2. Get the exposure parameters and set them to the camera
                    auto_exposure.GetExposureParams(exp_time, exp_gain);

                    cam_params.exp_time = exp_time;
                    cam_params.exp_gain = exp_gain;
                    cam_params.lights = {1, 1, 1, 1};
                    cam.SetCamParams(cam_params);
                }

                // 5. Get the final exposure parameters until the adjustment is done
                int final_exp_time, final_exp_gain;
                auto_exposure.GetExposureParams(final_exp_time, final_exp_gain);
                std::cout << "Final exposure time: " << final_exp_time << ", final gain: " << final_exp_gain << std::endl;

                cam_params.exp_time = final_exp_time;
                cam_params.exp_gain = final_exp_gain;
                cam_params.lights = {1, 1, 1, 1};
                cam.SetCamParams(cam_params);

                cv::Mat final_image = cam.CapImg();
                cv::imwrite("final_image.png", final_image);
            }
        } else {
            // 未知命令忽略或打印
            std::cerr << "Unknown command: " << cmd << std::endl;
        }
    }

    close(client_socket);
    close(server_fd);

    return 0;
}
