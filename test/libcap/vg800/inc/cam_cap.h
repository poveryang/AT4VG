#ifndef CAM_CAP_H
#define CAM_CAP_H

#include <opencv2/opencv.hpp>

#ifdef USE_V4L2
#include "v4l2Capture.h"
#else
extern "C" {
#include "sm_video_capture.h"
}
#endif

struct SensorInfo {
    std::string name;
    int width;
    int height;
    int format;
    int fps;
};

struct CameraParams {
    int exp_time{};           // exposure time
    int exp_gain{};           // exposure gain
    std::vector<int> lights;  // light intensities
    int focus_pos{};          // focus position
};

class CamCapture {
public:
    CameraParams cur_params;

    std::string device_name;

    explicit CamCapture();

    ~CamCapture();

    void SetCamParams(CameraParams& cam_params);

    cv::Mat CapImg() const;

private:
#ifdef USE_V4L2
    V4L2Capture& vcap = V4L2Capture::getInstance();
#else
    int camera_number = -1;
#endif

    int sensor_width;
    int sensor_height;

    std::map<std::string, SensorInfo> sensor_map =
    {
#ifdef DEVICE_VS100P
        {"vs1000p_5mp", {"vs1000p_5mp", 2560, 1936, 8, 60}},
        {"vs1000p_2mp", {"vs1000p_2mp", 1920, 1200, 8, 60}},
#elif DEVICE_VN4000CL
        {"vn4000cl", {"ap1302", 1408, 1024, SM_FMT_YUYV, 0}},
#elif DEVICE_VS800P
        {"vs800p", {"sc132gs", 1088, 1288, DATA_FMT_RAW8, 0}},
        {"med200s", {"ar0234", 1920, 1200, DATA_FMT_RAW8, 0}}
#elif DEVICE_FST100
        {"fst100", {"sc132gs", 640, 480, DATA_FMT_RAW8, 0}}
#elif DEVICE_VG800
        {"vg800", {"ov9281", 1000, 800, 8, 60}}
#endif
    };
};

#endif //CAM_CAP_H
