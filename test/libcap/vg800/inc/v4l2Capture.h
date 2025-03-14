#pragma once

/*******************************************************************************
 * FILENAME:      v4l2Capture.h
 *
 * AUTHORS:       Shen Qi
 *
 * START DATE:    Thursday January 28th 2021
 *
 * CONTACT:       qi.shen@smartmore.com
 *******************************************************************************/

#include <unistd.h>
#include <linux/videodev2.h>
#include <iostream>
#include <mutex>

#include "mem_manager.h"

#define BUFFER_NUM				32

#define LIGHT_BRIGHT_MAX		24
//#define DMA_M2M_ENABLE 	
#define LIGHT_NUM				4

struct sensorParam{
	unsigned int active; //参数生效标志
	unsigned int gain;
	unsigned int exposure;
	unsigned int focus;
	int focus_state;
	unsigned int lightBright[16];
};

class V4L2Capture
{
public:
    static V4L2Capture &getInstance()
    {
        static V4L2Capture instance;
        return instance;
    }

public:
    V4L2Capture(){};
    ~V4L2Capture(){};
    V4L2Capture(const V4L2Capture &) = delete;
    V4L2Capture &operator=(const V4L2Capture &) = delete;

public:
    int openDevice();
    int closeDevice();
    bool isOpening();
    int initDevice(int frameRate, int width, int height, int bit);
    int startCapture();
    int stopCapture();
    int freeBuffers();
    int getNewestFrame(void **frame_buf, size_t *len, int timeout, int runmode);
    struct sensorParam getCurrentFrameSensorParam();
    int getFrame(void **, size_t *, int timeout);
    int backFrame();
    struct timeval getFrameDonetime();

    // set gain and exposure should be done after start capture, otherwise failed
    int setGain(int value);
    int setExposure(int value);
    int getFrameRate();
    int setVerticalFlip(int value);
    int seHorizontalFlip(int value);
    int setCrop(int left, int top, int width, int height);
    int getWidth();
    int getHeight();
	int setStrobeEnable(int enable);
    int setLightBright(int num, int value);
	int setLightEnable(int enable)	;
	int setLightAim(int enable);
	int setLensFocus(int value);
	int getLensVaild(void);
	int getLensTemperature(void);
	int setLightMaxTime(int value);
	int getMaxExposure();
	int getMaxGain();
	int getMaxLensValue();

    int setIODebounceTime(int iTime);  //IO防抖时间设置
private:
    int getformat();
    int setFrameRateFormat(int frameRate, int width, int height, int bit);
    int initBuffers();

    struct cam_buffer
    {
        void *start;
        unsigned int length;
		unsigned long phy;
    };

    const char *devName = "/dev/video0";
    int mWidth = 1280;
    int mHeihgt = 800;
    struct timeval m_newframetime;
    const int capFMT8 = V4L2_PIX_FMT_GREY;
    const int capFMT10 = V4L2_PIX_FMT_Y10;
    int gain = 16;
    int exposure = 2233;
    int sWidth = 0xFFFF;
    int fdCam = -1;
    cam_buffer buffers[BUFFER_NUM];
	int memory_type = V4L2_MEMORY_USERPTR;    //V4L2_MEMORY_MMAP是使用之前需要20ms的复制方式;              V4L2_MEMORY_USERPTR;    是使用后面改的只需要1ms的复制方式
	struct v4l2_plane planes[3];
    int frameIndex = -1;	
	struct sm_mem_dev *memDev;
    int light_status = 0;
    struct sensorParam app_param,cur_param;
	int strobe_state = 0;
    std::mutex mtx;
};
