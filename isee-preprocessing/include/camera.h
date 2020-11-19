#ifndef CAMERA_HPP
#define CAMERA_HPP

#include "ros/ros.h"
#include <stdio.h>
#include <pthread.h>
#include <opencv2/opencv.hpp>   
#include "MvErrorDefine.h"
#include "CameraParams.h"
#include "MvCameraControl.h"  

namespace MVS
{
    #define MAX_IMAGE_DATA_SIZE (20 * 1024 * 1024)
    #define IMAGE_DATA_SIZE (10 * 1024 * 1024)
    #define IMAGE_SAVE_SIZE (10 * 1024 * 1024)

    enum CamerProperties
    {
        CAP_PROP_FRAMERATE_ENABLE,  //帧数可调
        CAP_PROP_FRAMERATE,         //帧数
        CAP_PROP_HEIGHT,            //图像高度
        CAP_PROP_WIDTH,             //图像宽度
        CAP_PROP_EXPOSURE_TIME,     //曝光时间
        CAP_PROP_GAMMA_ENABLE,      //伽马因子可调
        CAP_PROP_GAMMA,             //伽马因子
        CAP_PROP_GAINAUTO,              //亮度
        CAP_PROP_SATURATION_ENABLE, //饱和度可调
        CAP_PROP_SATURATION,        //饱和度
        CAP_PROP_OFFSETX,           //图像X方向偏置
        CAP_PROP_OFFSETY,           //图像Y方向偏置
        CAP_PROP_TRIGGER_MODE,
        CAP_PROP_TRIGGER_SOURCE,
        CAP_PROP_LINE_SELECTOR
    };

    class Camera
    {
    public:

        Camera(ros::NodeHandle &node);                         //构造函数
        ~Camera();                                             //析构函数
        static void *HKWorkThread(void *p_handle);             //原始信息转换线程
        bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo); //输出摄像头信息
        bool set(MVS::CamerProperties type, float value);      //设置摄像头参数
        bool reset();                                          //恢复默认参数
        void operator>>(cv::Mat &image);                       //读图

    private:
        void *handle;
        pthread_t nThreadID;
        int nRet;

        int width;
        int height;
        int OffsetX;
        int OffsetY;
        int FrameRateEnable;
        int FrameRate;
        int ExposureTime;
        int GammaEnable;
        float Gamma;
        int GainAuto;
        int SaturationEnable;
        int Saturation;
        int TriggerMode;
        int TriggerSource;
        int LineSelector;

    };
    
} // namespace MVS
#endif
