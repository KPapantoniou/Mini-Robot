#ifndef CAMERA_CALIBRATOR_H
#define CAMERA_CALIBRATOR_H

#include <opencv2/opencv.hpp> 
#include <vector>             
#include <string>             
#include <fstream>

class CameraCalibrator;


static CameraCalibrator* s_currentCalibratorInstance = nullptr;


void staticMouseCallback(int event, int x, int y, int flags, void* userdata);

class CameraCalibrator {
public:
    CameraCalibrator(int cameraIndex = 0, const std::string& windowName = "Camera Feed - Click Two Points on Ruler");

    ~CameraCalibrator();

    void runCalibration();

    double getPixelPerCmRatio() const { return m_pixelPerCmRatio; }

    double getCmPerPixelRatio() const { return m_cmPerPixelRatio; }

    bool saveCalibration(const std::string& filename) const;

    bool loadCalibration(const std::string& filename);

private:
    cv::VideoCapture m_cap;                 
    std::string m_windowName;              
    std::vector<cv::Point> m_points;        
    cv::Mat m_currentFrame;                
    double m_pixelPerCmRatio;               
    double m_cmPerPixelRatio;               

     void handleMouseClick(int event, int x, int y, int flags);

     friend void staticMouseCallback(int event, int x, int y, int flags, void* userdata);
};

#endif 
