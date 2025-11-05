#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include "udp_client.h"

class RobotController {
protected:
    double cmPerPixelRatio;
    double stopAcceptancePixels;
    double alignmentTolerance;
    UdpClient* udpClient;

public:
    RobotController(double cmPerPixel, UdpClient* client);
    virtual ~RobotController() = default;

    virtual bool detectMarkers(const cv::Mat& hsv, cv::Point2f& frontPoint, cv::Point2f& rearPoint) = 0;
    virtual void drawMarkers(cv::Mat& frame, const cv::Point2f& frontPoint, const cv::Point2f& rearPoint) = 0;
    virtual std::string getControllerType() const = 0;
    
    virtual char calculateMovement(const cv::Point2f& frontPoint, const cv::Point2f& rearPoint,
                                    const cv::Point2f& target, cv::Mat& frame,
                                    double& debugAngle, double& debugDistance);

protected:
    double calculateDistance(const cv::Point2f& p1, const cv::Point2f& p2) const;
    double calculateAngle(const cv::Point2f& from, const cv::Point2f& to) const;
    double normalizeAngle(double angle) const;
};
