#pragma once
#include "robot_controller.h"

class MacroController : public RobotController {
private:
    double angleTolerance;
    double minDistanceForRotation;

public:
    MacroController(double cmPerPixel, UdpClient* client);

    bool detectMarkers(const cv::Mat& hsv, cv::Point2f& frontPoint, cv::Point2f& rearPoint) override;
    void drawMarkers(cv::Mat& frame, const cv::Point2f& frontPoint, const cv::Point2f& rearPoint) override;
    char calculateMovement(const cv::Point2f& frontPoint, const cv::Point2f& rearPoint,
                           const cv::Point2f& target, cv::Mat& frame,
                           double& debugAngle, double& debugDistance) override;

    std::string getControllerType() const override;

private:
    cv::Point2f findMarkerCenter(const cv::Mat& mask, double minArea);
};
