#include "macro_controller.h"
#include <opencv2/opencv.hpp>


MacroController::MacroController(double cmPerPixel, UdpClient* client)
    : RobotController(cmPerPixel, client) {
    angleTolerance = 2.0 * M_PI / 180.0;
    minDistanceForRotation = 1.0;
}

bool MacroController::detectMarkers(const cv::Mat& hsv, cv::Point2f& frontPoint, cv::Point2f& rearPoint) {
    cv::Mat maskRear, maskFront;
    cv::inRange(hsv, cv::Scalar(20,130,150), cv::Scalar(80,255,255), maskRear);
    cv::inRange(hsv, cv::Scalar(60, 20, 100), cv::Scalar(100, 50, 255),maskFront);
    rearPoint = findMarkerCenter(maskRear, 0.45 / cmPerPixelRatio);
    frontPoint = findMarkerCenter(maskFront, 0.6 / cmPerPixelRatio);
    return (rearPoint.x > 0 && frontPoint.x > 0);
}

void MacroController::drawMarkers(cv::Mat& frame, const cv::Point2f& frontPoint, const cv::Point2f& rearPoint) {
    cv::circle(frame, rearPoint, 5, cv::Scalar(0, 255, 255), -1);
    cv::circle(frame, frontPoint, 5, cv::Scalar(255, 255, 0), -1);
    cv::line(frame, rearPoint, frontPoint, cv::Scalar(0, 255, 0), 2);
}

char MacroController::calculateMovement(const cv::Point2f& frontPoint, const cv::Point2f& rearPoint,
                                        const cv::Point2f& target, cv::Mat& frame,
                                        double& debugAngle, double& debugDistance) {
    debugDistance = calculateDistance(frontPoint, target);
    if (debugDistance <= stopAcceptancePixels) {
    // if(frontPoint.y>=target.y){
        cv::putText(frame, "TARGET REACHED!", cv::Point(10, 80),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        return 'S';
    }

    cv::Point2f robotDirection = frontPoint - rearPoint;
    cv::Point2f targetDirection = target - frontPoint;

    double robotAngle = atan2(robotDirection.y, robotDirection.x);
    double targetAngle = atan2(targetDirection.y, targetDirection.x);
    debugAngle = normalizeAngle(targetAngle - robotAngle);

    if (debugDistance > minDistanceForRotation && std::abs(debugAngle) > angleTolerance) {
        return debugAngle > 0 ? 'R' : 'L';
    }
    return 'C';
}

std::string MacroController::getControllerType() const {
    return "Macro Controller";
}

cv::Point2f MacroController::findMarkerCenter(const cv::Mat& mask, double minArea) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::Point2f bestCenter(-1, -1);
    double maxArea = 0;
    for (const auto& c : contours) {
        double area = cv::contourArea(c);
        if (area >= minArea) {
            auto M = cv::moments(c);
            if (M.m00 != 0) {
                cv::Point2f center(M.m10 / M.m00, M.m01 / M.m00);
                if (area > maxArea) {
                    bestCenter = center;
                    maxArea = area;
                }
            }
        }
    }
    return bestCenter;
}
