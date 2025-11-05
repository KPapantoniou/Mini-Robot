
#include "robot_controller.h"
#include <cmath>
#include <opencv2/opencv.hpp>

RobotController::RobotController(double cmPerPixel, UdpClient* client)
    : cmPerPixelRatio(cmPerPixel), udpClient(client) {
    stopAcceptancePixels = 0.1/ cmPerPixelRatio;
    alignmentTolerance = 0.01 / cmPerPixelRatio;
}

char RobotController::calculateMovement(const cv::Point2f& frontPoint, const cv::Point2f& rearPoint,
                                        const cv::Point2f& target, cv::Mat& frame,
                                        double& debugAngle, double& debugDistance) {
    debugDistance = calculateDistance(frontPoint, target);
    if (debugDistance <= stopAcceptancePixels) {
        cv::putText(frame, "TARGET REACHED!", cv::Point(10, 80),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        return 'S';
    }
    return 'F';
}

double RobotController::calculateDistance(const cv::Point2f& p1, const cv::Point2f& p2) const {
    return cv::norm(p1 - p2);
}

double RobotController::calculateAngle(const cv::Point2f& from, const cv::Point2f& to) const {
    return atan2(to.y - from.y, to.x - from.x);
}

double RobotController::normalizeAngle(double angle) const {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}
