#include "micro_controller.h"
#include "robot_controller.h"
#include <opencv2/opencv.hpp>

using namespace std;

MicroController::MicroController(double cmPerPixel, UdpClient* client)
    : RobotController(cmPerPixel, client) {}

bool MicroController::detectMarkers(const cv::Mat& hsv, cv::Point2f& frontPoint, cv::Point2f& rearPoint) {
    cv::Mat maskFront;

   
    cv::inRange(hsv, cv::Scalar(5, 5, 50), cv::Scalar(40, 40, 235), maskFront);

    float radius;
    cv::Point2f frontCenter = findMarkerCenter(maskFront, 0.01 / cmPerPixelRatio, radius);
    std::vector<cv::Point> contour = getBestContour(maskFront, 0.01 / cmPerPixelRatio);
    cv::Point2f tip = findContourTip(contour);

    frontPoint = tip;


    cv::Point2f direction = frontCenter - tip;
    rearPoint = frontCenter + direction;

    return (frontPoint.x > 0 && frontPoint.y > 0);
}

void MicroController::drawMarkers(cv::Mat& frame, const cv::Point2f& frontPoint, const cv::Point2f& rearPoint) {
    cv::circle(frame, frontPoint, 3, cv::Scalar(0, 0, 255), -1); 
    cv::putText(frame, "Tip", frontPoint + cv::Point2f(10, -10),
                cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);

    cv::circle(frame, rearPoint, 3, cv::Scalar(255, 255, 0), -1); 
    cv::line(frame, rearPoint, frontPoint, cv::Scalar(0, 255, 0), 1);
}

char MicroController::calculateMovement(const cv::Point2f& frontPoint, const cv::Point2f& rearPoint,
                                        const cv::Point2f& target, cv::Mat& frame,
                                        double& debugAngle, double& debugDistance) {
    debugDistance = calculateDistance(frontPoint, target);
    debugAngle = 0;
    // Stop if the tip overlaps the target region in the mask
    double stop_accept = 0;
    cout<<frontPoint<<" Front"<<endl;
    if (frontPoint.y >= target.y- stop_accept ) {
        cv::putText(frame, "TARGET REACHED!", cv::Point(10, 80),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        return 'S';
    }

    

    if (frontPoint.x<target.x - alignmentTolerance) {
            cv::putText(frame, "Rotating Left (Micro)", cv::Point(10, 60),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
            return 'L';
    } else if(frontPoint.x > target.x + alignmentTolerance){
            cv::putText(frame, "Rotating Right (Micro)", cv::Point(10, 60),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
            return 'R';
    }
    else {
        cv::putText(frame, "Moving Forward (Micro)", cv::Point(10, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        return 'C';
    }
}

std::string MicroController::getControllerType() const {
    return "Micro Controller";
}

cv::Point2f MicroController::findMarkerCenter(const cv::Mat& mask, double minArea, float& out_radius) {
    vector<vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double maxArea = 0;
    cv::Point2f bestCenter(-1, -1);
    out_radius = 0;

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area >= minArea) {
            cv::Moments M = cv::moments(contour);
            if (M.m00 != 0) {
                cv::Point2f center(M.m10 / M.m00, M.m01 / M.m00);
                float radius;
                cv::minEnclosingCircle(contour, center, radius);
                if (area > maxArea) {
                    maxArea = area;
                    bestCenter = center;
                    out_radius = radius;
                }
            }
        }
    }
    return bestCenter;
}

std::vector<cv::Point> MicroController::getBestContour(const cv::Mat& mask, double minArea) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double maxArea = 0;
    std::vector<cv::Point> bestContour;

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area >= minArea && area > maxArea) {
            maxArea = area;
            bestContour = contour;
        }
    }
    return bestContour;
}

cv::Point2f MicroController::findContourTip(const std::vector<cv::Point>& contour) {
    if (contour.empty()) return cv::Point2f(-1, -1);

    cv::Point2f tip = contour[0];
    for (const auto& p : contour) {
        if (p.y > tip.y) {
            tip = p;
        }
    }
    return tip;
}
