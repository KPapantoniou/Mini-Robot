#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <fstream>
#include <limits>
#include <filesystem>
#include <memory>

#include "camera_calibrator.h"
#include "udp_client.h"
#include "macro_controller.h"
#include "micro_controller.h"

namespace fs = std::filesystem;
using namespace std;

cv::Point2f userTarget(-1, -1);
bool targetSelected = false;

void onMouse(int event, int x, int y, int, void*) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        userTarget = cv::Point2f(x, y);
        targetSelected = true;
        std::cout << "Target selected at: (" << x << ", " << y << ")" << std::endl;
    }
}

bool initializeVideoRecording(cv::VideoCapture& cap, cv::VideoWriter& writer) {
    char response;
    std::cout << "Do you want to record the session? (y/n): ";
    std::cin >> response;

    if (response != 'y' && response != 'Y') {
        return false;
    }

    std::string videoFilename;
    std::cout << "Enter a name for the video file (without extension): ";
    std::cin >> videoFilename;

    fs::path mainFolder = "fvideo";
    fs::path sessionFolder = mainFolder / videoFilename;

    try {
        fs::create_directories(sessionFolder);
    } catch (const std::exception& e) {
        std::cerr << "Error creating folders: " << e.what() << std::endl;
        return false;
    }

    fs::path fullPath = sessionFolder / (videoFilename + ".avi");

    int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    double fps = 20.0;
    cv::Size frameSize(
        static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH)),
        static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT))
    );

    writer.open(fullPath.string(), codec, fps, frameSize, true);
    if (!writer.isOpened()) {
        std::cerr << "Error: Could not open video file for writing: " << fullPath << std::endl;
        return false;
    }

    std::cout << "Recording video to: " << fullPath << std::endl;
    return true;
}

int main(int argc, char* argv[]) {
    double cmPerPixelRatio = 0.0;
    const string calibrationFilename = "calibration_data.txt";
    bool performCalibration = false;
    bool useMacroController = true; // Default to macro

    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--calibrate") {
            performCalibration = true;
        } else if (std::string(argv[i]) == "--micro") {
            useMacroController = false;
            cout << "Using Micro Controller" << endl;
        } else if (std::string(argv[i]) == "--macro") {
            useMacroController = true;
            cout << "Using Macro Controller" << endl;
        }
    }

    CameraCalibrator calibrator(2);

    if (performCalibration){
        cout << "Forcing camera calibration..."<<endl;
        calibrator.runCalibration();
        if(calibrator.getCmPerPixelRatio()>0){
            calibrator.saveCalibration(calibrationFilename);
            cmPerPixelRatio = calibrator.getCmPerPixelRatio();
        } else {
            cerr<<"Calibration failed or was interrupted. Exiting"<<endl;
            return -1;
        }
        try {
            cv::destroyWindow("Camera Feed - Click Two Points on Ruler");
        } catch (...) {
            cerr << "Warning: Tried to destroy a window that wasn't created." << endl;
        }
    } else {
        if(!calibrator.loadCalibration(calibrationFilename)){
            cout <<"Calibration data missing. Initiating Calibration..."<<endl;
            calibrator.runCalibration();
            if (calibrator.getCmPerPixelRatio() > 0) {
                calibrator.saveCalibration(calibrationFilename);
                cmPerPixelRatio = calibrator.getCmPerPixelRatio();
            } else {
                cerr << "Calibration failed or was interrupted. Exiting." << endl;
                return -1;
            }
        } else {
            cmPerPixelRatio = calibrator.getCmPerPixelRatio();
        }
    }

    if(cmPerPixelRatio<=0){
        cerr<<"Invalid pixel to cm ratio"<<endl;
        return -1;
    }
    cout << "Using Centimeter per Pixel Ratio: "<<cmPerPixelRatio<<" cm/pixel"<<endl;

    string esp_ip ="192.168.234.188";
    int esp_port = 1234;
    UdpClient udpClient(esp_ip, esp_port);

    if (!udpClient.isInitialized()) {
        cerr << "Failed to initialize UDP Client. Exiting." << endl;
        return -1;
    }

    // Create appropriate controller based on command line argument
    std::unique_ptr<RobotController> controller;
    if (useMacroController) {
        controller = std::make_unique<MacroController>(cmPerPixelRatio, &udpClient);
    } else {
        controller = std::make_unique<MicroController>(cmPerPixelRatio, &udpClient);
    }

    cv::VideoCapture cap(2);
    if (!cap.isOpened()) {
        cerr << "Error: Could not open the camera. " << endl;
        return -1;
    }
    cout << "Camera opened successfully!" << endl;

    cv::VideoWriter videoWriter;
    bool recordVideo = initializeVideoRecording(cap, videoWriter);

    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    ofstream logFile("robot_path.csv");
    logFile << "time,controller_type,rear_x,rear_y,front_x,front_y,target_x,target_y,debug_angle,debug_distance,command\n";

    int frame_count = 0;
    cv::namedWindow("Robot View", cv::WINDOW_AUTOSIZE);
    cv::setWindowProperty("Robot View", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    cv::setMouseCallback("Robot View", onMouse, nullptr);
    auto startTime = std::chrono::steady_clock::now();
    while(true){
        cv::Mat frame, blurred, hsv;
        
        try{
            if (!cap.read(frame)) { 
                cerr << "Warning: Could not read frame from camera. Retrying..." << endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
                if (!cap.read(frame)) { 
                    cerr << "Error: Failed to read frame after retry. Camera might be disconnected. Exiting." << endl;
                    break; 
                }
            }
            
            if (frame.empty() || frame.cols == 0 || frame.rows == 0 || frame.channels() == 0) {
                cerr << "Warning: Captured frame is empty or invalid. Skipping frame." << endl;
                continue; 
            }
            
            int key = cv::waitKey(30);
            if (key == 27) { // ESC key
                udpClient.sendCommand('S'); 
                break;
            }

            cv::GaussianBlur(frame, blurred, cv::Size(5,5), 0);
            cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

            cv::Point2f frontPoint, rearPoint;
            bool markersDetected = controller->detectMarkers(hsv, frontPoint, rearPoint);
            
            if ( markersDetected && targetSelected) {
                // frame_count++;
                auto currentTime = std::chrono::steady_clock::now();
                std::chrono::duration<double> elapsedSeconds = currentTime - startTime;
                double timeInSeconds = elapsedSeconds.count();
                
                //for detecting HSV of Target
                /*
                if (userTarget.x >= 0 && userTarget.y >= 0 &&
                userTarget.x < hsv.cols && userTarget.y < hsv.rows) {

                // Get HSV color at the target point
                cv::Vec3b hsvValue = hsv.at<cv::Vec3b>(userTarget);
                int h = hsvValue[0];
                int s = hsvValue[1];
                int v = hsvValue[2];

                // Show on screen
                std::string hsvText = "HSV: (" + std::to_string(h) + "," +
                                    std::to_string(s) + "," + std::to_string(v) + ")";
                cv::putText(frame, hsvText, userTarget + cv::Point2f(10, 20),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);

                // Optional: print to console or log
                std::cout << "Target HSV: " << hsvText << std::endl;
                }
                */
                // Draw markers
                controller->drawMarkers(frame, frontPoint, rearPoint);
                
                // Draw target
                cv::circle(frame, userTarget, 1, cv::Scalar(255, 0, 255), -1);
                cv::line(frame, frontPoint, userTarget, cv::Scalar(255, 255, 0), 2);
                cv::putText(frame, "Target", userTarget + cv::Point2f(10, -10), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 255), 1);
                
                // Calculate movement
                double debugAngle, debugDistance;
                char command = controller->calculateMovement(frontPoint, rearPoint, userTarget, frame, debugAngle, debugDistance);
                
                // Send command
                if (command == 'S') {
                    for (int i = 0; i < 10; i++){
                        udpClient.sendCommand('S'); 
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    }
                    cout << "Reached the Target!" << endl;
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    break;
                } else {
                    udpClient.sendCommand(command);
                }
                
                // Display debug information
                cv::putText(frame, controller->getControllerType(), cv::Point(10, 140), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
                cv::putText(frame, "Debug Value: " + to_string(debugAngle*57.2957795) + " deg",
                            cv::Point(10, 100), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
                cv::putText(frame, "Distance: " + to_string(static_cast<float>(debugDistance*cmPerPixelRatio)) + " cm",
                            cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
                
                // Log data
                logFile << timeInSeconds << "," << controller->getControllerType() << ","
                        << rearPoint.x*cmPerPixelRatio << "," << rearPoint.y*cmPerPixelRatio << ","
                        << frontPoint.x*cmPerPixelRatio << "," << frontPoint.y*cmPerPixelRatio << ","
                        << userTarget.x*cmPerPixelRatio << "," << userTarget.y*cmPerPixelRatio << ","
                        << debugAngle << "," << debugDistance*cmPerPixelRatio*10 << "," << command << "\n";
                        
            } else {
                udpClient.sendCommand('S');
                if (!markersDetected) {
                    cv::putText(frame, "Markers Not Found (Stopping)", cv::Point(10, 30), 
                               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
                }
                if (!targetSelected) {
                    cv::putText(frame, "Click to select target", cv::Point(10, 50), 
                               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
                }
                cv::putText(frame, controller->getControllerType() + " (Waiting)", cv::Point(10, 70), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
            }
            
        } catch (const cv::Exception& e){
            cerr << "OpenCV Exception caught: " << e.what() << endl;
            udpClient.sendCommand('S'); 
            break; 
        } catch (const std::exception& e) {
            cerr << "Standard Exception caught: " << e.what() << endl;
            udpClient.sendCommand('S'); 
            break; 
        } catch (...) {
            cerr << "Unknown Exception caught." << endl;
            udpClient.sendCommand('S'); 
            break; 
        }
        
        if (recordVideo) {
            videoWriter.write(frame);
        }

        cv::imshow("Robot View", frame);
    }

    logFile.close();
    cap.release();
    if (recordVideo) {
        videoWriter.release();
        std::cout << "Video saved successfully." << std::endl;
    }

    return 0;
}