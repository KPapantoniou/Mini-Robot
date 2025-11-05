#include "camera_calibrator.h" 
#include <iostream>           
#include <cmath>              

void staticMouseCallback(int event, int x, int y, int flags, void* userdata) {
    if (s_currentCalibratorInstance) {
        s_currentCalibratorInstance->handleMouseClick(event, x, y, flags);
    }
}


CameraCalibrator::CameraCalibrator(int cameraIndex, const std::string& windowName)
    : m_cap(cameraIndex), m_windowName(windowName),
      m_pixelPerCmRatio(0.0), m_cmPerPixelRatio(0.0) {
    
    if (!m_cap.isOpened()) {
        std::cerr << "Error: Could not open camera at index " << cameraIndex << std::endl;
        
    }
}


CameraCalibrator::~CameraCalibrator() {
 
    if (m_cap.isOpened()) {
        m_cap.release();
    }
 
    cv::destroyWindow(m_windowName);
}


void CameraCalibrator::handleMouseClick(int event, int x, int y, int flags) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        m_points.push_back(cv::Point(x, y));
        std::cout << "Point selected: (" << x << ", " << y << ")" << std::endl;

        if (m_points.size() == 2) {
            double pixelDistance = std::sqrt(
                std::pow(m_points[1].x - m_points[0].x, 2) +
                std::pow(m_points[1].y - m_points[0].y, 2)
            );

            std::cout << "\nTwo points selected. Pixel distance: " << pixelDistance << " pixels." << std::endl;
            std::cout << "Please enter the actual real-world distance (in centimeters) between these two points on your ruler: ";

            double cmDistance;
            std::cin >> cmDistance;

            if (std::cin.fail() || cmDistance <= 0) {
                std::cout << "Invalid input. Please enter a positive number for the distance." << std::endl;
                std::cin.clear(); 
                
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                m_points.clear(); 
                return;
            }

            m_pixelPerCmRatio = pixelDistance / cmDistance;
            m_cmPerPixelRatio = cmDistance / pixelDistance;

            std::cout << "\n--- Calculation Result ---" << std::endl;
            std::cout << "Pixel per Centimeter Ratio: " << m_pixelPerCmRatio << " pixels/cm" << std::endl;
            std::cout << "Centimeter per Pixel Ratio: " << m_cmPerPixelRatio << " cm/pixel" << std::endl;
            std::cout << "--------------------------" << std::endl;

            m_points.clear(); 
            std::cout << "\nClick two new points to measure again, or press 'q' to quit." << std::endl;
        }
    }
}


void CameraCalibrator::runCalibration() {
    if (!m_cap.isOpened()) {
        std::cerr << "Cannot run calibration: Camera not opened." << std::endl;
        return;
    }

    
    s_currentCalibratorInstance = this;

    
    cv::namedWindow(m_windowName, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(m_windowName, staticMouseCallback, NULL);

    std::cout << "Place a ruler in front of the camera." << std::endl;
    std::cout << "Click on two distinct points on the ruler in the camera feed." << std::endl;
    std::cout << "For example, click on the 0 cm mark and then the 10 cm mark." << std::endl;
    std::cout << "After clicking two points, enter the actual distance in centimeters in the console." << std::endl;
    std::cout << "Press 'q' or 'ESC' to quit the program." << std::endl;

    while (true) {
        m_cap >> m_currentFrame;

        if (m_currentFrame.empty()) {
            std::cerr << "Error: Could not read frame from camera." << std::endl;
            break;
        }

        cv::Mat displayFrame = m_currentFrame.clone(); 

        
        for (const auto& p : m_points) {
            cv::circle(displayFrame, p, 5, cv::Scalar(0, 255, 0), -1);
        }

        
        if (m_points.size() == 2) {
            cv::line(displayFrame, m_points[0], m_points[1], cv::Scalar(0, 0, 255), 2);
        }

        cv::imshow(m_windowName, displayFrame);
        char key = (char)cv::waitKey(30);

        if (key == 'q' || key == 27) { 
            std::cout << "Exiting calibration due to 'q' or 'ESC' key press." << std::endl;
            break;
        }
    }
    s_currentCalibratorInstance = nullptr; 
}

bool CameraCalibrator::saveCalibration(const std::string& filename) const{
    std::ofstream outFile(filename);
    if(!outFile.is_open()){
        std::cerr<<"Error: Could not open file "<< filename<<" for writing calibration"<<std::endl;
        return false;
    }
    outFile << m_pixelPerCmRatio << std::endl;
    outFile << m_cmPerPixelRatio << std::endl;
    std::cout << "Calibration data saved to "<<filename<<std::endl;
    return true; 
}

bool CameraCalibrator::loadCalibration(const std::string& filename){
    std::ifstream inFile(filename);
    if (!inFile.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for reading calibration data." << std::endl;
        return false;
    }

    inFile >> m_pixelPerCmRatio;
    inFile >> m_cmPerPixelRatio;
    inFile.close();

    if (inFile.fail()) {
        std::cerr << "Error: Failed to read calibration data from " << filename << ". File might be corrupted or empty." << std::endl;
        m_pixelPerCmRatio = 0.0;
        m_cmPerPixelRatio = 0.0;
        return false;
    }

    std::cout << "Calibration data loaded from " << filename << std::endl;
    std::cout << "Loaded Pixel per Centimeter Ratio: " << m_pixelPerCmRatio << " pixels/cm" << std::endl;
    std::cout << "Loaded Centimeter per Pixel Ratio: " << m_cmPerPixelRatio << " cm/pixel" << std::endl;
    return true;
}