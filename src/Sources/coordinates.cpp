#include "coordinates.h"
#include <math.h>

cv::Point Coordinates::offset(0,0);
double Coordinates::theta = -22.227 * M_PI / 180;
float Coordinates::pixelsToMM = 0.89;

Coordinates::Coordinates()
{

}

bool Coordinates::calibrateToRobotOrigo(const std::string &path, const std::pair<cv::Mat, cv::Mat>& calibrationMaps)
{
    std::vector<cv::String> fileNames;
    cv::glob(path, fileNames, false);
    std::vector<cv::Point> CoordinateXandY;

    if (fileNames.size() != 2) {
        return false;
    }

    for (int i = 0; i < fileNames.size(); i++) {
        std::cout << fileNames[i] << std::endl;

        cv::Mat img = cv::imread(fileNames[i]);
        cv::Mat imgUndistorted;

        cv::remap(img, imgUndistorted, calibrationMaps.first, calibrationMaps.second, CV_INTER_LINEAR);

        cv::Mat ball_ROI(imgUndistorted, cv::Rect(440,0,700, 880));
        cv::Mat grey(ball_ROI.clone());
        cv::cvtColor(ball_ROI, grey, cv::COLOR_BGR2GRAY);

        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(grey, circles, CV_HOUGH_GRADIENT, 2, 10, 100, 50, 15, 20);

        cv::Point cameraBallLocationXY;
        if (circles.size() == 1) {
            cameraBallLocationXY = cv::Point(circles[0][0], circles[0][1]);
        }

//        for( size_t i = 0; i < circles.size(); i++ )
//        {
//            cv::Vec3i c = circles[i];
//            cv::Point center = cv::Point(c[0], c[1]);
//            // circle center
//            cv::circle( ball_ROI, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
//            // circle outline
//            int radius = c[2];
//            cv::circle( ball_ROI, center, radius, cv::Scalar(255,0,255), 3, cv::LINE_AA);
//        }

//        cv::imshow("ballimage", ball_ROI);
//        cv::waitKey(0);

        CoordinateXandY.push_back(cameraBallLocationXY);
    }

    offset.y = CoordinateXandY[0].x;
    offset.x = CoordinateXandY[1].y;


}

cv::Point Coordinates::cameraRotateToRobotSys(const cv::Point &cameraXandY)
{
    cv::Point RobotXandY;

    double rotationMatrix[2][2]  = {{std::cos(theta), -std::sin(theta)}, {std::sin(theta), std::cos(theta)}};

    RobotXandY.x = (cameraXandY.x - offset.x * pixelsToMM) * rotationMatrix[0][0] + (cameraXandY.y - offset.y) * rotationMatrix[0][1];
    RobotXandY.y = -((cameraXandY.x - offset.x) * rotationMatrix[1][0] + (cameraXandY.y - offset.y) * rotationMatrix[1][1]);

//    RobotXandY.x = ((cameraXandY.x - offset.x) * pixelsToMM) * rotationMatrix[0][0] + ((cameraXandY.y - offset.y) * pixelsToMM) * rotationMatrix[0][1];
//    RobotXandY.y = -(((cameraXandY.x - offset.x) * pixelsToMM) * rotationMatrix[1][0] + ((cameraXandY.y - offset.y) * pixelsToMM) * rotationMatrix[1][1]);


    return RobotXandY;

}

cv::Point2f *Coordinates::generate4CameraPoints()
{
    cv::Point2f* cameraP = new cv::Point2f[4];
    cameraP[0] = cameraRotateToRobotSys(cv::Point2f(119, 769));
    cameraP[1] = cameraRotateToRobotSys(cv::Point2f(597, 771));
    cameraP[2] = cameraRotateToRobotSys(cv::Point2f(119, 253));
    cameraP[3] = cameraRotateToRobotSys(cv::Point2f(597, 255));

    return cameraP;
}

cv::Point2f *Coordinates::generate4RobotPoints()
{
    cv::Point2f* robotP = new cv::Point2f[4];
    //missing coordinates here
    robotP[0] = cv::Point2f((29.9), (-856.1));
    robotP[1] = cv::Point2f((538.7), (-643.2));
    robotP[2] = cv::Point2f((1), (1));
    robotP[3] = cv::Point2f((1), (1));

    return robotP;
}

cv::Point Coordinates::getOffset() const
{
    return offset;
}


