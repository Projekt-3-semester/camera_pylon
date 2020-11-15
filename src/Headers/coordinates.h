#ifndef COORDINATES_H
#define COORDINATES_H
#include <vector>
#include <pylon/PylonIncludes.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class Coordinates
{
public:
    Coordinates();

    static bool calibrateToRobotOrigo(const std::string& path, const std::pair<cv::Mat, cv::Mat>& calibrationMaps);
    static cv::Point cameraRotateToRobotSys(const cv::Point& cameraXandY);

    static cv::Point2f * generate4CameraPoints();
    static cv::Point2f * generate4RobotPoints();

    cv::Point getOffset() const;

private:



    static int calibrationCount;
    static double theta;
    static float pixelsToMM;
    static cv::Point offset;
};

#endif // COORDINATES_H
