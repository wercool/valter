#ifndef TEMPLATEMATCHING_H
#define TEMPLATEMATCHING_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

using namespace std;

class TemplateMatching
{
private:
    cv::Mat targetImage;
    cv::Mat templateImage;

public:
    TemplateMatching();

    void readTargetImage(string targetImagePath);
    void readTemplateMatching(string templateImagePath);
};

#endif // TEMPLATEMATCHING_H
