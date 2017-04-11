#ifndef IMAGEMANIPULATOR_H
#define IMAGEMANIPULATOR_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

class ImageManipulator
{
private:
    cv::Mat srcImage;
    cv::Mat procImage;

    //Brightness, contrast, grayscale
    int procImageBrightness = 0;
    double procImageContrast = 1.0;
    bool grayscale = false;

    //Linear filters
    int homogeneousBlur = 0;
    int gaussianBlur = 0;
    int medianBlur = 0;
    int bilateralBlur = 0;

    //Canny
    int cannyThreshold = 100;

    //Contrours
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

public:
    ImageManipulator();

    void preProcess();

    void changeBrightnessAndContrast();
    void applyHomogeneousBlur();
    void applyGaussianBlur();
    void applyMedianBlur();
    void applyBilateralBlur();

    void findContours();

    cv::Mat getSrcImage() const;
    void setSrcImage(const cv::Mat &value);
    cv::Mat getProcImage() const;

    void setProcImage(const cv::Mat &value);
    int getProcImageBrightness() const;
    void setProcImageBrightness(int value);
    double getProcImageContrast() const;
    void setProcImageContrast(double value);
    bool getGrayscale() const;
    void setGrayscale(bool value);

    int getHomogeneousBlur() const;
    void setHomogeneousBlur(int value);
    int getGaussianBlur() const;
    void setGaussianBlur(int value);
    int getMedianBlur() const;
    void setMedianBlur(int value);
    int getBilateralBlur() const;
    void setBilateralBlur(int value);
    bool getFindContours() const;
    void setFindContours(bool value);
    bool getContourProcess() const;
    void setContourProcess(bool value);
    int getCannyThreshold() const;
    void setCannyThreshold(int value);
};

#endif // IMAGEMANIPULATOR_H
