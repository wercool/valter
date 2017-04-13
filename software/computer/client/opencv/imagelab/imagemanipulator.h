#ifndef IMAGEMANIPULATOR_H
#define IMAGEMANIPULATOR_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/photo.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <QDebug>
#include "utils.h"
#include <unistd.h>
#include <thread>

class ImageManipulator
{
private:
    cv::Mat srcImage;
    cv::Mat nonModifiedImage;
    cv::Mat procImage;
    cv::Mat procAggregateImage;
    cv::Mat videoFrame;
    cv::Mat videoFrameWithKeypoints;
    cv::Mat objectImage;
    cv::Mat objectImageWithKeypoints;

    cv::VideoCapture videoCapture;

    //Brightness, contrast, grayscale
    int procImageBrightness = 0;
    double procImageContrast = 1.0;
    bool grayscale = false;
    int colorReduceFactor = 0;
    int colorDenoiserStrength = 0;

    //Linear filters
    int normalizedBoxFilter = 0;
    int homogeneousBlur = 0;
    int gaussianBlur = 0;
    int medianBlur = 0;
    int bilateralBlur = 0;

    //Canny
    int cannyThreshold = 100;
    unsigned int contourLengthThreshold = 0;
    cv::Mat cannyResult;

    //Contrours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point>> processedContours;

    //Video
    bool videoCaptured = false;

    std::thread *captureVideoThread;

    //Features
    std::vector<cv::KeyPoint> keypoints;
    std::vector<cv::KeyPoint> sceneKeypoints;
    int minHessian = 400;
public:
    ImageManipulator();

    void preProcess();
    void setROI(cv::Rect roi);

    void captureVideo();
    void stopVideo();
    void captureVideoWorker();

    void changeBrightnessAndContrast();
    void colorReduce();
    void colorDenoising();
    void applyNormalizedBoxBlur();
    void applyHomogeneousBlur();
    void applyGaussianBlur();
    void applyMedianBlur();
    void applyBilateralBlur();

    void findContours();
    void extractFeaturesFromObjectImage();

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
    int getNormalizedBoxFilter() const;
    void setNormalizedBoxFilter(int value);
    unsigned int getContourLengthThreshold() const;
    void setContourLengthThreshold(int value);
    cv::Mat getObjectImage() const;
    void setObjectImage(const cv::Mat &value);
    cv::Mat getObjectImageWithKeypoints() const;
    void setObjectImageWithKeypoints(const cv::Mat &value);
    int getMinHessian() const;
    void setMinHessian(int value);
    cv::Mat getNonModifiedImage() const;
    void setNonModifiedImage(const cv::Mat &value);
    int getColorReduce() const;
    void setColorReduce(int value);
    int getColorReduceFactor() const;
    void setColorReduceFactor(int value);
    int getColorDenoiserStrength() const;
    void setColorDenoiserStrength(int value);
};

#endif // IMAGEMANIPULATOR_H
