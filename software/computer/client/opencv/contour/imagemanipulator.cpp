#include "imagemanipulator.h"

ImageManipulator::ImageManipulator()
{

}

void ImageManipulator::preProcess()
{
    changeBrightnessAndContrast();
    applyHomogeneousBlur();
    applyGaussianBlur();
    applyMedianBlur();
    applyBilateralBlur();
}

void ImageManipulator::changeBrightnessAndContrast()
{
    // http://docs.opencv.org/2.4/doc/tutorials/core/basic_linear_transform/basic_linear_transform.html
    cv::Mat resultImage = srcImage.clone();

    for( int y = 0; y < resultImage.rows; y++ )
    {
        for( int x = 0; x < resultImage.cols; x++ )
        {
            for( int c = 0; c < 3; c++ )
            {
                resultImage.at<cv::Vec3b>(y,x)[c] = cv::saturate_cast<uchar>(procImageContrast * (srcImage.at<cv::Vec3b>(y,x)[c]) + procImageBrightness);
            }
        }
    }

    if (grayscale)
    {
        cv::Mat resultImageGS(resultImage.size(), CV_8UC1);
        cv::cvtColor(resultImage, resultImageGS, CV_RGB2GRAY);
        resultImage = resultImageGS.clone();
    }

    procImage = resultImage.clone();
}

void ImageManipulator::applyHomogeneousBlur()
{
    // http://docs.opencv.org/2.4/doc/tutorials/imgproc/gausian_median_blur_bilateral_filter/gausian_median_blur_bilateral_filter.html
    cv::Mat resultImage = procImage.clone();
    for ( int i = 1; i < homogeneousBlur; i = i + 2 )
    {
        cv::blur(resultImage, procImage, cv::Size(i, i), cv::Point(-1,-1));
    }
}

void ImageManipulator::applyGaussianBlur()
{
    // http://docs.opencv.org/2.4/doc/tutorials/imgproc/gausian_median_blur_bilateral_filter/gausian_median_blur_bilateral_filter.html
    cv::Mat resultImage = procImage.clone();
    for ( int i = 1; i < gaussianBlur; i = i + 2 )
    {
        cv::GaussianBlur(resultImage, procImage, cv::Size(i, i), 0, 0);
    }
}

void ImageManipulator::applyMedianBlur()
{
    // http://docs.opencv.org/2.4/doc/tutorials/imgproc/gausian_median_blur_bilateral_filter/gausian_median_blur_bilateral_filter.html
    cv::Mat resultImage = procImage.clone();
    for ( int i = 1; i < medianBlur; i = i + 2 )
    {
        cv::medianBlur(resultImage, procImage, i);
    }
}

void ImageManipulator::applyBilateralBlur()
{
    // http://docs.opencv.org/2.4/doc/tutorials/imgproc/gausian_median_blur_bilateral_filter/gausian_median_blur_bilateral_filter.html
    cv::Mat resultImage = procImage.clone();
    for ( int i = 1; i < bilateralBlur; i = i + 2 )
    {
        cv::bilateralFilter(resultImage, procImage, i, i*2, i/2);
    }
}

void ImageManipulator::findContours()
{
    preProcess();
    cv::Mat cannyResult;
    cv::Mat procImageGray;
    if (!grayscale)
    {
        cv::cvtColor(procImage, procImageGray, cv::COLOR_BGR2GRAY);
    }
    else
    {
        procImageGray = procImage.clone();
    }
    cv::Canny(procImageGray, cannyResult, cannyThreshold, cannyThreshold*2, 3);
    cv::imshow("Contours", cannyResult);
}

int ImageManipulator::getProcImageBrightness() const
{
    return procImageBrightness;
}

void ImageManipulator::setProcImageBrightness(int value)
{
    procImageBrightness = value;
}

double ImageManipulator::getProcImageContrast() const
{
    return procImageContrast;
}

void ImageManipulator::setProcImageContrast(double value)
{
    procImageContrast = value;
}

cv::Mat ImageManipulator::getProcImage() const
{
    return procImage;
}

void ImageManipulator::setProcImage(const cv::Mat &value)
{
    procImage = value;
}

cv::Mat ImageManipulator::getSrcImage() const
{
    return srcImage;
}

void ImageManipulator::setSrcImage(const cv::Mat &value)
{
    srcImage = value;
}

int ImageManipulator::getBilateralBlur() const
{
    return bilateralBlur;
}

void ImageManipulator::setBilateralBlur(int value)
{
    bilateralBlur = value;
}

int ImageManipulator::getMedianBlur() const
{
    return medianBlur;
}

void ImageManipulator::setMedianBlur(int value)
{
    medianBlur = value;
}

int ImageManipulator::getGaussianBlur() const
{
    return gaussianBlur;
}

void ImageManipulator::setGaussianBlur(int value)
{
    gaussianBlur = value;
}

int ImageManipulator::getHomogeneousBlur() const
{
    return homogeneousBlur;
}

void ImageManipulator::setHomogeneousBlur(int value)
{
    homogeneousBlur = value;
}

bool ImageManipulator::getGrayscale() const
{
    return grayscale;
}

void ImageManipulator::setGrayscale(bool value)
{
    grayscale = value;
}
