#ifndef UTILS_H
#define UTILS_H

#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/photo.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <unistd.h>
#include <thread>

static __attribute__((unused)) std::string format_string(const std::string fmt_str, ...)
{
    int final_n, n = ((int)fmt_str.size()) * 2; /* Reserve two times as much as the length of the fmt_str */
    std::string str;
    std::unique_ptr<char[]> formatted;
    va_list ap;
    while(1)
    {
        formatted.reset(new char[n]); /* Wrap the plain char array into the unique_ptr */
        strcpy(&formatted[0], fmt_str.c_str());
        va_start(ap, fmt_str);
        final_n = vsnprintf(&formatted[0], n, fmt_str.c_str(), ap);
        va_end(ap);
        if (final_n < 0 || final_n >= n)
            n += abs(final_n - n + 1);
        else
            break;
    }
    return std::string(formatted.get());
}

static __attribute__((unused)) std::vector<cv::Point> contoursConvexHull(std::vector<std::vector<cv::Point>>contours)
{
    std::vector<cv::Point> result;
    std::vector<cv::Point> pts;
    for ( size_t i = 0; i < contours.size(); i++)
    {
        for ( size_t j = 0; j < contours[i].size(); j++)
        {
            pts.push_back(contours[i][j]);
        }
    }
    cv::convexHull(pts, result);
    return result;
}

static __attribute__((unused)) void createPOI(const cv::Mat& src, const cv::Mat& polyMat, cv::Mat& dst)
{
    std::vector<cv::Mat> array(3);

    for (int i=0; i<3; i++)
    {
        cv::extractChannel(src, array[i], i);
        cv::bitwise_and(array[i], polyMat, array[i]);
    }

    cv::merge(array, dst);
}

static __attribute__((unused)) std::vector<double> getGrayscaleVectorFromMatFile(const std::string fileName)
{
    cv::Mat matRGB = cv::imread(fileName.c_str());
    cv::Mat matGrayscale;
    cv::cvtColor(matRGB, matGrayscale, CV_BGR2GRAY);

    std::vector<double> grayscaleMatVector;

    for(int y = 0; y < matRGB.rows; y++)
    {
        for(int x = 0; x < matRGB.cols; x++)
        {
            cv::Scalar intensity = matRGB.at<uchar>(y, x);
            grayscaleMatVector.push_back(intensity[0] / 255.0);
        }
    }

    return grayscaleMatVector;
}

static __attribute__((unused)) double dotProduct(std::vector<double> A, std::vector<double> B)
{
    double scalar;
    for(unsigned int i = 0; i < A.size(); i++)
    {
       scalar = scalar + (A[i] * B[i]);
    }
    return scalar;
}

static __attribute__((unused)) std::vector<double> HadamarProduct(std::vector<double> A, std::vector<double> B)
{
    std::vector<double> HadamarProduct;
    for(unsigned int i = 0; i < A.size(); i++)
    {
       HadamarProduct.push_back(A[i] * B[i]);
    }
    return HadamarProduct;
}

#endif // UTILS_H
