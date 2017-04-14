#ifndef CASCADECLASSIFIER_H
#define CASCADECLASSIFIER_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <QDebug>
#include "utils.h"
#include <unistd.h>
#include <thread>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iterator>
#include <algorithm>


using namespace std;

class CascadeClassifier
{
private:
    bool mode = 0; // 0 - training, 1 - detection
    string positiveImagesFolder = "/tmp";
    string negativeImagesFolder = "/tmp";
    bool capturePositive = false;
    bool captureNegative = false;

    vector<string> positiveFileNames;
    std::thread *positiveImagesProcessingThread;
    int positiveImageProcessingDelay = 500;
    int positiveImageProcessingThreshold = 100;

    //Video
    cv::Mat videoFrame;
    cv::VideoCapture videoCapture;
    std::thread *captureVideoThread;
    bool videoCaptured = false;
    int positiveImageNum = 0;
    int negativeImageNum = 0;

public:
    CascadeClassifier();

    void captureVideo();
    void stopVideo();
    void captureVideoWorker();

    void processPositiveImagesToCollectionFile();
    void positiveImagesProcessingWorker();

    string getPositiveImagesFolder() const;
    void setPositiveImagesFolder(const string &value);
    string getNegativeImagesFolder() const;
    void setNegativeImagesFolder(const string &value);
    bool getCapturePositive() const;
    void setCapturePositive(bool value);
    bool getCaptureNegative() const;
    void setCaptureNegative(bool value);
    int getPositiveImageProcessingDelay() const;
    void setPositiveImageProcessingDelay(int value);
    int getPositiveImageProcessingThreshold() const;
    void setPositiveImageProcessingThreshold(int value);
};

#endif // CASCADECLASSIFIER_H
