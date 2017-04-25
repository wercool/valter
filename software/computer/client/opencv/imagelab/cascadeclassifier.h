#ifndef CASCADECLASSIFIER_H
#define CASCADECLASSIFIER_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include "opencv2/objdetect.hpp"
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

#include <sys/types.h>
#include <dirent.h>


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
    vector<string> negativeFileNames;
    std::thread *positiveImagesProcessingThread;
    int positiveImageProcessingDelay = 500;
    int positiveImageProcessingThreshold = 100;
    int positiveImageProcessingGaussianBlur = 0;
    int positiveImageProcessingBrightness = 0;
    double positiveImageProcessingContrast = 1.0;
    int positiveImageProcessingCannyThreshold = 100;
    int minContourArea = 500;
    std::string cascadeClassifierFile;
    cv::CascadeClassifier objectCascade;
    std::thread *objectDetectionThread;
    bool cropPositiveImages = false;
    bool cropPositiveImagesRealTime = false;
    bool sharpen = false;
    float sharpenFactor = 0.0;
    bool positiveSampleBackgroundBlack = true;

    int croppedWidth = 100;
    int croppedHeight = 100;

    vector<string> positiveCroppedInfo;


    std::thread *trainingSamplesProcessingThread;

    //Video
    cv::Mat videoFrame;
    cv::VideoCapture videoCapture;
    std::thread *captureVideoThread;
    bool videoCaptured = false;
    int positiveImageNum = 0;
    int negativeImageNum = 0;

    bool thresholdPreview = false;
    bool gaussianBlurPreview = true;
    bool brightnessPreview = true;
    bool showCroppedPositives = false;
    bool createTrainingSamplesPreview = true;

    std::string samplesFolder;

public:
    CascadeClassifier();

    void captureVideo();
    void stopVideo();
    void captureVideoWorker();

    void processPositiveImagesToCollectionFile(bool crop);
    void positiveImagesProcessingWorker();
    void startDetection();
    void stopDetection();
    void objectDetectionWorker();

    void processTrainingSamples();
    void trainingSamplesProcessingWorker();

    void readPositiveImagesDir();
    void readNegativeImagesDir();
    void savePositiveCroppedInfo();

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
    std::string getCascadeClassifierFile() const;
    void setCascadeClassifierFile(const std::string &value);
    int getCroppedWidth() const;
    void setCroppedWidth(int value);
    int getCroppedHeight() const;
    void setCroppedHeight(int value);
    bool getThresholdPreview() const;
    void setThresholdPreview(bool value);
    bool getGaussianBlurPreview() const;
    void setGaussianBlurPreview(bool value);
    bool getBrightnessPreview() const;
    void setBrightnessPreview(bool value);
    int getPositiveImageProcessingBrightness() const;
    void setPositiveImageProcessingBrightness(int value);
    double getPositiveImageProcessingContrast() const;
    void setPositiveImageProcessingContrast(double value);
    int getPositiveImageProcessingGaussianBlur() const;
    void setPositiveImageProcessingGaussianBlur(int value);
    int getPositiveImageProcessingCannyThreshold() const;
    void setPositiveImageProcessingCannyThreshold(int value);
    int getMinContourArea() const;
    void setMinContourArea(int value);
    bool getCropPositiveImagesRealTime() const;
    void setCropPositiveImagesRealTime(bool value);
    bool getShowCroppedPositives() const;
    void setShowCroppedPositives(bool value);
    bool getCreateTrainingSamplesPreview() const;
    void setCreateTrainingSamplesPreview(bool value);
    vector<string> getNegativeFileNames() const;
    void setNegativeFileNames(const vector<string> &value);
    vector<string> getPositiveFileNames() const;
    void setPositiveFileNames(const vector<string> &value);
    std::string getSamplesFolder() const;
    void setSamplesFolder(const std::string &value);
    bool getSharpen() const;
    void setSharpen(bool value);
    float getSharpenFactor() const;
    void setSharpenFactor(float value);
    bool getPositiveSampleBackgroundBlack() const;
    void setPositiveSampleBackgroundBlack(bool value);
};

#endif // CASCADECLASSIFIER_H
