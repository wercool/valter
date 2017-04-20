#ifndef NEURALNETWORK_H
#define NEURALNETWORK_H

#include <QDebug>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iterator>
#include <algorithm>
#include <sys/types.h>
#include <dirent.h>
#include <thread>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include "opencv2/objdetect.hpp"

using namespace std;

class NeuralNetwork
{
private:
    string referenceObjectFileName;
    cv::Mat referenceObject;
    vector<double> referenceObjectVector;

    string referenceObjectsFolderName;
    string trainingObjectsFolderName;

    vector<string> referenceObjectsFileName;

    std::thread *trainingDataCreationProcessingThread;
    int createTrainingObjectsShowDelay;
    int trainingSamplesNumber;

public:
    NeuralNetwork();

    void readReferenceObjectMat();
    void createTrainingObjectsFromReferences();
    void trainingDataCreationWorker();

    string getReferenceObjectFileName() const;
    void setReferenceObjectFileName(const string &value);
    cv::Mat getReferenceObject() const;
    void setReferenceObject(const cv::Mat &value);
    vector<double> getReferenceObjectVector() const;
    void setReferenceObjectVector(const vector<double> &value);
    string getTrainingObjectsFolderName() const;
    void setTrainingObjectsFolderName(const string &value);
    string getReferenceObjectsFolderName() const;
    void setReferenceObjectsFolderName(const string &value);
    int getCreateTrainingObjectsShowDelay() const;
    void setCreateTrainingObjectsShowDelay(int value);
    int getTrainingSamplesNumber() const;
    void setTrainingSamplesNumber(int value);
};

#endif // NEURALNETWORK_H
