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

#include "utils.h"

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
    vector<string> trainingSamplesFileName;

    std::thread *trainingDataCreationProcessingThread;
    int createTrainingObjectsShowDelay;
    int trainingSamplesNumber;
    bool createTrainingSamplesPreview = true;

    //Network Parameters
    std::vector<int> layers;
    int miniBatchSize = 10;
    int epochs = 10;
    double eta = 0.15;

    typedef std::vector<std::vector<double>> BIASES, *pBIASES;
    typedef std::vector<std::vector<std::vector<double>>> WEIGHTS, *pWEIGHTS;
    typedef std::vector<std::vector<double>> ACTIVATIONS, *pACTIVATIONS;

    BIASES biases;
    WEIGHTS weights;
    std::vector<std::vector<double>> miniBatchData;
    std::vector<std::vector<double>> miniBatchQualifier;

    bool training = false;
    std::thread *trainingThread;

    void SGD();
    void createMiniBatch();
    void singleSampleTraining(std::vector<double> sampleData, std::vector<double> sampleQualifier);
    void backpropagation(std::vector<double> sampleData, std::vector<double> sampleQualifier, pBIASES d_nabla_b, pWEIGHTS d_nabla_w);

    double sigmoid(double z);

    void zeroNablas(pBIASES nabla_b, pWEIGHTS nabla_w);

    void trainingWorker();

public:
    NeuralNetwork();

    void readReferenceObjectMat();
    void createTrainingObjectsFromReferences();
    void trainingDataCreationWorker();

    void initNetwork();

    void readTrainingSamplesFileNames();
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
    bool getCreateTrainingSamplesPreview() const;
    void setCreateTrainingSamplesPreview(bool value);
    std::vector<int> getLayers() const;
    void setLayers(const std::vector<int> &value);
    int getMiniBatchSize() const;
    void setMiniBatchSize(int value);
    int getEpochs() const;
    void setEpochs(int value);
    double getEta() const;
    void setEta(double value);
    bool getTraining() const;
    void setTraining(bool value);
    vector<string> getTrainingSamplesFileName() const;
    void setTrainingSamplesFileName(const vector<string> &value);
};

#endif // NEURALNETWORK_H
