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
    int createTrainingObjectsShowDelay = 250;
    int trainingSamplesNumber;
    bool createTrainingSamplesPreview = true;

    //Network Parameters
    std::vector<int> layers; //contains number of neurons in eash layer
    int miniBatchSize = 10;
    int epochs = 10;
    double eta = 0.15;
    //QuadraticCost
    //CrossEntropyCost
    string costFunction = "QuadraticCost";

    typedef std::vector<std::vector<double>> BIASES;
    typedef std::vector<std::vector<std::vector<double>>> WEIGHTS;
    typedef std::vector<std::vector<double>> ACTIVATIONS;

    BIASES biases;
    WEIGHTS weights;
    std::vector<std::vector<double>> miniBatchData;
    std::vector<std::vector<double>> miniBatchQualifier;

    bool training = false;
    std::thread *trainingThread;

    void SGD();
    void createMiniBatch();
    void backpropagation(std::vector<double> sampleData, std::vector<double> sampleQualifier, BIASES &d_nabla_b, WEIGHTS &d_nabla_w);
    vector<double> feedForward(std::vector<double> inputActivation);

    vector<double> QuadraticCost(vector<double> activation, vector<double> sampleQualifier, vector<double> zl);
    vector<double> CrossEntropyCost(vector<double> activation, vector<double> sampleQualifier);

    double sigmoid(double z);
    vector<double> sigmoidPrime(vector<double> z);

    void zeroNablas(BIASES &nabla_b, WEIGHTS &nabla_w);
    vector<double> costDerivative(vector<double> outputActivations, vector<double>qualifier);

    void trainingWorker();

public:
    NeuralNetwork();

    void readReferenceObjectMat();
    void createTrainingObjectsFromReferences();
    void trainingDataCreationWorker();

    void initNetwork();
    void loadNetworkFromFile(string filepath);
    void saveNetworkToFile(string filepath);

    void recognizeReferenceObject();

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
    string getCostFunction() const;
    void setCostFunction(const string &value);
};

#endif // NEURALNETWORK_H
