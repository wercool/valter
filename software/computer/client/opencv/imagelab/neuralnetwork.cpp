#include "neuralnetwork.h"

NeuralNetwork::NeuralNetwork()
{

}

void NeuralNetwork::readReferenceObjectMat()
{
    cv::Mat referenceObjectRGB = cv::imread(getReferenceObjectFileName());
    cv::cvtColor(referenceObjectRGB, referenceObject, CV_BGR2GRAY);
    cv::imshow("Reference Object", referenceObject);

    vector<double> referenceObjectVector;

    for(int y = 0; y < referenceObjectRGB.rows; y++)
    {
        for(int x = 0; x < referenceObjectRGB.cols; x++)
        {
            cv::Scalar intensity = referenceObjectRGB.at<uchar>(y, x);
            referenceObjectVector.push_back(intensity[0] / 255.0);
        }
    }
    setReferenceObjectVector(referenceObjectVector);
}

void NeuralNetwork::createTrainingObjectsFromReferences()
{
    referenceObjectsFileName.clear();

    DIR *dpdf;
    struct dirent *epdf;
    dpdf = opendir(getReferenceObjectsFolderName().c_str());
    if (dpdf != NULL)
    {
       while ((epdf = readdir(dpdf)))
       {
          if (std::string(epdf->d_name) != "." && std::string(epdf->d_name) != "..")
          {
            string referenceObjectFileName = std::string(epdf->d_name);
            referenceObjectsFileName.push_back(referenceObjectFileName);
            qDebug("Reference Objects: %s", referenceObjectFileName.c_str());
          }
       }
    }
    trainingDataCreationProcessingThread = new std::thread(&NeuralNetwork::trainingDataCreationWorker, this);
}

void NeuralNetwork::trainingDataCreationWorker()
{
    unsigned int tsNum = 0;
    for (unsigned int i = 0; i < referenceObjectsFileName.size(); i++)
    {
        string referenceObjectFileName = getReferenceObjectsFolderName() + "/" + referenceObjectsFileName[i];
        cv::Mat referenceObjectRGB = cv::imread(referenceObjectFileName);
        cv::Mat referenceObject;
        cv::cvtColor(referenceObjectRGB, referenceObject, CV_BGR2GRAY);
        cv::imshow("Reference Object", referenceObject);

        double angle = 0.0;
        vector<double> scales = {1.0, 0.9, 0.8, 0.7};

        for (int s = 0; s < getTrainingSamplesNumber(); s++)
        {
            for (unsigned int s = 0; s < scales.size(); s++)
            {
                cv::Mat referenceObjectInst = referenceObject.clone();

                cv::Point2f referenceObjectInstCenter(referenceObjectInst.cols / 2.0, referenceObjectInst.rows/2.0);
                cv::Mat referenceObjectInstRotMat = cv::getRotationMatrix2D(referenceObjectInstCenter, angle, scales[s]);
                cv::Mat transformedReferenceObject = cv::Mat(referenceObject.cols, referenceObject.rows, referenceObject.type(), cv::Scalar(255));
                cv::warpAffine(referenceObjectInst, transformedReferenceObject, referenceObjectInstRotMat, referenceObjectInst.size(), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);

                cv::imshow("Transformed Reference Object", transformedReferenceObject);

                if (!getCreateTrainingSamplesPreview())
                {
                    string referenceObjectFileNameWOExt = ((string)referenceObjectsFileName[i]).substr(0, ((string)referenceObjectsFileName[i]).find(".jpg"));
                    string trainingSampleFileName = format_string("%s/%s-ts-%d.jpg", getTrainingObjectsFolderName().c_str(), referenceObjectFileNameWOExt.c_str(), tsNum++);
                    qDebug("Training sample file: %s", trainingSampleFileName.c_str());
                    cv::imwrite(trainingSampleFileName.c_str(), transformedReferenceObject);
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(getCreateTrainingObjectsShowDelay()));
            }
            angle += 180 / getTrainingSamplesNumber();
        }
    }
}

void NeuralNetwork::initNetwork()
{
    miniBatchQualifier.clear();

    double randBiasScale = 1.0;
    double randWeightScale = 1.0;
    biases.clear();
    for (unsigned int l = 1; l < layers.size(); l++)
    {
        vector<double> layerNeuronBiases;
        for (int n = 0; n < layers[l]; n++)
        {
            double randBias = (((double)(rand() % 100)) / 100.0)*((rand() % 2) == 0 ? -1 : 1) * randBiasScale;
            layerNeuronBiases.push_back(randBias);
        }
        biases.push_back(layerNeuronBiases);

        vector<vector<double>> layerNeuronWeights;
        for (int n = 0; n < layers[l]; n++)
        {
            vector<double> neuronWeights;
            for (int w = 0; w < layers[l - 1]; w++)
            {
                double randWeight = (((double)(rand() % 100)) / 100.0)*((rand() % 2) == 0 ? -1 : 1) * randWeightScale;
                neuronWeights.push_back(randWeight);
            }
            layerNeuronWeights.push_back(neuronWeights);
        }
        weights.push_back(layerNeuronWeights);
    }
}

void NeuralNetwork::trainingWorker()
{
    SGD();
    qDebug("Training finished.");
}

void NeuralNetwork::SGD()
{
    for (int e = 0; e < epochs; e++)
    {
        createMiniBatch();
        //mbi = mini batch index
        for (unsigned int mbi = 0; mbi < miniBatchData.size(); mbi++)
        {
            singleSampleTraining(miniBatchData[mbi], miniBatchQualifier[mbi]);
        }
    }
}

void NeuralNetwork::createMiniBatch()
{
    miniBatchData.clear();
    miniBatchQualifier.clear();

    vector<int> trainingSampleFileVectorIndexes;
    vector<string> trainingSampleFileNames;

    //selecting random training samples
    for (int i = 0; i < miniBatchSize; i++)
    {
        int trainingSampleFileVectorIndex = rand() % trainingSamplesFileName.size();

        if(std::find(trainingSampleFileVectorIndexes.begin(), trainingSampleFileVectorIndexes.end(), trainingSampleFileVectorIndex) != trainingSampleFileVectorIndexes.end())
        {
            i--;
            continue;
        }

        trainingSampleFileNames.push_back(trainingSamplesFileName[trainingSampleFileVectorIndex]);

        trainingSampleFileVectorIndexes.push_back(trainingSampleFileVectorIndex);
    }

    for (int i = 0; i < miniBatchSize; i++)
    {
        string trainingSampleFileName = trainingSampleFileNames[i];
        string trainingSampleFilePath = getTrainingObjectsFolderName() + "/" + trainingSampleFileName;
        //getting sample grayscale vector
        vector<double> trainingSampleVector = getGrayscaleVectorFromMatFile(trainingSampleFilePath);
        miniBatchData.push_back(trainingSampleVector);

        int sampleQualifierRespectiveNeuronIdx = atoi(trainingSampleFileName.substr(0, trainingSampleFileName.find('-')).c_str());
        vector<double> sampleQualifier;
        //oli - output layer neuron index
        for (int oli = 0; oli < layers[layers.size() - 1]; oli++)
        {
            sampleQualifier.push_back((sampleQualifierRespectiveNeuronIdx == oli) ? 1.0 : 0.0);
        }
        miniBatchQualifier.push_back(sampleQualifier);
    }
}

void NeuralNetwork::singleSampleTraining(std::vector<double> sampleData, std::vector<double> sampleQualifier)
{
    BIASES nabla_b;
    WEIGHTS nabla_w;
    zeroNablas(&nabla_b, &nabla_w);

    BIASES d_nabla_b;
    WEIGHTS d_nabla_w;
    backpropagation(sampleData, sampleQualifier, &d_nabla_b, &d_nabla_w);
}

void NeuralNetwork::backpropagation(std::vector<double> sampleData,
                                    std::vector<double> sampleQualifier,
                                    NeuralNetwork::pBIASES d_nabla_b,
                                    NeuralNetwork::pWEIGHTS d_nabla_w)
{
    BIASES nabla_b;
    WEIGHTS nabla_w;
    zeroNablas(&nabla_b, &nabla_w);

    vector<double> activation = sampleData; //output of the layer, input for the next layer
    ACTIVATIONS activations;                //actinvations by layer (for backpropagation)
    activations.push_back(activation);

    vector<vector<double>> zs;

    for (unsigned int l = 1; l < layers.size(); l++)
    {
        vector<vector<double>> neuronLayerWeights = weights[l - 1];
        vector<double> neuronLayerBiases = biases[l - 1];
        vector<double> zl; //input for sigmoid function of each neuron in the layer
        vector<double> al; //output of neurons (zl passed through sigmoig) for the layer
        for (int n = 0; n < layers[l]; n++)
        {
            vector<double> neuronWeights = neuronLayerWeights[n];
            double zn = dotProduct(activation, neuronWeights) + neuronLayerBiases[n];
            zl.push_back(zn);
            al.push_back(sigmoid(zn));
        }
        zs.push_back(zl);
        activations.push_back(al);
        activation = al;
    }
    //qDebug("Output layer size: %d", ((vector<double>)(activations[activations.size() - 1])).size());
    vector<double> delta = HadamarProduct(costDerivative(activations[activations.size() - 1], sampleQualifier), sigmoidPrime(zs[zs.size() - 1]));
    nabla_b[nabla_b.size() - 1] = delta;
    vector<vector<double>> dw;
    for (unsigned i = 0; i < delta.size(); i++)
    {
        vector<double> dwn;
        for (unsigned int j = 0; j < ((vector<double>)activations[activations.size() - 2]).size(); j++)
        {
            dwn.push_back(delta[i] * ((vector<double>)activations[activations.size() - 2])[j]);
        }
        dw.push_back(dwn);
    }
    nabla_w[nabla_w.size() - 1] = dw;

    for (unsigned int l = (layers.size() - 2); l > 0; l--)
    {
        vector<double> z = zs[l - 1];
        vector<double> sp = sigmoidPrime(z);

        vector<vector<double>> weightsInPrevLayer = weights[l];
        vector<double> deltaWeight;
        for (unsigned int i = 0; i < weightsInPrevLayer.size(); i++)
        {
            deltaWeight.push_back(dotProduct(delta, weightsInPrevLayer[i]));
        }
        qDebug("%d %d", deltaWeight.size(), sp.size());
    }
}

double NeuralNetwork::sigmoid(double z)
{
    double s = 1.0 / (1.0 + exp(-z));
    return s;
}

vector<double> NeuralNetwork::sigmoidPrime(vector<double> z)
{
    vector<double> sigmoidPrime;
    for (unsigned int i = 0; i < z.size(); i++)
    {
        sigmoidPrime.push_back(sigmoid(z[i]) * (1 - sigmoid(z[i])));
    }
    return sigmoidPrime;
}

//gradient of the cost function
vector<double> NeuralNetwork::costDerivative(vector<double> outputActivations, vector<double> qualifier)
{
    vector<double> costDerivative;
    for (unsigned int i = 0; i < outputActivations.size(); i++)
    {
        costDerivative.push_back(outputActivations[i] - qualifier[i]);
    }
    return costDerivative;
}

void NeuralNetwork::zeroNablas(NeuralNetwork::pBIASES nabla_b, NeuralNetwork::pWEIGHTS nabla_w)
{
    for (unsigned int l = 1; l < layers.size(); l++)
    {
        vector<double> layerNeuronBiases;
        for (int n = 0; n < layers[l]; n++)
        {
            layerNeuronBiases.push_back(0.0);
        }
        nabla_b->push_back(layerNeuronBiases);

        vector<vector<double>> layerNeuronWeights;
        for (int n = 0; n < layers[l]; n++)
        {
            vector<double> neuronWeights;
            for (int w = 0; w < layers[l - 1]; w++)
            {
                neuronWeights.push_back(0.0);
            }
            layerNeuronWeights.push_back(neuronWeights);
        }
        nabla_w->push_back(layerNeuronWeights);
    }

}

void NeuralNetwork::readTrainingSamplesFileNames()
{
    trainingSamplesFileName.clear();

    DIR *dpdf;
    struct dirent *epdf;
    dpdf = opendir(getTrainingObjectsFolderName().c_str());
    if (dpdf != NULL)
    {
       while ((epdf = readdir(dpdf)))
       {
          if (std::string(epdf->d_name) != "." && std::string(epdf->d_name) != "..")
          {
            string trainingSampleFileName = std::string(epdf->d_name);
            int outputNeuronNum = atoi(trainingSampleFileName.substr(0, trainingSampleFileName.find('-')).c_str());
            if (outputNeuronNum < layers[layers.size() - 1])
            {
                trainingSamplesFileName.push_back(trainingSampleFileName);
                qDebug("Training Sample: %s , Neuron: %d", trainingSampleFileName.c_str(), outputNeuronNum);
            }
          }
       }
    }
}

string NeuralNetwork::getReferenceObjectFileName() const
{
    return referenceObjectFileName;
}

void NeuralNetwork::setReferenceObjectFileName(const string &value)
{
    referenceObjectFileName = value;
}

cv::Mat NeuralNetwork::getReferenceObject() const
{
    return referenceObject;
}

void NeuralNetwork::setReferenceObject(const cv::Mat &value)
{
    referenceObject = value;
}

vector<double> NeuralNetwork::getReferenceObjectVector() const
{
    return referenceObjectVector;
}

void NeuralNetwork::setReferenceObjectVector(const vector<double> &value)
{
    referenceObjectVector = value;
//    for (unsigned int i = 0; i < referenceObjectVector.size(); i++)
//    {
//        qDebug("%.2f", referenceObjectVector[i]);
//    }
}

string NeuralNetwork::getReferenceObjectsFolderName() const
{
    return referenceObjectsFolderName;
}

void NeuralNetwork::setReferenceObjectsFolderName(const string &value)
{
    referenceObjectsFolderName = value;
}

string NeuralNetwork::getTrainingObjectsFolderName() const
{
    return trainingObjectsFolderName;
}

void NeuralNetwork::setTrainingObjectsFolderName(const string &value)
{
    trainingObjectsFolderName = value;
}

int NeuralNetwork::getCreateTrainingObjectsShowDelay() const
{
    return createTrainingObjectsShowDelay;
}

void NeuralNetwork::setCreateTrainingObjectsShowDelay(int value)
{
    createTrainingObjectsShowDelay = value;
}

int NeuralNetwork::getTrainingSamplesNumber() const
{
    return trainingSamplesNumber;
}

void NeuralNetwork::setTrainingSamplesNumber(int value)
{
    trainingSamplesNumber = value;
}

bool NeuralNetwork::getCreateTrainingSamplesPreview() const
{
    return createTrainingSamplesPreview;
}

void NeuralNetwork::setCreateTrainingSamplesPreview(bool value)
{
    createTrainingSamplesPreview = value;
}

std::vector<int> NeuralNetwork::getLayers() const
{
    return layers;
}

void NeuralNetwork::setLayers(const std::vector<int> &value)
{
    layers = value;
}

int NeuralNetwork::getMiniBatchSize() const
{
    return miniBatchSize;
}

void NeuralNetwork::setMiniBatchSize(int value)
{
    miniBatchSize = value;
}

int NeuralNetwork::getEpochs() const
{
    return epochs;
}

void NeuralNetwork::setEpochs(int value)
{
    epochs = value;
}

double NeuralNetwork::getEta() const
{
    return eta;
}

void NeuralNetwork::setEta(double value)
{
    eta = value;
}
bool NeuralNetwork::getTraining() const
{
    return training;
}

void NeuralNetwork::setTraining(bool value)
{
    training = value;
    if (training)
    {
        trainingThread = new std::thread(&NeuralNetwork::trainingWorker, this);
    }
}

vector<string> NeuralNetwork::getTrainingSamplesFileName() const
{
    return trainingSamplesFileName;
}

void NeuralNetwork::setTrainingSamplesFileName(const vector<string> &value)
{
    trainingSamplesFileName = value;
}
