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
        vector<double> scales = {1.0, 0.95, 0.9, 0.85, 0.8, 0.75, 0.7, 0.65, 0.6, 0.55, 0.5};

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
            if (getRotateSamples())
            {
                angle += 180 / getTrainingSamplesNumber();
            }
        }
    }
}

void NeuralNetwork::initNetwork()
{
    miniBatchData.clear();
    miniBatchQualifier.clear();

    double randBiasScale = 1.5;
    double randWeightScale = 1.5;

    biases.clear();
    weights.clear();

    for (unsigned int l = 1; l < layers.size(); l++)
    {
//        qDebug("Layer %d", l);

        vector<double> layerNeuronBiases;
        for (int n = 0; n < layers[l]; n++)
        {
            double randBias = (((double)(rand() % 100)) / 100.0)*((rand() % 2) == 0 ? -1 : 1) * randBiasScale;
            layerNeuronBiases.push_back(randBias);
        }
        biases.push_back(layerNeuronBiases);

//        string biasesStringified;
//        for (unsigned int n = 0 ; n < biases[l - 1].size(); n++)
//        {
//            biasesStringified += format_string("%.8f ", layerNeuronBiases[n]);
//        }
//        qDebug("B[ %s]", biasesStringified.c_str());

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

//        for (unsigned int wl = 0; wl < weights[l - 1].size(); wl++)
//        {
//            string weightsStringified;
//            for (unsigned int wn = 0; wn < weights[l - 1][wl].size(); wn++)
//            {
//                weightsStringified += format_string("%.8f ", weights[l - 1][wl][wn]);
//            }
//            qDebug("W[%d][ %s]", wl, weightsStringified.c_str());
//        }

    }
}

void NeuralNetwork::loadNetworkFromFile(string filepath)
{
    layers.clear();
    biases.clear();
    weights.clear();

    ifstream nnItializationFile;
    nnItializationFile.open(filepath);

    string dummyLine;
    std::getline(nnItializationFile, dummyLine);

    string layersStringified;
    std::getline(nnItializationFile, layersStringified);
    vector<string> layersStringifiedSeparated = split(layersStringified, ',');
    for (unsigned l = 0; l < layersStringifiedSeparated.size(); l++)
    {
        layers.push_back(atoi(stringToCharPtr(layersStringifiedSeparated[l])));
    }

    string miniBatchStringified;
    std::getline(nnItializationFile, miniBatchStringified);
    setMiniBatchSize(atoi(stringToCharPtr(miniBatchStringified)));

    string epochsStringified;
    std::getline(nnItializationFile, epochsStringified);
    setEpochs(atoi(stringToCharPtr(epochsStringified)));

    string etaStringified;
    std::getline(nnItializationFile, etaStringified);
    setEta(atof(stringToCharPtr(etaStringified)));

    string costFunction;
    std::getline(nnItializationFile, costFunction);
    setCostFunction(costFunction);

    std::getline(nnItializationFile, dummyLine);

    for (unsigned l = 1; l < layers.size(); l++)
    {
        vector<double> lnb;
        vector<vector<double>> lnw;
        std::getline(nnItializationFile, dummyLine);
        for (int n = 0; n < layers[l]; n++)
        {
            string lnbStringified;
            std::getline(nnItializationFile, lnbStringified);
            lnb.push_back(atof(stringToCharPtr(lnbStringified)));
            string lnwStringified;
            std::getline(nnItializationFile, lnwStringified);
            vector<string> lnwStringifiedSeparated = split(lnwStringified, ',');
            vector<double> nw;
            for (unsigned int w = 0; w < lnwStringifiedSeparated.size(); w++)
            {
                nw.push_back(atof(stringToCharPtr(lnwStringifiedSeparated[w])));
            }
            lnw.push_back(nw);
        }
        biases.push_back(lnb);
        weights.push_back(lnw);
    }
}

void NeuralNetwork::saveNetworkToFile(string filepath)
{
    ofstream nnItializationFile;
    nnItializationFile.open(filepath);

    nnItializationFile << "# Neural network parameters";
    nnItializationFile << "\n";
    std::ostringstream layerOSS;
    std::copy(layers.begin(), layers.end(), std::ostream_iterator<int>(layerOSS, ","));
    string layersStrigified = layerOSS.str().substr(0, layerOSS.str().size() - 1);
    nnItializationFile << layersStrigified << "\n";

    nnItializationFile << format_string("%d", miniBatchSize) << "\n";
    nnItializationFile << format_string("%d", epochs) << "\n";
    nnItializationFile << format_string("%.4f", eta) << "\n";
    nnItializationFile << format_string("%s", getCostFunction().c_str()) << "\n";

    nnItializationFile << "# Biases and Weights distributed layer by layer neuron by neuron";
    nnItializationFile << "\n";

    for (unsigned int l = 1; l < layers.size(); l++)
    {
        nnItializationFile << format_string("#Layer %d\n", l);
        for (int n = 0; n < layers[l]; n++)
        {
            double lnb = biases[l - 1][n];
            nnItializationFile << format_string("%e", lnb) << "\n";
            string weightsStringified;
            for (unsigned int w = 0; w < weights[l - 1][n].size(); w++)
            {
                weightsStringified += format_string("%e,", weights[l - 1][n][w]);
            }
            weightsStringified = weightsStringified.substr(0, weightsStringified.size() - 1);
            nnItializationFile << weightsStringified << "\n";
        }
    }

    nnItializationFile.close();
}

void NeuralNetwork::recognizeReferenceObject()
{
    vector<double> outputVector = feedForward(referenceObjectVector);
    qDebug("Neural Network Output Vector");
    for (unsigned int o = 0; o < outputVector.size(); o++)
    {
        qDebug("%.15f", outputVector[o]);
    }
}

void NeuralNetwork::trainingWorker()
{
    trainingSamples.clear();
    for (unsigned int i = 0; i < trainingSamplesFileName.size(); i++)
    {
        string trainingSampleFileName = trainingSamplesFileName[i];
        string trainingSampleFilePath = getTrainingObjectsFolderName() + "/" + trainingSampleFileName;
        //getting sample grayscale vector
        vector<double> trainingSampleVector = getGrayscaleVectorFromMatFile(trainingSampleFilePath);
        trainingSamples[trainingSampleFileName] = trainingSampleVector;
    }
    qDebug("%d samples data read", (int)trainingSamples.size());
    SGD();
    qDebug("Training finished.");
}

void NeuralNetwork::SGD()
{
    int miniBatchesInTrainingData = trainingDataLength / miniBatchSize;
    qDebug("Mini Batch cycles: %d", miniBatchesInTrainingData);

    for (int e = 0; e < epochs; e++)
    {
        vector<string> trainingSamplesFileNameShuffled = trainingSamplesFileName;
        random_shuffle(trainingSamplesFileNameShuffled.begin(), trainingSamplesFileNameShuffled.end());

        for (int mbci = 0; mbci < miniBatchesInTrainingData; mbci++)
        {
            vector<string> miniBatchSampleFileNames;
            for (int i = 0; i < miniBatchSize; i++)
            {
                miniBatchSampleFileNames.push_back(trainingSamplesFileNameShuffled[mbci*miniBatchSize + i]);
            }

            prepareMiniBatch(miniBatchSampleFileNames);

            qDebug("Epoch %d Mini Batch %d", e, mbci);

            if (!training)
            {
                qDebug("Training terminated.");
                return;
            }

            //mbi = mini batch index
            BIASES nabla_b;
            WEIGHTS nabla_w;
            zeroNablas(nabla_b, nabla_w);
            for (unsigned int mbi = 0; mbi < miniBatchData.size(); mbi++)
            {
                BIASES d_nabla_b;
                WEIGHTS d_nabla_w;
                zeroNablas(d_nabla_b, d_nabla_w);
                backpropagation(miniBatchData[mbi], miniBatchQualifier[mbi], d_nabla_b, d_nabla_w);
                for (unsigned int l = 1; l < layers.size(); l++)
                {
                    for (int n = 0; n < layers[l]; n++)
                    {
                        nabla_b[l - 1][n] += d_nabla_b[l - 1][n];
                        for (unsigned int w = 0; w < weights[l - 1][n].size(); w++)
                        {
                            nabla_w[l - 1][n][w] += d_nabla_w[l - 1][n][w];
                        }
                    }
                }
            }
            //update neural network biases and weights
            for (unsigned int mbi = 0; mbi < miniBatchData.size(); mbi++)
            {
                for (unsigned int l = 1; l < layers.size(); l++)
                {
                    for (int n = 0; n < layers[l]; n++)
                    {
                        //bk → b′k = bk − (η/m)*∑∂CXj/∂bl
                        biases[l - 1][n] = biases[l - 1][n] - (eta / miniBatchData.size()) * nabla_b[l - 1][n];
                        for (unsigned int w = 0; w < weights[l - 1][n].size(); w++)
                        {
                            //wk → w′k = wk − (η/m)*∑∂CXj/∂wk
                            weights[l - 1][n][w] = weights[l - 1][n][w] - (eta / miniBatchData.size()) * nabla_w[l - 1][n][w];
                        }
                    }
                }
            }
        }

        qDebug("Epoch %d complete", e+1);
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

    string miniBatchQualifiers;

    for (int i = 0; i < miniBatchSize; i++)
    {
        string trainingSampleFileName = trainingSampleFileNames[i];
        string trainingSampleFilePath = getTrainingObjectsFolderName() + "/" + trainingSampleFileName;
        //getting sample grayscale vector
        vector<double> trainingSampleVector = getGrayscaleVectorFromMatFile(trainingSampleFilePath);
        miniBatchData.push_back(trainingSampleVector);

//        string stringifiedSampleVector;
//        for (unsigned int s = 0; s < trainingSampleVector.size(); s++)
//        {
//            stringifiedSampleVector += format_string("%e ", trainingSampleVector[s]);
//        }
//        qDebug("S[%d][ %s]", (int)trainingSampleVector.size(), stringifiedSampleVector.c_str());

        int sampleQualifierRespectiveNeuronIdx = atoi(trainingSampleFileName.substr(0, trainingSampleFileName.find('-')).c_str());
        miniBatchQualifiers += format_string("%d ", sampleQualifierRespectiveNeuronIdx);
        vector<double> sampleQualifier;
        //oli - output layer neuron index
        for (int oli = 0; oli < layers[layers.size() - 1]; oli++)
        {
            sampleQualifier.push_back((sampleQualifierRespectiveNeuronIdx == oli) ? 1.0 : 0.0);
        }
        miniBatchQualifier.push_back(sampleQualifier);

//        string stringifiedSampleQualifier;
//        for (unsigned int s = 0; s < sampleQualifier.size(); s++)
//        {
//            stringifiedSampleQualifier += format_string("%.1f ", sampleQualifier[s]);
//        }
//        qDebug("Q[ %s]", stringifiedSampleQualifier.c_str());

//        qDebug("Training sample [%d] with size %d", sampleQualifierRespectiveNeuronIdx, (int)trainingSampleVector.size());
    }
    qDebug("Q[ %s]", miniBatchQualifiers.c_str());
}

void NeuralNetwork::prepareMiniBatch(vector<string> miniBatchSampleFileNames)
{
    miniBatchData.clear();
    miniBatchQualifier.clear();

    for (unsigned int i = 0 ; i < miniBatchSampleFileNames.size(); i++)
    {
        string trainingSampleFileName = miniBatchSampleFileNames[i];
        int sampleQualifierRespectiveNeuronIdx = atoi(trainingSampleFileName.substr(0, trainingSampleFileName.find('-')).c_str());

        vector<double> sampleQualifier;
        //oli - output layer neuron index
        for (int oli = 0; oli < layers[layers.size() - 1]; oli++)
        {
            sampleQualifier.push_back((sampleQualifierRespectiveNeuronIdx == oli) ? 1.0 : 0.0);
        }
        miniBatchQualifier.push_back(sampleQualifier);

        //string trainingSampleFilePath = getTrainingObjectsFolderName() + "/" + trainingSampleFileName;
        //getting sample grayscale vector
        //vector<double> trainingSampleVector = getGrayscaleVectorFromMatFile(trainingSampleFilePath);
        //miniBatchData.push_back(trainingSampleVector);

        //getting sample grayscale vector from trainigSamples map
        miniBatchData.push_back(trainingSamples[trainingSampleFileName]);
    }
}

void NeuralNetwork::backpropagation(std::vector<double> sampleData,
                                    std::vector<double> sampleQualifier,
                                    BIASES &d_nabla_b,
                                    WEIGHTS &d_nabla_w)
{
    //feedforward
    vector<double> activation = sampleData; //output of the layer, input for the next layer
    ACTIVATIONS activations;//activations by layer (for backpropagation)
    activations.push_back(activation);

    vector<vector<double>> zs;

    for (unsigned int l = 1; l < layers.size(); l++)
    {
        vector<vector<double>> neuronLayerWeights = weights[l - 1];
        vector<double> neuronLayerBiases = biases[l - 1];
        vector<double> zl; //input for sigmoid function of each neuron in the layer (weighted input)
        vector<double> al; //output of neurons (zl passed through sigmoig) for the layer, al=σ(zl)
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

    //backward pass
    //δL=∇aC⊙σ′(zL)
    vector<double> delta;
    if (getCostFunction() == "QuadraticCost")
    {
        delta = QuadraticCost(activations[(activations.size() - 1)], sampleQualifier, zs[(zs.size() - 1)]);
    }
    if (getCostFunction() == "CrossEntropyCost")
    {
        delta = CrossEntropyCost(activations[(activations.size() - 1)], sampleQualifier);
    }

    d_nabla_b[d_nabla_b.size() - 1] = delta;
    vector<vector<double>> dwl;
    for (unsigned i = 0; i < delta.size(); i++)
    {
        vector<double> dwln;
        for (unsigned int j = 0; j < ((vector<double>)activations[(activations.size() - 1) - 1]).size(); j++)
        {
            dwln.push_back(delta[i] * ((vector<double>)activations[(activations.size() - 1) - 1])[j]);
        }
        dwl.push_back(dwln);
    }
    d_nabla_w[d_nabla_w.size() - 1] = dwl;  //∂C∂w=ain*δout

    for (unsigned int l = (layers.size() - 2); l > 0; l--)
    {
        vector<double> zl = zs[l - 1];
        vector<double> sp = sigmoidPrime(zl);

        vector<vector<double>> weightsT = matrix2dTranspose(weights[l]);
        vector<double> weightsDotDelta = dotProductMatrixXVector(weightsT, delta);
        //δl=((w_l+1)Tδ_l+1)⊙σ′(zl)
        delta = HadamarProduct(weightsDotDelta, sp);
        d_nabla_b[l - 1] = delta;
        d_nabla_w[l - 1] = dotProductM(delta, activations[(activations.size() - 1) - l - 1]);
    }
}

vector<double> NeuralNetwork::feedForward(std::vector<double> inputActivation)
{
    vector<double> activation = inputActivation;

    for (unsigned int l = 1; l < layers.size(); l++)
    {
        vector<vector<double>> neuronLayerWeights = weights[l - 1];
        vector<double> neuronLayerBiases = biases[l - 1];
        vector<double> al; //output of neurons (zl passed through sigmoig) for the layer, al=σ(zl)
        for (int n = 0; n < layers[l]; n++)
        {
            vector<double> neuronWeights = neuronLayerWeights[n];
            double zn = dotProduct(activation, neuronWeights) + neuronLayerBiases[n];
            al.push_back(sigmoid(zn));
        }
        activation = al;
    }
    return activation; //activation of output layer
}

vector<double> NeuralNetwork::QuadraticCost(vector<double> activation, vector<double> sampleQualifier, vector<double> zl)
{
    return HadamarProduct(costDerivative(activation, sampleQualifier), sigmoidPrime(zl));
}

vector<double> NeuralNetwork::CrossEntropyCost(vector<double> activation, vector<double> sampleQualifier)
{
    vector<double> delta;
    for (unsigned int i = 0; i < sampleQualifier.size(); i++)
    {
        delta.push_back(activation[i] - sampleQualifier[i]);
    }
    return delta;
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

void NeuralNetwork::zeroNablas(BIASES &nabla_b, WEIGHTS &nabla_w)
{
    for (unsigned int l = 1; l < layers.size(); l++)
    {
        vector<double> layerNeuronBiases;
        for (int n = 0; n < layers[l]; n++)
        {
            layerNeuronBiases.push_back(0.0);
        }
        nabla_b.push_back(layerNeuronBiases);

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
        nabla_w.push_back(layerNeuronWeights);
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
//                qDebug("Training Sample: %-20s, Output Neuron ID: %d", trainingSampleFileName.c_str(), outputNeuronNum);
            }
          }
       }

       trainingDataLength = (int)trainingSamplesFileName.size();

       qDebug("Samples files read: %d", trainingDataLength);
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
    qDebug("Layers: %d", (int)layers.size());
    for (unsigned int l = 0; l < layers.size(); l++)
    {
        qDebug("L%d[%d]", l, layers[l]);
    }
}

int NeuralNetwork::getMiniBatchSize() const
{
    return miniBatchSize;
}

void NeuralNetwork::setMiniBatchSize(int value)
{
    miniBatchSize = value;
    qDebug("Batch Size: %d", miniBatchSize);
}

int NeuralNetwork::getEpochs() const
{
    return epochs;
}

void NeuralNetwork::setEpochs(int value)
{
    epochs = value;
    qDebug("Epochs: %d", epochs);
}

double NeuralNetwork::getEta() const
{
    return eta;
}

void NeuralNetwork::setEta(double value)
{
    eta = value;
    qDebug("Eta: %.10f", eta);
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

string NeuralNetwork::getCostFunction() const
{
    return costFunction;
}

void NeuralNetwork::setCostFunction(const string &value)
{
    costFunction = value;
    qDebug("Cost Function: %s", costFunction.c_str());
}

bool NeuralNetwork::getRotateSamples() const
{
    return rotateSamples;
}

void NeuralNetwork::setRotateSamples(bool value)
{
    rotateSamples = value;
}
