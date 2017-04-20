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

                std::this_thread::sleep_for(std::chrono::milliseconds(getCreateTrainingObjectsShowDelay()));
            }
            angle += 180 / getTrainingSamplesNumber();
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
