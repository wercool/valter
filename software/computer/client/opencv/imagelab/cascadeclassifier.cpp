#include "cascadeclassifier.h"

CascadeClassifier::CascadeClassifier()
{
    
}

void CascadeClassifier::captureVideoWorker()
{
    try
    {
        videoCapture = cv::VideoCapture(0); // open the default camera
        if(!videoCapture.isOpened())  // check if we succeeded
            return;
        videoCaptured = true;
        while (videoCaptured)
        {
            videoCapture.read(videoFrame);

            if (capturePositive)
            {
                cv::Mat grayPositiveImage;
                cv::cvtColor(videoFrame, grayPositiveImage, CV_BGR2GRAY);
                std::string fileName = format_string("%s/pos%05d.jpg", positiveImagesFolder.c_str(), positiveImageNum++);
                cv::imwrite(fileName, grayPositiveImage);
                qDebug("POSITIVE [%s]", fileName.c_str());
                cv::imshow("Video frames GRAY", grayPositiveImage);
            }
            if (captureNegative)
            {
                cv::Mat grayNegativeImage;
                cv::cvtColor(videoFrame, grayNegativeImage, CV_BGR2GRAY);
                std::string fileName = format_string("%s/neg%05d.jpg", negativeImagesFolder.c_str(), negativeImageNum++);
                cv::imwrite(fileName, grayNegativeImage);
                qDebug("NEGATIVE [%s]", fileName.c_str());
                cv::imshow("Video frames GRAY", grayNegativeImage);
            }

            if (capturePositive || captureNegative)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            cv::imshow("Video frames", videoFrame);
        }
    }
    catch (const std::exception& e)
    {
    }
}

void CascadeClassifier::processPositiveImagesToCollectionFile(bool crop)
{
    cropPositiveImages = crop;
    //ifstream positivesDescFile(positiveImagesFolder + "/positives.lst");
    //copy(istream_iterator<string>(positivesDescFile), istream_iterator<string>(), back_inserter(positiveFileNames));
    DIR *dpdf;
    struct dirent *epdf;
    dpdf = opendir(positiveImagesFolder.c_str());
    if (dpdf != NULL)
    {
       while ((epdf = readdir(dpdf)))
       {
          if (std::string(epdf->d_name) != "." && std::string(epdf->d_name) != ".." && std::string(epdf->d_name) != "collection.dat")
          {
            positiveFileNames.push_back(std::string(epdf->d_name));
            qDebug("%s", std::string(epdf->d_name).c_str());
          }
       }
    }
    std::sort(positiveFileNames.begin(), positiveFileNames.end());
    positiveImagesProcessingThread = new std::thread(&CascadeClassifier::positiveImagesProcessingWorker, this);
}

//http://docs.opencv.org/3.2.0/dc/d88/tutorial_traincascade.html
//http://note.sonots.com/SciSoftware/haartraining.html
//http://docs.opencv.org/2.4/doc/tutorials/imgproc/shapedescriptors/bounding_rects_circles/bounding_rects_circles.html
void CascadeClassifier::positiveImagesProcessingWorker()
{
    vector<string> positiveCollection;
    vector<string> positiveCroppedCollection;
    for(unsigned int i=0; i < positiveFileNames.size(); ++i)
    {
        cv::Mat src;
        cv::Mat positiveImage;
        cv::Mat thresholdPositiveImage;
        vector<vector<cv::Point>> contours;
        vector<cv::Vec4i> hierarchy;

        qDebug("%s/%s", positiveImagesFolder.c_str(), ((string)positiveFileNames[i]).c_str());
        string positiveFilePath = positiveImagesFolder + "/" + positiveFileNames[i];
        src = cv::imread(positiveFilePath, true);
        cv::cvtColor(src, positiveImage, CV_BGR2GRAY);

        /// Detect edges using Threshold
        cv::threshold(positiveImage, thresholdPositiveImage, positiveImageProcessingThreshold, 255, cv::THRESH_BINARY);
        /// Find contours
        cv::findContours(thresholdPositiveImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

        /// Approximate contours to polygons + get bounding rects and circles
        vector<vector<cv::Point>> contoursPoly(contours.size());
        vector<cv::Rect> boundRect(contours.size());
        vector<cv::Point2f> center(contours.size());
        vector<float> radius(contours.size());

        for(unsigned int i = 0; i < contours.size(); i++ )
        {
            approxPolyDP(cv::Mat(contours[i]), contoursPoly[i], 3, true);
            boundRect[i] = boundingRect(cv::Mat(contoursPoly[i]));
            //minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
        }

        /// Draw polygonal contour + bonding rects + circles
        cv::Mat drawing = cv::Mat::zeros(thresholdPositiveImage.size(), CV_8UC3 );
        cv::RNG rng(12345);

        for(unsigned int i = 0; i< contours.size(); i++)
        {
            cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
            cv::drawContours(drawing, contoursPoly, i, color, 1, 8, vector<cv::Vec4i>(), 0, cv::Point());
            cv::rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, cv::LINE_8, 0);
            //cv::circle(drawing, center[i], (int)radius[i], color, 2, 8, 0);
        }

        int largestContourArea = 0;
        int targetObjectContourindex = 0;
        for(size_t i = 1/* 0 - is a most outer contour */; i< contours.size(); i++)
        {
            double contourArea = cv::contourArea(contours[i]);
            if(contourArea > largestContourArea)
            {
                largestContourArea = contourArea;
                targetObjectContourindex = i;               //Store the index of largest contour
            }
        }

        if (((cv::Rect)boundRect[targetObjectContourindex]).width < positiveImage.cols && ((cv::Rect)boundRect[targetObjectContourindex]).height < positiveImage.rows)
        {
            cv::rectangle(positiveImage, boundRect[targetObjectContourindex].tl(), boundRect[targetObjectContourindex].br(), cv::Scalar(0, 255, 0), 2, cv::LINE_8, 0);

            string positiveCollectionElement = format_string("%s 1 %d %d %d %d",
                                                             ((string)positiveFileNames[i]).c_str(),
                                                             ((cv::Rect)boundRect[targetObjectContourindex]).x,
                                                             ((cv::Rect)boundRect[targetObjectContourindex]).y,
                                                             ((cv::Rect)boundRect[targetObjectContourindex]).width,
                                                             ((cv::Rect)boundRect[targetObjectContourindex]).height);
            positiveCollection.push_back(positiveCollectionElement);

            if (cropPositiveImages)
            {
                cv::Mat croppedPositiveImage = positiveImage(boundRect[targetObjectContourindex]);
                string positiveCroppedFilePath = positiveImagesFolder + "/cropped_" + positiveFileNames[i];

                if (getCroppedWidth() != 0 && getCroppedHeight() != 0)
                {
                    cv::Mat croppedPositiveResizedImage;
                    cv::resize(croppedPositiveImage, croppedPositiveResizedImage, cv::Size(getCroppedWidth(), getCroppedHeight()), 0, 0, cv::INTER_CUBIC);
                    cv::imwrite(positiveCroppedFilePath, croppedPositiveResizedImage);
                    cv::imshow("Cropped Resized Positive Image", croppedPositiveResizedImage);

                    string positiveCroppedCollectionElement = format_string("%s 1 %d %d %d %d",
                                                                     ((string)("cropped_" + positiveFileNames[i])).c_str(),
                                                                     0,
                                                                     0,
                                                                     getCroppedWidth(),
                                                                     getCroppedHeight());
                    positiveCroppedCollection.push_back(positiveCroppedCollectionElement);
                }
                else
                {
                    cv::imwrite(positiveCroppedFilePath, croppedPositiveImage);
                    cv::imshow("Cropped Positive Image", croppedPositiveImage);
                }
            }
        }
        cv::imshow("Positive Image", positiveImage);
        cv::imshow("Positive Image Threshold", thresholdPositiveImage);
        cv::imshow("Positive Image Contours", drawing);
        std::this_thread::sleep_for(std::chrono::milliseconds(getPositiveImageProcessingDelay()));
    }

    for(size_t i=0; i< positiveCroppedCollection.size(); i++)
    {
       qDebug("%s", ((string)positiveCroppedCollection[i]).c_str());
    }
    ofstream collectionCroppedFile;
    collectionCroppedFile.open(positiveImagesFolder + "/collection_cropped.dat");
    std::ostream_iterator<std::string> outputCroppedIterator(collectionCroppedFile, "\n");
    std::copy(positiveCroppedCollection.begin(), positiveCroppedCollection.end(), outputCroppedIterator);
    collectionCroppedFile.close();


    qDebug("Collection file:");
    for(size_t i=0; i< positiveCollection.size(); i++)
    {
       qDebug("%s", ((string)positiveCollection[i]).c_str());
    }
    ofstream collectionFile;
    collectionFile.open(positiveImagesFolder + "/collection.dat");
    std::ostream_iterator<std::string> outputIterator(collectionFile, "\n");
    std::copy(positiveCollection.begin(), positiveCollection.end(), outputIterator);
    collectionFile.close();
}

void CascadeClassifier::startDetection()
{
    objectDetectionThread = new std::thread(&CascadeClassifier::objectDetectionWorker, this);
}

void CascadeClassifier::stopDetection()
{
    videoCapture.release();
    videoCaptured = false;
    cv::destroyWindow("Obejct detection");
}

void CascadeClassifier::objectDetectionWorker()
{
    try
    {
        videoCapture = cv::VideoCapture(0); // open the default camera
        if(!videoCapture.isOpened())  // check if we succeeded
            return;
        videoCaptured = true;
        while (videoCaptured)
        {
            cv::Mat frameGray;
            cv::Mat displayResult;
            videoCapture.read(displayResult);

            cv::cvtColor(displayResult, frameGray, cv::COLOR_BGR2GRAY);
            cv::equalizeHist(frameGray, frameGray);

            std::vector<cv::Rect> foundObjects;

            objectCascade.detectMultiScale(frameGray, foundObjects, 1.1, 2, 0|cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));

            for ( size_t i = 0; i < foundObjects.size(); i++ )
            {
               // cv::Mat objectROI = frameGray(foundObjects[i]);
                cv::rectangle(displayResult, ((cv::Rect)foundObjects[i]).tl(), ((cv::Rect)foundObjects[i]).br(), cv::Scalar(0, 255, 0), 2, cv::LINE_8, 0);
            }

            cv::imshow("Obejct detection", displayResult);
        }
    }
    catch (const std::exception& e)
    {
    }
}

void CascadeClassifier::captureVideo()
{
    captureVideoThread = new std::thread(&CascadeClassifier::captureVideoWorker, this);
}

void CascadeClassifier::stopVideo()
{
    videoCapture.release();
    videoCaptured = false;
    cv::destroyWindow("Video frames");
}

string CascadeClassifier::getPositiveImagesFolder() const
{
    return positiveImagesFolder;
}

void CascadeClassifier::setPositiveImagesFolder(const string &value)
{
    positiveImagesFolder = value;
}

string CascadeClassifier::getNegativeImagesFolder() const
{
    return negativeImagesFolder;
}

void CascadeClassifier::setNegativeImagesFolder(const string &value)
{
    negativeImagesFolder = value;
}

bool CascadeClassifier::getCapturePositive() const
{
    return capturePositive;
}

void CascadeClassifier::setCapturePositive(bool value)
{
    capturePositive = value;
}

bool CascadeClassifier::getCaptureNegative() const
{
    return captureNegative;
}

void CascadeClassifier::setCaptureNegative(bool value)
{
    captureNegative = value;
}

int CascadeClassifier::getPositiveImageProcessingDelay() const
{
    return positiveImageProcessingDelay;
}

void CascadeClassifier::setPositiveImageProcessingDelay(int value)
{
    positiveImageProcessingDelay = value;
}

int CascadeClassifier::getPositiveImageProcessingThreshold() const
{
    return positiveImageProcessingThreshold;
}

void CascadeClassifier::setPositiveImageProcessingThreshold(int value)
{
    positiveImageProcessingThreshold = value;
}

std::string CascadeClassifier::getCascadeClassifierFile() const
{
    return cascadeClassifierFile;
}

void CascadeClassifier::setCascadeClassifierFile(const std::string &value)
{
    cascadeClassifierFile = value;
    if( !objectCascade.load(cascadeClassifierFile) ){ qDebug("--(!)Error loading face cascade\n"); };
}

int CascadeClassifier::getCroppedWidth() const
{
    return croppedWidth;
}

void CascadeClassifier::setCroppedWidth(int value)
{
    croppedWidth = value;
}

int CascadeClassifier::getCroppedHeight() const
{
    return croppedHeight;
}

void CascadeClassifier::setCroppedHeight(int value)
{
    croppedHeight = value;
}
