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

            cv::Mat preprocessedFrame;

            if (getBrightnessPreview())
            {
                cv::Mat inputImage = videoFrame.clone();
                cv::Mat resultImage = cv::Mat::zeros(inputImage.size(), inputImage.type());

                for( int y = 0; y < inputImage.rows; y++ )
                {
                    for( int x = 0; x < inputImage.cols; x++ )
                    {
                        for( int c = 0; c < 3; c++ )
                        {
                            resultImage.at<cv::Vec3b>(y,x)[c] = cv::saturate_cast<uchar>(getPositiveImageProcessingContrast() * (inputImage.at<cv::Vec3b>(y,x)[c]) + getPositiveImageProcessingBrightness());
                        }
                    }
                }

                cv::cvtColor(resultImage, preprocessedFrame, CV_BGR2GRAY);
            }
            else
            {
                cv::cvtColor(videoFrame, preprocessedFrame, CV_BGR2GRAY);
            }

            if (getGaussianBlurPreview())
            {
                cv::Mat resultImage = preprocessedFrame.clone();
                for ( int i = 1; i < getPositiveImageProcessingGaussianBlur(); i = i + 2 )
                {
                    cv::GaussianBlur(preprocessedFrame, resultImage, cv::Size(i, i), 0, 0);
                }
                preprocessedFrame = resultImage.clone();
            }

            cv::Mat grayNonThresholded = preprocessedFrame.clone();

            if (getThresholdPreview())
            {
                cv::Mat thresholdedFrame = cv::Mat::zeros(preprocessedFrame.size(), preprocessedFrame.type());
                cv::threshold(preprocessedFrame, thresholdedFrame, positiveImageProcessingThreshold, 255, cv::THRESH_BINARY);
                cv::imshow("Thresholded video frames", thresholdedFrame);
                cv::Mat cannyResultFromThresholded;
                cv::Canny(thresholdedFrame, cannyResultFromThresholded, getPositiveImageProcessingCannyThreshold(), getPositiveImageProcessingCannyThreshold()*2, 3);
                cv::imshow("Canny result from thresholded", cannyResultFromThresholded);
            }

            cv::Mat cannyResultFromNonThresholded;
            cv::Canny(grayNonThresholded, cannyResultFromNonThresholded, getPositiveImageProcessingCannyThreshold(), getPositiveImageProcessingCannyThreshold()*2, 3);

            if (!getCropPositiveImagesRealTime())
            {
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
            }

            vector<vector<cv::Point>> contours;
            vector<cv::Vec4i> hierarchy;

            /// Find contours
            cv::findContours(cannyResultFromNonThresholded, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));

            /// Approximate contours to polygons + get bounding rects and circles
            vector<vector<cv::Point>> contoursPoly(contours.size());
            vector<cv::Rect> boundRect(contours.size());
            vector<cv::Point2f> center(contours.size());
            vector<float> radius(contours.size());

            for(unsigned int i = 0; i < contours.size(); i++ )
            {
                cv::approxPolyDP(cv::Mat(contours[i]), contoursPoly[i], 1, true);
                boundRect[i] = cv::boundingRect(cv::Mat(contoursPoly[i]));
                //minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
            }

            /// Draw polygonal contour + bonding rects + circles
            cv::Mat cannyResultFromNonThresholdedContours = cv::Mat::zeros(cannyResultFromNonThresholded.size(), CV_8UC3);
            cv::RNG rng(12345);

            vector<vector<cv::Point>> majorContours;

            for(unsigned int i = 0; i< contours.size(); i++)
            {
                if (((cv::Rect)boundRect[i]).area() > getMinContourArea())
                {
                    cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
                    cv::drawContours(cannyResultFromNonThresholdedContours, contoursPoly, i, color, 1, 8, vector<cv::Vec4i>(), 0, cv::Point());
                    cv::rectangle(cannyResultFromNonThresholdedContours, boundRect[i].tl(), boundRect[i].br(), color, 2, cv::LINE_8, 0);
                    majorContours.push_back(contours[i]);
                }
            }

            cv::Mat grayFrame;
            cv::cvtColor(videoFrame, grayFrame, CV_BGR2GRAY);

            if (majorContours.size() > 0)
            {
                vector<cv::Point> convexHullPoints = contoursConvexHull(majorContours);
                cv::Mat convexHullsImage = grayFrame.clone();
                if (convexHullPoints.size() > 0)
                {
                    vector<cv::Point> ROIVertices;
                    cv::approxPolyDP(convexHullPoints, ROIVertices, 0.1, false);

                    cv::polylines(convexHullsImage, ROIVertices, true, cv::Scalar(255, 255, 255), 1);
                    cv::Rect majorContourBoundingRect = cv::boundingRect(cv::Mat(ROIVertices));
                    if (majorContourBoundingRect.area() > getMinContourArea())
                    {
                        if (getCropPositiveImagesRealTime())
                        {
                            majorContourBoundingRect.x -= 5;
                            majorContourBoundingRect.y -= 5;
                            majorContourBoundingRect.width += 10;
                            majorContourBoundingRect.height += 10;

                            if (majorContourBoundingRect.x > 0 &&
                                majorContourBoundingRect.y > 0 &&
                                (majorContourBoundingRect.x + majorContourBoundingRect.width < convexHullsImage.cols) &&
                                (majorContourBoundingRect.y + majorContourBoundingRect.height < convexHullsImage.rows))
                            {
                                cv::rectangle(convexHullsImage, majorContourBoundingRect.tl(), majorContourBoundingRect.br(), cv::Scalar(0, 0, 0), 2, cv::LINE_8, 0);
                                cv::imshow("Contours Convex Hull", convexHullsImage);


                                cv::Mat positiveImageMask = cv::Mat::zeros(grayFrame.size(), CV_8UC1);
                                cv::fillConvexPoly(positiveImageMask, &ROIVertices[0], ROIVertices.size(), cv::Scalar(255, 255, 255), 8, 0);

                                cv::Mat maskedPositiveImage = cv::Mat::zeros(grayFrame.size(), CV_8U);
                                grayFrame.copyTo(maskedPositiveImage, positiveImageMask);

                                cv::Mat croppedPositiveImage = maskedPositiveImage(majorContourBoundingRect);

                                double aspectRatio = (double)croppedPositiveImage.cols / (double)croppedPositiveImage.rows;
                                cv::Mat croppedPositiveResizedImage;
                                cv::resize(croppedPositiveImage, croppedPositiveResizedImage, cv::Size(getCroppedWidth(), round(getCroppedWidth() / aspectRatio)), 0, 0, cv::INTER_CUBIC);

                                cv::Mat finalPositiveImage = cv::Mat::zeros(getCroppedWidth(), getCroppedWidth(), CV_8U);

                                double fitX = (finalPositiveImage.cols - croppedPositiveResizedImage.cols)/2;
                                double fitY = (finalPositiveImage.rows - croppedPositiveResizedImage.rows)/2;

                                if (fitX >= 0 &&
                                    fitY >= 0 &&
                                    (fitX + croppedPositiveResizedImage.cols <= finalPositiveImage.cols) &&
                                    (fitY + croppedPositiveResizedImage.rows <= finalPositiveImage.rows))
                                {
                                    croppedPositiveResizedImage.copyTo(finalPositiveImage(cv::Rect(fitX, fitY, croppedPositiveResizedImage.cols, croppedPositiveResizedImage.rows)));

                                    if (getShowCroppedPositives())
                                    {
                                        cv::imshow("Cropped Resized Positive Image", finalPositiveImage);
                                        cv::imshow("Cropped Positive Image", croppedPositiveImage);
                                    }

                                    if (getCapturePositive() && !finalPositiveImage.empty())
                                    {
                                        string positiveCroppedFilePath = positiveImagesFolder + "/" + format_string("pos%05d.jpg", positiveCroppedInfo.size());
                                        string positiveCroppedInfoElement = format_string("positives/pos%05d.jpg 1 %d %d %d %d",
                                                                                         positiveCroppedInfo.size(),
                                                                                         0,
                                                                                         0,
                                                                                         finalPositiveImage.cols,
                                                                                         finalPositiveImage.cols);
                                        positiveCroppedInfo.push_back(positiveCroppedInfoElement);

                                        cv::imwrite(positiveCroppedFilePath, finalPositiveImage);
                                    }
                                }
                            }
                            else
                            {
                                cv::rectangle(convexHullsImage, majorContourBoundingRect.tl(), majorContourBoundingRect.br(), cv::Scalar(0, 0, 0), 1, cv::LINE_8, 0);
                                cv::imshow("Contours Convex Hull", convexHullsImage);
                            }
                        }
                        else
                        {
                            cv::rectangle(convexHullsImage, majorContourBoundingRect.tl(), majorContourBoundingRect.br(), cv::Scalar(0, 0, 0), 1, cv::LINE_8, 0);
                            cv::imshow("Contours Convex Hull", convexHullsImage);
                        }
                    }
                }
            }

//            cv::imshow("Video frames", videoFrame);
            cv::imshow("Processed video frames", preprocessedFrame);
            cv::imshow("Canny result from non-thresholded", cannyResultFromNonThresholded);
//            cv::imshow("Canny result from non-thresholded (contours)", cannyResultFromNonThresholdedContours);


            if (capturePositive || captureNegative)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
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
    if (!getCropPositiveImagesRealTime())
    {
        positiveImagesProcessingThread = new std::thread(&CascadeClassifier::positiveImagesProcessingWorker, this);
    }
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
            cv::approxPolyDP(cv::Mat(contours[i]), contoursPoly[i], 3, true);
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
                if (((cv::Rect)boundRect[i]).width < positiveImage.cols && ((cv::Rect)boundRect[i]).height < positiveImage.rows)
                {
                    largestContourArea = contourArea;
                    targetObjectContourindex = i;               //Store the index of largest contour
                }
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
                    if (((cv::Rect)boundRect[targetObjectContourindex]).width > getCroppedWidth() / 2 || ((cv::Rect)boundRect[targetObjectContourindex]).height > getCroppedHeight() / 2)
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

void CascadeClassifier::processTrainingSamples()
{
    trainingSamplesProcessingThread = new std::thread(&CascadeClassifier::trainingSamplesProcessingWorker, this);
}

void CascadeClassifier::trainingSamplesProcessingWorker()
{
    cv::Mat positiveImage;
    cv::Mat negativeImageROI;
    for(unsigned int i = 0; i < positiveFileNames.size(); ++i)
    {
        qDebug("%s", ((string)positiveFileNames[i]).c_str());
        positiveImage = cv::imread(positiveFileNames[i], CV_8U);
        cv::imshow("Source Positive Image", positiveImage);

        cv::RNG rng(12345);
        int randNegativeImageFileIndex = rand() % negativeFileNames.size();
        string randNegativeImageFile = negativeFileNames[randNegativeImageFileIndex];
        cv::Mat negativeImage = cv::imread(randNegativeImageFile, CV_8U);
        int negativeImageROIWidth  = (int)round(positiveImage.cols * 1.15);
        int negativeImageROIHeight = (int)round(positiveImage.rows * 1.15);
        if (negativeImageROIWidth > negativeImage.cols || negativeImageROIHeight > negativeImage.rows)
        {
            i--;
            continue;
        }
        cv::Rect negativeImageROIRect(rng.uniform(0, negativeImage.cols - negativeImageROIWidth), rng.uniform(0, negativeImage.rows - negativeImageROIHeight), negativeImageROIWidth, negativeImageROIHeight);
        negativeImageROI = negativeImage(negativeImageROIRect);
        cv::imshow("Source Negative Image ROI", negativeImageROI);

        cv::Mat processedPositiveImage = positiveImage.clone();
        for (int i = 1; i < getPositiveImageProcessingGaussianBlur(); i = i + 2)
        {
            cv::GaussianBlur(positiveImage, processedPositiveImage, cv::Size(i, i), 0, 0);
        }
        cv::imshow("Processed Positive Image", processedPositiveImage);

        cv::Mat cannyResultProcessedPositiveImage;
        cv::Canny(processedPositiveImage, cannyResultProcessedPositiveImage, getPositiveImageProcessingCannyThreshold(), getPositiveImageProcessingCannyThreshold()*2, 3);

        cv::imshow("Canny Result from processed Positive Image", cannyResultProcessedPositiveImage);

        /// Getting majour contour
        vector<vector<cv::Point>> contours;
        vector<cv::Vec4i> hierarchy;

        /// Find contours
        cv::findContours(cannyResultProcessedPositiveImage, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));

        /// Approximate contours to polygons + get bounding rects and circles
        vector<vector<cv::Point>> contoursPoly(contours.size());
        vector<cv::Rect> boundRect(contours.size());

        for(unsigned int i = 0; i < contours.size(); i++ )
        {
            cv::approxPolyDP(cv::Mat(contours[i]), contoursPoly[i], 1, true);
            boundRect[i] = cv::boundingRect(cv::Mat(contoursPoly[i]));
        }

        vector<vector<cv::Point>> majorContours;
        for(unsigned int i = 0; i < contours.size(); i++)
        {
            if (((cv::Rect)boundRect[i]).area() > getMinContourArea())
            {
                majorContours.push_back(contours[i]);
            }
        }

        if (majorContours.size() > 0)
        {
            vector<cv::Point> convexHullPoints = contoursConvexHull(majorContours);
            cv::Mat convexHullsImage = positiveImage.clone();
            if (convexHullPoints.size() > 0)
            {
                vector<cv::Point> ROIVertices;
                cv::approxPolyDP(convexHullPoints, ROIVertices, 0.1, false);

                //cv::polylines(convexHullsImage, ROIVertices, true, cv::Scalar(255, 255, 255), 1);

                cv::Rect majorContourBoundingRect = cv::boundingRect(cv::Mat(ROIVertices));
                if (majorContourBoundingRect.area() > getMinContourArea())
                {
                    cv::Mat sampleImageMask = cv::Mat::zeros(positiveImage.size(), CV_8UC1);
                    cv::fillConvexPoly(sampleImageMask, &ROIVertices[0], ROIVertices.size(), cv::Scalar(255, 255, 255), cv::LINE_8, 0);
                    cv::erode(sampleImageMask, sampleImageMask, cv::Mat(), cv::Point(-1, -1), 2);

                    cv::Mat finalSampleImage = negativeImageROI.clone();
                    cv::Rect finalSamplePositiveROI = cv::Rect((negativeImageROI.cols - sampleImageMask.cols) / 2,
                                                               (negativeImageROI.rows - sampleImageMask.rows) / 2,
                                                                sampleImageMask.cols, sampleImageMask.rows);
                    positiveImage.copyTo(finalSampleImage(finalSamplePositiveROI), sampleImageMask);

                    cv::imshow("Final Sample Image", finalSampleImage);

                    if (!getCreateTrainingSamplesPreview())
                    {
                        if (!getSamplesFolder().empty())
                        {
                            std::string sampleFileName = format_string("%s/sample%05d.jpg", samplesFolder.c_str(), i);
                            cv::imwrite(sampleFileName, finalSampleImage);
                        }
                    }
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(getPositiveImageProcessingDelay()));
    }
}

void CascadeClassifier::readPositiveImagesDir()
{
    DIR *dpdf;
    struct dirent *epdf;
    dpdf = opendir(positiveImagesFolder.c_str());
    if (dpdf != NULL)
    {
       while ((epdf = readdir(dpdf)))
       {
          if (std::string(epdf->d_name) != "." && std::string(epdf->d_name) != "..")
          {
            string positiveFileName = positiveImagesFolder + "/" + std::string(epdf->d_name);
            positiveFileNames.push_back(positiveFileName);
            qDebug("Positive Image: %s", positiveFileName.c_str());
          }
       }
    }
}

void CascadeClassifier::readNegativeImagesDir()
{
    DIR *dpdf;
    struct dirent *epdf;
    dpdf = opendir(negativeImagesFolder.c_str());
    if (dpdf != NULL)
    {
       while ((epdf = readdir(dpdf)))
       {
          if (std::string(epdf->d_name) != "." && std::string(epdf->d_name) != "..")
          {
            string negativeFileName = negativeImagesFolder + "/" + std::string(epdf->d_name);
            negativeFileNames.push_back(negativeFileName);
            qDebug("Negative Image: %s", negativeFileName.c_str());
          }
       }
    }
}

void CascadeClassifier::savePositiveCroppedInfo()
{
    for(size_t i=0; i< positiveCroppedInfo.size(); i++)
    {
       qDebug("%s", ((string)positiveCroppedInfo[i]).c_str());
    }
    ofstream positiveCroppedInfoFile;
    positiveCroppedInfoFile.open(positiveImagesFolder + "/collection.info");
    std::ostream_iterator<std::string> outputCroppedIterator(positiveCroppedInfoFile, "\n");
    std::copy(positiveCroppedInfo.begin(), positiveCroppedInfo.end(), outputCroppedIterator);
    positiveCroppedInfoFile.close();
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

bool CascadeClassifier::getThresholdPreview() const
{
    return thresholdPreview;
}

void CascadeClassifier::setThresholdPreview(bool value)
{
    thresholdPreview = value;
}

bool CascadeClassifier::getGaussianBlurPreview() const
{
    return gaussianBlurPreview;
}

void CascadeClassifier::setGaussianBlurPreview(bool value)
{
    gaussianBlurPreview = value;
}

bool CascadeClassifier::getBrightnessPreview() const
{
    return brightnessPreview;
}

void CascadeClassifier::setBrightnessPreview(bool value)
{
    brightnessPreview = value;
}

int CascadeClassifier::getPositiveImageProcessingBrightness() const
{
    return positiveImageProcessingBrightness;
}

void CascadeClassifier::setPositiveImageProcessingBrightness(int value)
{
    positiveImageProcessingBrightness = value;
}

double CascadeClassifier::getPositiveImageProcessingContrast() const
{
    return positiveImageProcessingContrast;
}

void CascadeClassifier::setPositiveImageProcessingContrast(double value)
{
    positiveImageProcessingContrast = value;
}

int CascadeClassifier::getPositiveImageProcessingGaussianBlur() const
{
    return positiveImageProcessingGaussianBlur;
}

void CascadeClassifier::setPositiveImageProcessingGaussianBlur(int value)
{
    positiveImageProcessingGaussianBlur = value;
}

int CascadeClassifier::getPositiveImageProcessingCannyThreshold() const
{
    return positiveImageProcessingCannyThreshold;
}

void CascadeClassifier::setPositiveImageProcessingCannyThreshold(int value)
{
    positiveImageProcessingCannyThreshold = value;
}

int CascadeClassifier::getMinContourArea() const
{
    return minContourArea;
}

void CascadeClassifier::setMinContourArea(int value)
{
    minContourArea = value;
}

bool CascadeClassifier::getCropPositiveImagesRealTime() const
{
    return cropPositiveImagesRealTime;
}

void CascadeClassifier::setCropPositiveImagesRealTime(bool value)
{
    cropPositiveImagesRealTime = value;
}

bool CascadeClassifier::getShowCroppedPositives() const
{
    return showCroppedPositives;
}

void CascadeClassifier::setShowCroppedPositives(bool value)
{
    showCroppedPositives = value;
}

bool CascadeClassifier::getCreateTrainingSamplesPreview() const
{
    return createTrainingSamplesPreview;
}

void CascadeClassifier::setCreateTrainingSamplesPreview(bool value)
{
    createTrainingSamplesPreview = value;
}

vector<string> CascadeClassifier::getNegativeFileNames() const
{
    return negativeFileNames;
}

void CascadeClassifier::setNegativeFileNames(const vector<string> &value)
{
    negativeFileNames = value;
}

vector<string> CascadeClassifier::getPositiveFileNames() const
{
    return positiveFileNames;
}

void CascadeClassifier::setPositiveFileNames(const vector<string> &value)
{
    positiveFileNames = value;
}

std::string CascadeClassifier::getSamplesFolder() const
{
    return samplesFolder;
}

void CascadeClassifier::setSamplesFolder(const std::string &value)
{
    samplesFolder = value;
}
