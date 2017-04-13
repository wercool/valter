#include "imagemanipulator.h"

ImageManipulator::ImageManipulator()
{

}

void ImageManipulator::preProcess()
{
    changeBrightnessAndContrast();
    applyNormalizedBoxBlur();
    applyHomogeneousBlur();
    applyGaussianBlur();
    applyMedianBlur();
    applyBilateralBlur();
}

void ImageManipulator::captureVideoWorker()
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

            if (!objectImage.empty())
            {
                int minHessian = getMinHessian();
                cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(minHessian);
                detector->detect(videoFrame, sceneKeypoints);
                cv::drawKeypoints(videoFrame, sceneKeypoints, videoFrameWithKeypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);

                cv::Ptr<cv::xfeatures2d::SURF> extractor = cv::xfeatures2d::SURF::create(minHessian);

                cv::Mat descriptorsObject, descriptorsScene;

                extractor->compute(objectImage, keypoints, descriptorsObject);
                extractor->compute(videoFrame, sceneKeypoints, descriptorsScene);

                cv::Ptr<cv::FlannBasedMatcher> matcher = cv::FlannBasedMatcher::create();
                std::vector<cv::DMatch> matches;
                matcher->match(descriptorsObject, descriptorsScene, matches);

                double max_dist = 0;
                double min_dist = 200;

                //-- Quick calculation of max and min distances between keypoints
                for(int i = 0; i < descriptorsObject.rows; i++)
                {
                    double dist = matches[i].distance;
                    if(dist < min_dist) min_dist = dist;
                    if(dist > max_dist) max_dist = dist;
                }

                std::vector<cv::DMatch> goodMatches;
                for(int i = 0; i < descriptorsObject.rows; i++)
                {
                    if(matches[i].distance < 3 * min_dist)
                    {
                        goodMatches.push_back(matches[i]);
                    }
                }
                std::vector<cv::Point2f> obj;
                std::vector<cv::Point2f> scene;

                for(unsigned int i = 0; i < goodMatches.size(); i++)
                {
                    //-- Get the keypoints from the good matches
                    obj.push_back(keypoints[goodMatches[i].queryIdx].pt);
                    scene.push_back(sceneKeypoints[goodMatches[i].trainIdx].pt);
                }

                cv::Mat H = cv::findHomography(obj, scene, CV_RANSAC);

                if (!H.empty())
                {
                    //-- Get the corners from the image_1 ( the object to be "detected" )
                    std::vector<cv::Point2f> objCorners(4);
                    objCorners[0] = cv::Point(0, 0);
                    objCorners[1] = cv::Point(objectImage.cols, 0);
                    objCorners[2] = cv::Point(objectImage.cols, objectImage.rows);
                    objCorners[3] = cv::Point(0, objectImage.rows);
                    std::vector<cv::Point2f> sceneCorners(4);

                    cv::perspectiveTransform(objCorners, sceneCorners, H);

                    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
                    cv::line(videoFrame, sceneCorners[0], sceneCorners[1], cv::Scalar(0, 255, 0), 4);
                    cv::line(videoFrame, sceneCorners[1], sceneCorners[2], cv::Scalar( 0, 255, 0), 4);
                    cv::line(videoFrame, sceneCorners[2], sceneCorners[3], cv::Scalar( 0, 255, 0), 4);
                    cv::line(videoFrame, sceneCorners[3], sceneCorners[0], cv::Scalar( 0, 255, 0), 4);
                }
                cv::imshow("Video frames with SURF features", videoFrameWithKeypoints);
            }
            cv::imshow("Video frames", videoFrame);
        }
    }
    catch (const std::exception& e)
    {
    }
}

void ImageManipulator::captureVideo()
{
    captureVideoThread = new std::thread(&ImageManipulator::captureVideoWorker, this);
}

void ImageManipulator::stopVideo()
{
    videoCapture.release();
    videoCaptured = false;
    cv::destroyWindow("Video frames");
    cv::destroyWindow("Video frames with SURF features");
}

void ImageManipulator::changeBrightnessAndContrast()
{
    // http://docs.opencv.org/2.4/doc/tutorials/core/basic_linear_transform/basic_linear_transform.html
    cv::Mat resultImage = srcImage.clone();

    for( int y = 0; y < resultImage.rows; y++ )
    {
        for( int x = 0; x < resultImage.cols; x++ )
        {
            for( int c = 0; c < 3; c++ )
            {
                resultImage.at<cv::Vec3b>(y,x)[c] = cv::saturate_cast<uchar>(procImageContrast * (srcImage.at<cv::Vec3b>(y,x)[c]) + procImageBrightness);
            }
        }
    }

    if (grayscale)
    {
        cv::Mat resultImageGS(resultImage.size(), CV_8UC1);
        cv::cvtColor(resultImage, resultImageGS, CV_RGB2GRAY);
        resultImage = resultImageGS.clone();
    }

    procImage = resultImage.clone();
}

void ImageManipulator::applyNormalizedBoxBlur()
{
    if (normalizedBoxFilter > 0)
    {
        cv::Mat resultImage = procImage.clone();
        cv::blur(resultImage, procImage, cv::Size(normalizedBoxFilter, normalizedBoxFilter));
    }
}

void ImageManipulator::applyHomogeneousBlur()
{
    // http://docs.opencv.org/2.4/doc/tutorials/imgproc/gausian_median_blur_bilateral_filter/gausian_median_blur_bilateral_filter.html
    cv::Mat resultImage = procImage.clone();
    for ( int i = 1; i < homogeneousBlur; i = i + 2 )
    {
        cv::blur(resultImage, procImage, cv::Size(i, i), cv::Point(-1,-1));
    }
}

void ImageManipulator::applyGaussianBlur()
{
    // http://docs.opencv.org/2.4/doc/tutorials/imgproc/gausian_median_blur_bilateral_filter/gausian_median_blur_bilateral_filter.html
    cv::Mat resultImage = procImage.clone();
    for ( int i = 1; i < gaussianBlur; i = i + 2 )
    {
        cv::GaussianBlur(resultImage, procImage, cv::Size(i, i), 0, 0);
    }
}

void ImageManipulator::applyMedianBlur()
{
    // http://docs.opencv.org/2.4/doc/tutorials/imgproc/gausian_median_blur_bilateral_filter/gausian_median_blur_bilateral_filter.html
    cv::Mat resultImage = procImage.clone();
    for ( int i = 1; i < medianBlur; i = i + 2 )
    {
        cv::medianBlur(resultImage, procImage, i);
    }
}

void ImageManipulator::applyBilateralBlur()
{
    // http://docs.opencv.org/2.4/doc/tutorials/imgproc/gausian_median_blur_bilateral_filter/gausian_median_blur_bilateral_filter.html
    cv::Mat resultImage = procImage.clone();
    for ( int i = 1; i < bilateralBlur; i = i + 2 )
    {
        cv::bilateralFilter(resultImage, procImage, i, i*2, i/2);
    }
}

void ImageManipulator::findContours()
{
    preProcess();
    cv::Mat cannyResult;
    cv::Mat procImageGray;
    cv::RNG rng(12345);
    if (!grayscale)
    {
        cv::cvtColor(procImage, procImageGray, cv::COLOR_BGR2GRAY);
    }
    else
    {
        procImageGray = procImage.clone();
    }
    cv::Canny(procImageGray, cannyResult, cannyThreshold, cannyThreshold*2, 3);
    cv::imshow("Canny result", cannyResult);

    cv::addWeighted(cannyResult, 0.5, this->cannyResult, 0.5, 0, procAggregateImage);
    cv::imshow("Canny aggregated result", procAggregateImage);

    this->cannyResult = cannyResult.clone();

    cv::findContours(cannyResult, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    cv::Mat contoursImage = cv::Mat::zeros(cannyResult.size(), CV_8UC3);
    processedContours.clear();
    for( size_t i = 0; i < contours.size(); i++ )
    {
        qDebug("%s", format_string("contour [%d] length = %d%s", i, contours[i].size(), (contours[i].size() > contourLengthThreshold ? "    --included" : "")).c_str());
        if (contours[i].size() > contourLengthThreshold)
        {
            cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            cv::drawContours(contoursImage, contours, (int)i, color, 2, 8, hierarchy, 0, cv::Point());
            processedContours.push_back(contours[i]);
        }
    }
    cv::imshow("Contours", contoursImage);
}

int ImageManipulator::getProcImageBrightness() const
{
    return procImageBrightness;
}

void ImageManipulator::setProcImageBrightness(int value)
{
    procImageBrightness = value;
}

double ImageManipulator::getProcImageContrast() const
{
    return procImageContrast;
}

void ImageManipulator::setProcImageContrast(double value)
{
    procImageContrast = value;
}

cv::Mat ImageManipulator::getProcImage() const
{
    return procImage;
}

void ImageManipulator::setProcImage(const cv::Mat &value)
{
    procImage = value;
    procAggregateImage = cv::Mat::zeros(procImage.size(), CV_8UC1);
    cannyResult = cv::Mat::zeros(procImage.size(), CV_8UC1);
}

cv::Mat ImageManipulator::getSrcImage() const
{
    return srcImage;
}

void ImageManipulator::setSrcImage(const cv::Mat &value)
{
    srcImage = value;
}

int ImageManipulator::getNormalizedBoxFilter() const
{
    return normalizedBoxFilter;
}

void ImageManipulator::setNormalizedBoxFilter(int value)
{
    normalizedBoxFilter = value;
}

int ImageManipulator::getBilateralBlur() const
{
    return bilateralBlur;
}

void ImageManipulator::setBilateralBlur(int value)
{
    bilateralBlur = value;
}

int ImageManipulator::getMedianBlur() const
{
    return medianBlur;
}

void ImageManipulator::setMedianBlur(int value)
{
    medianBlur = value;
}

int ImageManipulator::getGaussianBlur() const
{
    return gaussianBlur;
}

void ImageManipulator::setGaussianBlur(int value)
{
    gaussianBlur = value;
}

int ImageManipulator::getHomogeneousBlur() const
{
    return homogeneousBlur;
}

void ImageManipulator::setHomogeneousBlur(int value)
{
    homogeneousBlur = value;
}

bool ImageManipulator::getGrayscale() const
{
    return grayscale;
}

void ImageManipulator::setGrayscale(bool value)
{
    grayscale = value;
}

int ImageManipulator::getCannyThreshold() const
{
    return cannyThreshold;
}

void ImageManipulator::setCannyThreshold(int value)
{
    cannyThreshold = value;
}

unsigned ImageManipulator::getContourLengthThreshold() const
{
    return contourLengthThreshold;
}

void ImageManipulator::setContourLengthThreshold(int value)
{
    contourLengthThreshold = (unsigned int) value;
}

cv::Mat ImageManipulator::getObjectImage() const
{
    return objectImage;
}

void ImageManipulator::setObjectImage(const cv::Mat &value)
{
    /*
    cv::Mat objectImageGray;
    cv::cvtColor(value, objectImageGray, cv::COLOR_BGR2GRAY);
    objectImage = objectImageGray.clone();
    */
    objectImage = value;
    extractFeaturesFromObjectImage();
}


cv::Mat ImageManipulator::getObjectImageWithKeypoints() const
{
    return objectImageWithKeypoints;
}

void ImageManipulator::setObjectImageWithKeypoints(const cv::Mat &value)
{
    objectImageWithKeypoints = value;
}

void ImageManipulator::extractFeaturesFromObjectImage()
{
    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(getMinHessian());
    detector->detect(objectImage, keypoints);
    cv::drawKeypoints(objectImage, keypoints, objectImageWithKeypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
}

int ImageManipulator::getMinHessian() const
{
    return minHessian;
}

void ImageManipulator::setMinHessian(int value)
{
    minHessian = value;
    extractFeaturesFromObjectImage();
}
