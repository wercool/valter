#include "templatematching.h"

TemplateMatching::TemplateMatching()
{

}

void TemplateMatching::readTargetImage(string targetImagePath)
{
    targetImage = cv::imread(targetImagePath);
    cv::imshow("Target image", targetImage);
}

void TemplateMatching::readTemplateMatching(string templateImagePath)
{
    templateImage = cv::imread(templateImagePath);
    cv::imshow("Template image", templateImage);
}

void TemplateMatching::templateMatch()
{
    /// Create the result matrix
    int result_cols =  targetImage.cols - templateImage.cols + 1;
    int result_rows = targetImage.rows - templateImage.rows + 1;

    templateMatchResult.create(result_rows, result_cols, CV_32FC1);

    /// Do the Matching and Normalize
//    cv::matchTemplate(targetImage, templateImage, templateMatchResult, cv::TM_SQDIFF);
//    cv::matchTemplate(targetImage, templateImage, templateMatchResult, cv::TM_CCORR_NORMED);
    cv::matchTemplate(targetImage, templateImage, templateMatchResult, cv::TM_CCOEFF_NORMED);
    cv::imshow("Template Match Result", templateMatchResult);
    cv::normalize(templateMatchResult, templateMatchResult, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
}
