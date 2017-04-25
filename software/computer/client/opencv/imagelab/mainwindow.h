#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QMessageBox>
#include "cvimagewidget.h"
#include "imagemanipulator.h"
#include "cascadeclassifier.h"
#include "neuralnetwork.h"
#include "utils.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    ImageManipulator *imageManipulator;
    CascadeClassifier *cascadeClassifier;
    NeuralNetwork *neuralNetwork;

    std::string fileName;
    void loadImage();

    std::string objectFileName;
    void loadObjectImage();

    CVImageWidget* srcImageWidget;
    CVImageWidget* procImageWidget;
    CVImageWidget* objectImageWidget;
    CVImageWidget* objectImageWithKeypointsWidget;

private slots:
    void on_brightnessHorizontalSlider_valueChanged(int value);

    void on_contrastHorizontalSlider_valueChanged(int value);

    void on_homogeneousBlurHorizontalSlider_valueChanged(int value);

    void on_gaussianBlurHorizontalSlider_valueChanged(int value);

    void on_medianBlurHorizontalSlider_valueChanged(int value);

    void on_bilateralBlurHorizontalSlider_valueChanged(int value);

    void on_processButton_clicked();

    void on_grayscaleCheckBox_clicked(bool checked);

    void on_findContoursButton_clicked();

    void on_cannyThresholdSlider_valueChanged(int value);

    void on_openFileButton_clicked();

    void on_normalizedBoxFilterSlider_valueChanged(int value);

    void on_contourLengthSlider_valueChanged(int value);

    void on_captureVideoButton_clicked();

    void on_openFileWithObjectButton_clicked();

    void on_minHessianSlider_valueChanged(int value);

    void on_colorReduceSlider_valueChanged(int value);

    void on_colorDenoisingSlider_valueChanged(int value);

    void on_positiveImagesFolderButton_clicked();

    void on_negativeImagesFolderButton_clicked();

    void on_captureAndSavePositiveImagesButton_clicked(bool checked);

    void on_captureAndSaveNegativeImagesButton_clicked(bool checked);

    void on_captureFramesButton_clicked(bool checked);

    void on_contourImagePathLineEditOKButton_clicked();

    void on_positiveImageFolderLineEditOKButton_clicked();

    void on_negativeImageFolderLineEditOKButton_clicked();

    void on_createCollectionFileFromPositiveImagesButton_clicked();

    void on_delayPositiveImageProcessingSlider_valueChanged(int value);

    void on_positiveImageProcessingThresholdSlider_valueChanged(int value);

    void on_cascadeClassifierFileOpenOKButton_clicked();

    void on_captureFrameObjectDetectionButton_clicked(bool checked);

    void on_openCascadeOfClassifiersButton_clicked();

    void on_imageThresholdSlider_valueChanged(int value);

    void on_positiveImageProcessingThresholdPreviewCheckBox_clicked(bool checked);

    void on_positiveImageProcessingGaussianBlurPreviewCheckBox_clicked(bool checked);

    void on_positiveImageProcessingBrightnessPreviewCheckBox_clicked(bool checked);

    void on_positiveImageProcessingGaussianBlurSlider_valueChanged(int value);

    void on_positiveImageProcessingBrightnessSlider_valueChanged(int value);

    void on_positiveImageProcessingContrastSlider_valueChanged(int value);

    void on_positiveImageProcessingCannyThresholdSlider_valueChanged(int value);

    void on_processingPositiveImagesMinControuAreaOkButton_clicked();

    void on_cropPositivesInRealTimeCheckBox_clicked(bool checked);

    void on_setCroppedWidthAndHeightButton_clicked();

    void on_showCroppedPositivesCheckBox_clicked(bool checked);

    void on_createTrainingSamplesPreviewCheckBox_clicked(bool checked);

    void on_createTrainingSamplesButton_clicked();

    void on_samplesFolderButton_clicked();

    void on_samplesFolderOKButton_clicked();

    void on_nnOpenReferenceObjectButton_clicked();

    void on_nnOpenReferenceObjectOKButton_clicked();

    void on_nnOpenReferenceObjectsFolderButton_clicked();

    void on_nnOpenReferenceObjectsFolderOKButton_clicked();

    void on_nnOpenTrainingObjectsFolderButton_clicked();

    void on_nnOpenTrainingObjectsFolderOKButton_clicked();

    void on_createTrainingObjectsButton_clicked();

    void on_createTrainingObjectsShowDelaySlider_valueChanged(int value);

    void on_nnCreateTrainingSamplesPreviewCheckBox_clicked(bool checked);

    void on_nnSetNeuralNetworkButton_clicked();

    void on_nnStartTrainingButton_clicked(bool checked);

    void on_nnClearLogButton_clicked();

    void on_sharpenCheckBox_clicked(bool checked);

    void on_sharpenSlider_valueChanged(int value);

    void on_ccSharpenCheckBox_clicked(bool checked);

    void on_ccSharpenSlider_valueChanged(int value);


    void on_ccBackgroundBlackRadioButton_clicked(bool checked);

    void on_ccBackgroundWhiteRadioButton_clicked(bool checked);

    void on_ccApplyFiltersCheckBox_clicked(bool checked);

    void on_ccApplyFiltersRandomlyCheckBox_clicked(bool checked);

    void on_ccRotateSamplesSlider_valueChanged(int value);

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
