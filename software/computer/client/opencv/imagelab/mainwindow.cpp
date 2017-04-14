#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    imageManipulator = new ImageManipulator();
    cascadeClassifier = new CascadeClassifier();

    // Create the processed image widget
    procImageWidget = new CVImageWidget();
    ui->imageLayout->insertWidget(0, procImageWidget);

    QSizePolicy sizePolicy;
    sizePolicy.setHorizontalPolicy(QSizePolicy::Expanding);
    QFrame* imageSeparatorLine = new QFrame;
    imageSeparatorLine->setFrameShape(QFrame::HLine);
    imageSeparatorLine->setSizePolicy(sizePolicy);
    ui->imageLayout->insertWidget(1, imageSeparatorLine);

    // Create the src image widget
    srcImageWidget = new CVImageWidget();
    srcImageWidget->setHasROI(true);
    ui->imageLayout->insertWidget(2, srcImageWidget);

    ui->imageLayout->setAlignment(Qt::AlignTop);

    objectImageWidget = new CVImageWidget();
    ui->findObjectverticalLayout->insertWidget(1, objectImageWidget);

    objectImageWithKeypointsWidget = new CVImageWidget();
    ui->findObjectverticalLayout->insertWidget(2, objectImageWithKeypointsWidget);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::loadImage()
{
    // Load src image
    cv::Mat srcImage = cv::imread(fileName, true);
    imageManipulator->setSrcImage(srcImage);
    srcImageWidget->showImage(imageManipulator->getSrcImage());

    // Clone src image to processed image
    cv::Mat procImage = srcImage.clone();
    imageManipulator->setProcImage(procImage);
    procImageWidget->showImage(imageManipulator->getProcImage());
}

void MainWindow::loadObjectImage()
{
    // Load object image
    cv::Mat objectImage = cv::imread(objectFileName, true);
    imageManipulator->setObjectImage(objectImage);
    objectImageWidget->showImage(imageManipulator->getObjectImage());

    objectImageWithKeypointsWidget->showImage(imageManipulator->getObjectImageWithKeypoints());
}

void MainWindow::on_openFileButton_clicked()
{
    fileName = QFileDialog::getOpenFileName(this, tr("Open Image"), "/home/maska", tr("Image Files (*.png *.jpg *.jpeg *.bmp)")).toUtf8().constData();
    ui->contourImagePathLineEdit->setText(fileName.c_str());
    loadImage();
}


void MainWindow::on_contourImagePathLineEditOKButton_clicked()
{
    fileName = ui->contourImagePathLineEdit->text().toStdString();
    loadImage();
}

void MainWindow::on_brightnessHorizontalSlider_valueChanged(int value)
{
    ui->brightnessLabel->setText(format_string("Brightness [%d]", value).c_str());
    imageManipulator->setProcImageBrightness(value);
}

void MainWindow::on_contrastHorizontalSlider_valueChanged(int value)
{
    double contrast = (double)value / 10;
    ui->contrastLabel->setText(format_string("Contrast [%.1f]", contrast).c_str());
    imageManipulator->setProcImageContrast(contrast);
}

void MainWindow::on_grayscaleCheckBox_clicked(bool checked)
{
    imageManipulator->setGrayscale(checked);
    if (checked)
    {
        ui->colorDenoisingSlider->setValue(0);
        ui->colorDenoisingSlider->setEnabled(false);
    }
    else
    {
        ui->colorDenoisingSlider->setEnabled(true);
    }
}

void MainWindow::on_normalizedBoxFilterSlider_valueChanged(int value)
{
    ui->normalizedBoxFilterLabel->setText(format_string("Normalized Box Filter (kernel length = %d)", value).c_str());
    imageManipulator->setNormalizedBoxFilter(value);
}

void MainWindow::on_homogeneousBlurHorizontalSlider_valueChanged(int value)
{
    ui->homogeneousBlurLabel->setText(format_string("Homogeneous Blur (kernel length = %d)", value).c_str());
    imageManipulator->setHomogeneousBlur(value);
}

void MainWindow::on_gaussianBlurHorizontalSlider_valueChanged(int value)
{
    ui->gaussianBlurLabel->setText(format_string("Gaussian Blur (kernel length = %d)", value).c_str());
    imageManipulator->setGaussianBlur(value);
}

void MainWindow::on_medianBlurHorizontalSlider_valueChanged(int value)
{
    ui->medianBlurLabel->setText(format_string("Median Blur (kernel length = %d)", value).c_str());
    imageManipulator->setMedianBlur(value);
}

void MainWindow::on_bilateralBlurHorizontalSlider_valueChanged(int value)
{
    ui->bilateralBlurLabel->setText(format_string("Bilateral Blur (kernel length = %d)", value).c_str());
    imageManipulator->setBilateralBlur(value);
}

void MainWindow::on_processButton_clicked()
{
    imageManipulator->setROI(srcImageWidget->getRoiRect());
    imageManipulator->preProcess();
    cv::imshow("Original", imageManipulator->getProcImage());
    procImageWidget->showImage(imageManipulator->getProcImage());
}

void MainWindow::on_cannyThresholdSlider_valueChanged(int value)
{
    ui->cannyThresholdLabel->setText(format_string("Canny Threshold [%d]", value).c_str());
    imageManipulator->setCannyThreshold(value);
}

void MainWindow::on_findContoursButton_clicked()
{
    imageManipulator->findContours();
}

void MainWindow::on_contourLengthSlider_valueChanged(int value)
{
    ui->contourLengthLabel->setText(format_string("Contour Length Threshold [%d]", value).c_str());
    imageManipulator->setContourLengthThreshold(value);
}

void MainWindow::on_captureVideoButton_clicked()
{
    if (ui->captureVideoButton->isChecked())
    {
        ui->openFileButton->setEnabled(false);
        imageManipulator->captureVideo();
    }
    else
    {
        ui->openFileButton->setEnabled(true);
        imageManipulator->stopVideo();
    }
}

void MainWindow::on_openFileWithObjectButton_clicked()
{
    objectFileName = QFileDialog::getOpenFileName(this, tr("Open Image"), "/home/maska", tr("Image Files (*.png *.jpg *.jpeg *.bmp)")).toUtf8().constData();
    loadObjectImage();
}

void MainWindow::on_minHessianSlider_valueChanged(int value)
{
    ui->minHessianLabel->setText(format_string("min Hessian [%d]", value).c_str());
    imageManipulator->setMinHessian(value);
    objectImageWithKeypointsWidget->showImage(imageManipulator->getObjectImageWithKeypoints());
}

void MainWindow::on_colorReduceSlider_valueChanged(int value)
{
    ui->colorReduceLabel->setText(format_string("Color Reduce [%d]", value).c_str());
    imageManipulator->setColorReduceFactor(value);
}

void MainWindow::on_colorDenoisingSlider_valueChanged(int value)
{
    imageManipulator->setColorDenoiserStrength(value);
    ui->colorDenoisingLabel->setText(format_string("Color Denoising [%d]", imageManipulator->getColorDenoiserStrength()).c_str());
}

//Haar Cascade Classifiers
void MainWindow::on_positiveImagesFolderButton_clicked()
{
    std::string positiveImagesFolder = QFileDialog::getExistingDirectory(this, tr("Open Positive Images Folder"), "/home/maska").toUtf8().constData();
    if (!positiveImagesFolder.empty())
    {
        cascadeClassifier->setPositiveImagesFolder(positiveImagesFolder);
        ui->positiveImageFolderLineEdit->setText(positiveImagesFolder.c_str());
        qDebug("Positive Image Folder: %s", positiveImagesFolder.c_str());
    }
}

void MainWindow::on_negativeImagesFolderButton_clicked()
{
    std::string negativeImagesFolder = QFileDialog::getExistingDirectory(this, tr("Open Negative Images Folder"), "/home/maska").toUtf8().constData();
    if (!negativeImagesFolder.empty())
    {
        cascadeClassifier->setNegativeImagesFolder(negativeImagesFolder);
        ui->negativeImageFolderLineEdit->setText(negativeImagesFolder.c_str());
        qDebug("Negative Image Folder: %s", negativeImagesFolder.c_str());
    }
}

void MainWindow::on_captureAndSavePositiveImagesButton_clicked(bool checked)
{
    cascadeClassifier->setCapturePositive(checked);
    if (checked)
    {
        ui->captureAndSaveNegativeImagesButton->setChecked(false);
        cascadeClassifier->setCaptureNegative(false);
    }
}

void MainWindow::on_captureAndSaveNegativeImagesButton_clicked(bool checked)
{
    cascadeClassifier->setCaptureNegative(checked);
    if (checked)
    {
        ui->captureAndSavePositiveImagesButton->setChecked(false);
        cascadeClassifier->setCapturePositive(false);
    }
}

void MainWindow::on_captureFramesButton_clicked(bool checked)
{
    if (checked)
    {
        cascadeClassifier->captureVideo();
    }
    else
    {
        cascadeClassifier->stopVideo();
    }
}

void MainWindow::on_positiveImageFolderLineEditOKButton_clicked()
{
    std::string folderName = ui->positiveImageFolderLineEdit->text().toStdString();
    cascadeClassifier->setPositiveImagesFolder(folderName);
    qDebug("Positive images Folder: %s", folderName.c_str());
}

void MainWindow::on_negativeImageFolderLineEditOKButton_clicked()
{
    std::string folderName = ui->negativeImageFolderLineEdit->text().toStdString();
    cascadeClassifier->setNegativeImagesFolder(folderName);
    qDebug("Negative images Folder: %s", folderName.c_str());
}


void MainWindow::on_createCollectionFileFromPositiveImagesButton_clicked()
{
    cascadeClassifier->processPositiveImagesToCollectionFile();
}

void MainWindow::on_delayPositiveImageProcessingSlider_valueChanged(int value)
{
    cascadeClassifier->setPositiveImageProcessingDelay(value);
}

void MainWindow::on_positiveImageProcessingThresholdSlider_valueChanged(int value)
{
    ui->positiveImageProcessingThresholdLabel->setText(format_string("Threshold [%d]", value).c_str());
    cascadeClassifier->setPositiveImageProcessingThreshold(value);
}
