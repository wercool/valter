#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->imageLayout->setAlignment(Qt::AlignTop);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::loadImage()
{
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
    ui->imageLayout->insertWidget(2, srcImageWidget);

    imageManipulator = new ImageManipulator();
    // Load src image
    cv::Mat srcImage = cv::imread(fileName, true);
    imageManipulator->setSrcImage(srcImage);
    srcImageWidget->showImage(imageManipulator->getSrcImage());

    // Clone src image to processed image
    cv::Mat procImage = srcImage.clone();
    imageManipulator->setProcImage(procImage);
    procImageWidget->showImage(imageManipulator->getProcImage());
}

void MainWindow::on_openFileButton_clicked()
{
    fileName = QFileDialog::getOpenFileName(this, tr("Open Image"), "/home/maska", tr("Image Files (*.png *.jpg *.jpeg *.bmp)")).toUtf8().constData();
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
    imageManipulator->preProcess();
    procImageWidget->showImage(imageManipulator->getProcImage());
}

void MainWindow::on_cannyThresholdSlider_valueChanged(int value)
{
    imageManipulator->setCannyThreshold(value);
}

void MainWindow::on_findContoursButton_clicked()
{
    imageManipulator->findContours();
}
