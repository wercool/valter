#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    imageManipulator = new ImageManipulator();
    cascadeClassifier = new CascadeClassifier();
    neuralNetwork = new NeuralNetwork();

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


void MainWindow::on_imageThresholdSlider_valueChanged(int value)
{
    ui->imageThresholdLabel->setText(format_string("Thresholding [%d]", value).c_str());
    imageManipulator->setImageThreshold(value);
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
void MainWindow::on_sharpenCheckBox_clicked(bool checked)
{
    imageManipulator->setSharpen(checked);
}


void MainWindow::on_sharpenSlider_valueChanged(int value)
{
    imageManipulator->setSharpenFactor((double)value / 1000.0);
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
        cascadeClassifier->readPositiveImagesDir();
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
        cascadeClassifier->readNegativeImagesDir();
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
    else
    {
        cascadeClassifier->savePositiveCroppedInfo();
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
    cascadeClassifier->readPositiveImagesDir();
}

void MainWindow::on_negativeImageFolderLineEditOKButton_clicked()
{
    std::string folderName = ui->negativeImageFolderLineEdit->text().toStdString();
    cascadeClassifier->setNegativeImagesFolder(folderName);
    qDebug("Negative images Folder: %s", folderName.c_str());
    cascadeClassifier->readNegativeImagesDir();
}


void MainWindow::on_createCollectionFileFromPositiveImagesButton_clicked()
{
    cascadeClassifier->setCroppedWidth(std::atoi(ui->croppedSampleWidthEdit->text().toStdString().c_str()));
    cascadeClassifier->setCroppedHeight(std::atoi(ui->croppedSampleHeightEdit->text().toStdString().c_str()));
    qDebug("Cropped width, height = %d, %d", cascadeClassifier->getCroppedWidth(), cascadeClassifier->getCroppedHeight());
    cascadeClassifier->processPositiveImagesToCollectionFile(ui->cropPositivesCheckBox->isChecked());
}

void MainWindow::on_delayPositiveImageProcessingSlider_valueChanged(int value)
{
    cascadeClassifier->setPositiveImageProcessingDelay(value);
}

void MainWindow::on_positiveImageProcessingThresholdSlider_valueChanged(int value)
{
    ui->positiveImageProcessingThresholdLabel->setText(format_string("Threshold: [%d]", value).c_str());
    cascadeClassifier->setPositiveImageProcessingThreshold(value);
}

void MainWindow::on_cascadeClassifierFileOpenOKButton_clicked()
{
    std::string cascadeClassifierFileName = ui->cascadeOfClassifiersFileLineEdit->text().toStdString();
    cascadeClassifier->setCascadeClassifierFile(cascadeClassifierFileName);
    qDebug("Cascade Classifier file name: %s", cascadeClassifierFileName.c_str());
}

void MainWindow::on_captureFrameObjectDetectionButton_clicked(bool checked)
{
    if (checked)
    {
        cascadeClassifier->startDetection();
    }
    else
    {
        cascadeClassifier->stopDetection();
    }
}

void MainWindow::on_openCascadeOfClassifiersButton_clicked()
{
    std::string cascadeClassifierFileName = QFileDialog::getOpenFileName(this, tr("Cascade of Classifier file"), "/home/maska", tr("Cascade Files (*.xml)")).toUtf8().constData();
    qDebug("Cascade Classifier file name: %s", cascadeClassifierFileName.c_str());
    cascadeClassifier->setCascadeClassifierFile(cascadeClassifierFileName);
    ui->cascadeOfClassifiersFileLineEdit->setText(cascadeClassifierFileName.c_str());
}


void MainWindow::on_positiveImageProcessingThresholdPreviewCheckBox_clicked(bool checked)
{
    cascadeClassifier->setThresholdPreview(checked);
}

void MainWindow::on_positiveImageProcessingGaussianBlurPreviewCheckBox_clicked(bool checked)
{
    cascadeClassifier->setGaussianBlurPreview(checked);
}

void MainWindow::on_positiveImageProcessingBrightnessPreviewCheckBox_clicked(bool checked)
{
    cascadeClassifier->setBrightnessPreview(checked);
}

void MainWindow::on_positiveImageProcessingGaussianBlurSlider_valueChanged(int value)
{
    ui->positiveImageProcessingGaussianBlurLabel->setText(format_string("Gaussian Blur: [%d]", value).c_str());
    cascadeClassifier->setPositiveImageProcessingGaussianBlur(value);
}

void MainWindow::on_positiveImageProcessingBrightnessSlider_valueChanged(int value)
{
    ui->positiveImageProcessingBrightnessLabel->setText(format_string("Brightness: [%d]", value).c_str());
    cascadeClassifier->setPositiveImageProcessingBrightness(value);
}

void MainWindow::on_positiveImageProcessingContrastSlider_valueChanged(int value)
{
    double contrast = (double)value / 10;
    ui->positiveImageProcessingContrastLabel->setText(format_string("Contrast: [%.2f]", contrast).c_str());
    cascadeClassifier->setPositiveImageProcessingContrast(contrast);
}

void MainWindow::on_positiveImageProcessingCannyThresholdSlider_valueChanged(int value)
{
    ui->positiveImageProcessingCannyThresholdLabel->setText(format_string("Canny Threshold: [%d]", value).c_str());
    cascadeClassifier->setPositiveImageProcessingCannyThreshold(value);
}

void MainWindow::on_processingPositiveImagesMinControuAreaOkButton_clicked()
{
    cascadeClassifier->setMinContourArea(std::atoi(ui->processingPositiveImagesMinControuAreaLineEdit->text().toStdString().c_str()));
}

void MainWindow::on_cropPositivesInRealTimeCheckBox_clicked(bool checked)
{
    cascadeClassifier->setCropPositiveImagesRealTime(checked);
}

void MainWindow::on_setCroppedWidthAndHeightButton_clicked()
{
    cascadeClassifier->setCroppedWidth(std::atoi(ui->croppedSampleWidthEdit->text().toStdString().c_str()));
    cascadeClassifier->setCroppedHeight(std::atoi(ui->croppedSampleHeightEdit->text().toStdString().c_str()));
    qDebug("Cropped width, height = %d, %d", cascadeClassifier->getCroppedWidth(), cascadeClassifier->getCroppedHeight());
    int minContourArea = cascadeClassifier->getCroppedWidth() * cascadeClassifier->getCroppedHeight();
    ui->processingPositiveImagesMinControuAreaLineEdit->setText(format_string("%d", minContourArea).c_str());
    cascadeClassifier->setMinContourArea(minContourArea);
}

void MainWindow::on_showCroppedPositivesCheckBox_clicked(bool checked)
{
    cascadeClassifier->setShowCroppedPositives(checked);
}

void MainWindow::on_createTrainingSamplesPreviewCheckBox_clicked(bool checked)
{
    cascadeClassifier->setCreateTrainingSamplesPreview(checked);
}

void MainWindow::on_createTrainingSamplesButton_clicked()
{
    if (cascadeClassifier->getNegativeFileNames().size() == 0)
    {
        QMessageBox *msgBox = new QMessageBox(0);
        msgBox->setText("Provide Negative images folder path");
        msgBox->exec();
        return;
    }

    if (cascadeClassifier->getPositiveFileNames().size() == 0)
    {
        QMessageBox *msgBox = new QMessageBox(0);
        msgBox->setText("Provide Positive images folder path");
        msgBox->exec();
        return;
    }

    cascadeClassifier->processTrainingSamples();
}

void MainWindow::on_samplesFolderButton_clicked()
{
    std::string samplesFolder = QFileDialog::getExistingDirectory(this, tr("Open Samples Folder"), "/home/maska").toUtf8().constData();
    if (!samplesFolder.empty())
    {
        cascadeClassifier->setSamplesFolder(samplesFolder);
        ui->samplesFolderEdit->setText(samplesFolder.c_str());
        qDebug("SamplesFolder: %s", samplesFolder.c_str());
    }
}

void MainWindow::on_samplesFolderOKButton_clicked()
{
    std::string samplesFolder = ui->samplesFolderEdit->text().toStdString();
    if (!samplesFolder.empty())
    {
        cascadeClassifier->setSamplesFolder(samplesFolder);
        qDebug("SamplesFolder: %s", samplesFolder.c_str());
    }
}


void MainWindow::on_ccSharpenCheckBox_clicked(bool checked)
{
    cascadeClassifier->setSharpen(checked);
}

void MainWindow::on_ccSharpenSlider_valueChanged(int value)
{
    cascadeClassifier->setSharpenFactor((double) value / 1000.0);
}

void MainWindow::on_ccBackgroundBlackRadioButton_clicked(bool checked)
{
    cascadeClassifier->setPositiveSampleBackgroundBlack(checked);
}

void MainWindow::on_ccBackgroundWhiteRadioButton_clicked(bool checked)
{
    cascadeClassifier->setPositiveSampleBackgroundBlack(!checked);
}

void MainWindow::on_ccApplyFiltersCheckBox_clicked(bool checked)
{
    cascadeClassifier->setApplyFiltersToPositiveSamples(checked);
}

void MainWindow::on_ccApplyFiltersRandomlyCheckBox_clicked(bool checked)
{
    cascadeClassifier->setApplyFiltersRandomlyToPositiveSamples(checked);
}

void MainWindow::on_ccRotateSamplesSlider_valueChanged(int value)
{
    ui->ccRotateSamplesLabel->setText(format_string("Rotate Samples: [%d]", value).c_str());
    cascadeClassifier->setRotateSamplesAngle(value);
}

void MainWindow::on_openFolderWithCascadeOfClassifiersOKButton_clicked()
{
    std::string cascadeClassifierFolder = ui->ccCascadeOfClassifiersFolderLineEdit->text().toStdString();
    if (!cascadeClassifierFolder.empty())
    {
        ui->ccCascadeOfClassifiersFolderLineEdit->setText(cascadeClassifierFolder.c_str());
        qDebug("Cascade of Classifiers Folder: %s", cascadeClassifierFolder.c_str());
        cascadeClassifier->readCascadeFolder(cascadeClassifierFolder);
        refreshCascadeClassifiersTreeView();
    }
}

void MainWindow::on_openFolderWithCascadeOfClassifiersButton_clicked()
{
    std::string cascadeClassifierFolder = QFileDialog::getExistingDirectory(this, tr("Open Cascade of Classifiers Folder"), "/home/maska").toUtf8().constData();
    if (!cascadeClassifierFolder.empty())
    {
        ui->ccCascadeOfClassifiersFolderLineEdit->setText(cascadeClassifierFolder.c_str());
        qDebug("Cascade of Classifiers Folder: %s", cascadeClassifierFolder.c_str());
        cascadeClassifier->readCascadeFolder(cascadeClassifierFolder);
        refreshCascadeClassifiersTreeView();
    }
}

void MainWindow::refreshCascadeClassifiersTreeView()
{
    ui->ccCascadesTreeWidget->clear();
    QTreeWidgetItem *objects = new QTreeWidgetItem(ui->ccCascadesTreeWidget);
    objects->setText(0, "Detection Obejcts");
    std::map<std::string, std::map<std::string, cv::CascadeClassifier>> objectCascades = cascadeClassifier->getObjectCascades();
    typedef std::map<std::string, std::map<std::string, cv::CascadeClassifier>>::iterator objectCascades_it_type;
    typedef std::map<std::string, cv::CascadeClassifier>::iterator cascadesClassifiers_it_type;
    for(objectCascades_it_type objIterator = objectCascades.begin(); objIterator != objectCascades.end(); objIterator++)
    {
        QTreeWidgetItem *objectItem = new QTreeWidgetItem(objects);
        objectItem->setText(0, ((std::string)objIterator->first).c_str());

        std::map<std::string, cv::CascadeClassifier> cascadeClassifiers = objIterator->second;
        for(cascadesClassifiers_it_type cascadeIterator = cascadeClassifiers.begin(); cascadeIterator != cascadeClassifiers.end(); cascadeIterator++)
        {
            QTreeWidgetItem *objectSubItem = new QTreeWidgetItem(objectItem);
            objectSubItem->setText(0, ((std::string)cascadeIterator->first).c_str());
            objectItem->addChild(objectSubItem);
        }
    }
    ui->ccCascadesTreeWidget->expandAll();
}

void MainWindow::on_ccClearDetectionObjectsButton_clicked()
{
    cascadeClassifier->clearDetectionObjectsMap();
    std::string cascadeClassifierFolder = ui->ccCascadeOfClassifiersFolderLineEdit->text().toStdString();
    if (!cascadeClassifierFolder.empty())
    {
        ui->ccCascadeOfClassifiersFolderLineEdit->setText(cascadeClassifierFolder.c_str());
        qDebug("Cascade of Classifiers Folder: %s", cascadeClassifierFolder.c_str());
        cascadeClassifier->readCascadeFolder(cascadeClassifierFolder);
        refreshCascadeClassifiersTreeView();
    }
}

void MainWindow::on_ccDetectionModePluralRadioButton_clicked(bool checked)
{
    if (checked) cascadeClassifier->setDetectionModePlural(true);
}

void MainWindow::on_ccDetectionModeSingleRadioButton_clicked(bool checked)
{
    if (checked) cascadeClassifier->setDetectionModePlural(false);
}


//Neural network

void MainWindow::on_nnOpenReferenceObjectButton_clicked()
{
    static string lastOpenedDirectory;
    std::string referenceObjectFileName = QFileDialog::getOpenFileName(this, tr("Reference Object for NN"), (lastOpenedDirectory.empty()) ? "/home/maska" : lastOpenedDirectory.c_str(), tr("JPG Image (*.jpg *.jpeg, *.png)")).toUtf8().constData();
    qDebug("Reference Object FileName: %s", referenceObjectFileName.c_str());
    neuralNetwork->setReferenceObjectFileName(referenceObjectFileName);
    ui->nnReferenceObjectLineEdit->setText(referenceObjectFileName.c_str());
    neuralNetwork->readReferenceObjectMat();
    lastOpenedDirectory = referenceObjectFileName;
}

void MainWindow::on_nnOpenReferenceObjectOKButton_clicked()
{
    std::string referenceObjectFileName = ui->nnReferenceObjectLineEdit->text().toStdString();
    neuralNetwork->setReferenceObjectFileName(referenceObjectFileName);
    neuralNetwork->readReferenceObjectMat();
}

void MainWindow::on_nnOpenReferenceObjectsFolderButton_clicked()
{
    static string lastOpenedDirectory;
    std::string referenceObjectsFolder = QFileDialog::getExistingDirectory(this, tr("Reference Objects Folder"), (lastOpenedDirectory.empty()) ? "/home/maska" : lastOpenedDirectory.c_str()).toUtf8().constData();
    qDebug("Reference Object Folder: %s", referenceObjectsFolder.c_str());
    neuralNetwork->setReferenceObjectsFolderName(referenceObjectsFolder);
    ui->nnReferenceObjectsFolderLineEdit->setText(referenceObjectsFolder.c_str());
    lastOpenedDirectory = referenceObjectsFolder;
}

void MainWindow::on_nnOpenReferenceObjectsFolderOKButton_clicked()
{
    std::string referenceObjectsFolder = ui->nnReferenceObjectsFolderLineEdit->text().toStdString();
    neuralNetwork->setReferenceObjectsFolderName(referenceObjectsFolder);
}

void MainWindow::on_nnOpenTrainingObjectsFolderButton_clicked()
{
    static string lastOpenedDirectory;
    std::string trainingObjectsFolder = QFileDialog::getExistingDirectory(this, tr("Training Objects Folder"), (lastOpenedDirectory.empty()) ? "/home/maska" : lastOpenedDirectory.c_str()).toUtf8().constData();
    qDebug("Training Object Folder: %s", trainingObjectsFolder.c_str());
    neuralNetwork->setTrainingObjectsFolderName(trainingObjectsFolder);
    ui->nnTrainingObjectsFolderLineEdit->setText(trainingObjectsFolder.c_str());
    lastOpenedDirectory = trainingObjectsFolder;
}

void MainWindow::on_nnOpenTrainingObjectsFolderOKButton_clicked()
{
    std::string trainingObjectsFolder = ui->nnTrainingObjectsFolderLineEdit->text().toStdString();
    neuralNetwork->setTrainingObjectsFolderName(trainingObjectsFolder);
}

void MainWindow::on_createTrainingObjectsButton_clicked()
{
    if (neuralNetwork->getReferenceObjectsFolderName().empty())
    {
        QMessageBox *msgBox = new QMessageBox(0);
        msgBox->setText("Provide Reference Objects folder path");
        msgBox->exec();
        return;
    }
    if(neuralNetwork->getTrainingObjectsFolderName().empty())
    {
        QMessageBox *msgBox = new QMessageBox(0);
        msgBox->setText("Provide Training Objects folder path");
        msgBox->exec();
        return;
    }
    neuralNetwork->setTrainingSamplesNumber(ui->trainingSamplesNumSpinBox->value());
    neuralNetwork->createTrainingObjectsFromReferences();
}

void MainWindow::on_createTrainingObjectsShowDelaySlider_valueChanged(int value)
{
    neuralNetwork->setCreateTrainingObjectsShowDelay(value);
}

void MainWindow::on_nnCreateTrainingSamplesPreviewCheckBox_clicked(bool checked)
{
    neuralNetwork->setCreateTrainingSamplesPreview(checked);
}

void MainWindow::on_nnSetNeuralNetworkButton_clicked()
{
    if(neuralNetwork->getTrainingObjectsFolderName().empty())
    {
        QMessageBox *msgBox = new QMessageBox(0);
        msgBox->setText("Provide Training Objects folder path");
        msgBox->exec();
        return;
    }

    std::vector<int> layers;
    string layersStr = ui->nnLayersLineEdit->text().toStdString();

    size_t pos = 0;
    while ((pos = layersStr.find(",")) != std::string::npos)
    {
        layers.push_back(atoi(layersStr.substr(0, pos).c_str()));
        layersStr.erase(0, pos + 1);
    }
    if (layersStr.size() > 0)
    {
        layers.push_back(atoi(layersStr.c_str()));
    }

    for (unsigned int i = 0; i < layers.size(); i++)
    {
        ui->nnLogTextEdit->appendPlainText(format_string("Layer %d size: %d neurons", i, layers[i]).c_str());
    }

    neuralNetwork->setLayers(layers);
    neuralNetwork->setMiniBatchSize(ui->nnMiniBatchSizeSpinBox->value());
    neuralNetwork->setEpochs(ui->nnEpochsSpinBox->value());
    neuralNetwork->setEta(ui->nnEtaSpinBox->value());

    if (ui->nnCostFunctionQuadraticCostRadioButton->isChecked())
    {
        neuralNetwork->setCostFunction("QuadraticCost");
    }
    if (ui->nnCostFunctionCrossEntropyCostRadioButton->isChecked())
    {
        neuralNetwork->setCostFunction("CrossEntropyCost");
    }

    neuralNetwork->readTrainingSamplesFileNames();

    if (neuralNetwork->getTrainingSamplesFileName().size() == 0)
    {
        QMessageBox *msgBox = new QMessageBox(0);
        msgBox->setText("Training Objects folder is empty");
        msgBox->exec();
        return;
    }

    neuralNetwork->initNetwork();

    ui->nnLogTextEdit->appendPlainText(format_string("Mini Batch size: %d", neuralNetwork->getMiniBatchSize()).c_str());
    ui->nnLogTextEdit->appendPlainText(format_string("Epochs: %d", neuralNetwork->getEpochs()).c_str());
    ui->nnLogTextEdit->appendPlainText(format_string("Learning Rate: %.5f", neuralNetwork->getEta()).c_str());
    ui->nnLogTextEdit->appendPlainText(format_string("Training samples number: %d", neuralNetwork->getTrainingSamplesFileName().size()).c_str());
}

void MainWindow::on_nnStartTrainingButton_clicked(bool checked)
{
    neuralNetwork->setTraining(checked);
}

void MainWindow::on_nnClearLogButton_clicked()
{
    ui->nnLogTextEdit->clear();
}

void MainWindow::on_nnInitializeFromFileButton_clicked()
{
    std::string nnInitializationFileName = QFileDialog::getOpenFileName(this, tr("Neural Network Initialization File"), "/home/maska", tr("NN Initi File (*.nn)")).toUtf8().constData();
    qDebug("Neural Network Initialization File: %s", nnInitializationFileName.c_str());
    neuralNetwork->loadNetworkFromFile(nnInitializationFileName);

    vector<int> layers = neuralNetwork->getLayers();
    std::ostringstream layerOSS;
    std::copy(layers.begin(), layers.end(), std::ostream_iterator<int>(layerOSS, ","));
    string layersStrigified = layerOSS.str().substr(0, layerOSS.str().size() - 1);

    ui->nnLayersLineEdit->setText(layersStrigified.c_str());
    ui->nnMiniBatchSizeSpinBox->setValue(neuralNetwork->getMiniBatchSize());
    ui->nnEpochsSpinBox->setValue(neuralNetwork->getEpochs());
    ui->nnEtaSpinBox->setValue(neuralNetwork->getEta());

    if (neuralNetwork->getCostFunction() == "QuadraticCost")
    {
        ui->nnCostFunctionQuadraticCostRadioButton->setChecked(true);
    }
    if (neuralNetwork->getCostFunction() == "CrossEntropyCost")
    {
        ui->nnCostFunctionCrossEntropyCostRadioButton->setChecked(true);
    }

    ui->nnLogTextEdit->clear();
    ui->nnLogTextEdit->appendPlainText(format_string("Mini Batch size: %d", neuralNetwork->getMiniBatchSize()).c_str());
    ui->nnLogTextEdit->appendPlainText(format_string("Epochs: %d", neuralNetwork->getEpochs()).c_str());
    ui->nnLogTextEdit->appendPlainText(format_string("Learning Rate: %.8f", neuralNetwork->getEta()).c_str());
}

void MainWindow::on_nnSaveToFileButton_clicked()
{
    std::string nnInitializationFileName = QFileDialog::getSaveFileName(this, tr("Neural Network Initialization File"), "/home/maska", tr("NN Initi File (*.nn)")).toUtf8().constData();
    qDebug("Neural Network Initialization File: %s", nnInitializationFileName.c_str());
    neuralNetwork->saveNetworkToFile(nnInitializationFileName);
}

void MainWindow::on_nnRecognizeReferenceObjectButton_clicked()
{
    if(neuralNetwork->getReferenceObjectFileName().empty())
    {
        QMessageBox *msgBox = new QMessageBox(0);
        msgBox->setText("Provide Reference Object file path");
        msgBox->exec();
        return;
    }

    vector<double> outputVector = neuralNetwork->recognizeReferenceObject();

    ui->nnLogTextEdit->appendPlainText("Neural Network Output Vector");
    for (unsigned int o = 0; o < outputVector.size(); o++)
    {
        ui->nnLogTextEdit->appendPlainText(format_string("%.15f", outputVector[o]).c_str());
    }
}

void MainWindow::on_nnRotateSamplesCheclBox_clicked(bool checked)
{
    neuralNetwork->setRotateSamples(checked);
}
