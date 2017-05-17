#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    setMinimumWidth(1024);
    setMinimumHeight(768);

    ui->setupUi(this);

    ui->graphicsView->setRenderHint(QPainter::Antialiasing, false);
    ui->graphicsView->setDragMode(QGraphicsView::NoDrag);
    ui->graphicsView->setOptimizationFlags(QGraphicsView::DontAdjustForAntialiasing | QGraphicsView::DontSavePainterState);
    ui->graphicsView->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    ui->graphicsView->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    ui->graphicsView->setCacheMode(QGraphicsView::CacheBackground);

    ui->graphicsView->setViewport(new QWidget());

//    if (QGLFormat::hasOpenGL())
//    {
//        ui->graphicsView->setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
//    }
//    else
//    {
//        ui->graphicsView->setViewport(new QWidget());
//    }

    graphicsViewScene = new QGraphicsScene;
    ui->graphicsView->setScene(graphicsViewScene);
    ui->graphicsView->scene()->clear();

    envMapPixmapItem = new QGraphicsPixmapItem();
    ui->graphicsView->scene()->addItem(envMapPixmapItem);

    lifeTimer = new QTimer(this);
    connect(lifeTimer, SIGNAL(timeout()), this, SLOT(lifeTimerCallback()));

    colony = Colony(graphicsViewScene);
    colony.setEnvMapMutex(&envMapMutex);

    cv::namedWindow("Env Map", cv::WINDOW_KEEPRATIO);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_loadEnvMapButton_clicked()
{
    std::string imageFilePath = QFileDialog::getOpenFileName(this, tr("Open Image"), "/home/maska/Documents/Valter/photo", tr("Image Files (*.png *.jpg *.jpeg *.bmp)")).toUtf8().constData();

    if (!imageFilePath.empty())
    {
        cv::Mat srcImage = cv::imread(imageFilePath, true);
        cv::cvtColor(srcImage, envMapOriginalMat, CV_BGRA2GRAY);

        updateEnvPixmap();
    }
}

void MainWindow::lifeTimerCallback()
{
    static int fpsc = 0;
    static int fpscEnvMap = 0;

    if (fpsc > 2)
    {
        vector<Creature *> pCreatures = colony.getCreatures();
        Creature *pCreature;
        QGraphicsItem *pCreatureGI;
        for (unsigned int i = 0; i < pCreatures.size(); i++)
        {
            pCreature = pCreatures[i];
            if (pCreature != nullptr)
            {
                pCreatureGI = dynamic_cast<QGraphicsItem *>(pCreature);

                if (pCreature->getVitality() > 0.0)
                {
                    pCreatureGI->setPos(QPointF(pCreature->getX(), pCreature->getY()));
                    pCreatureGI->setRotation(pCreature->getA());
                    pCreatureGI->setOpacity(pCreature->getVitality());
                }
                else
                {
                    pCreatureGI->setOpacity(1.0);
                }
            }
        }
        fpsc = 0;

        if (fpscEnvMap > 10)
        {
            if (ui->updateViewCheckBox->isChecked())
            {
                envMapPixmapItem->setPixmap(cvMatToPixMap(envMapMat));
            }
            if (ui->cvShowCheckBox->isChecked())
            {
                cv::resizeWindow("Env Map", 1024, 768);
                cv::imshow("Env Map", envMapMat);
            }
            fpscEnvMap = 0;
        }
        fpscEnvMap++;
    }

    fpsc++;

    double colonySize;
    if (colony.getColonySize() > 0)
    {
        colonySize = (double)colony.getStillAliveNum() / (double)colony.getColonySize();
    }
    else
    {
        colonySize = 0.0;
    }

    ui->colonySizeProgressBar->setValue(round(colonySize * 100));

    if ((double)colony.getStillAliveNum() / (double)colony.getColonySize() < 0.5)
    {

    }

}

void MainWindow::generatePerlinNoiseEnvMap(cv::Size size, double scale)
{
    PerlinNoiseMat pnm;
    envMapOriginalMat = pnm.generate(size, scale);

    updateEnvPixmap();
}

QPixmap MainWindow::cvMatToPixMap(cv::Mat mat)
{
    QPixmap pixmap = QPixmap::fromImage(QImage(reinterpret_cast<uchar const*>(mat.data),
                                              mat.cols,
                                              mat.rows,
                                              QImage::Format_Grayscale8));
    return pixmap;
}

void MainWindow::updateEnvPixmap()
{
    envMapMutex.lock();
    envMapMat = envMapOriginalMat.clone();
    envMapMutex.unlock();

    envMapPixmapItem->setPixmap(cvMatToPixMap(envMapMat));
    graphicsViewScene->setSceneRect(-100.0, -100.0, envMapMat.cols + 100, envMapMat.rows + 100);

    colony.setEnvMapMat(&envMapMat);
}

void MainWindow::activateLife(bool state)
{
    if (state)
    {
        colony.active();
        ui->startLifeButton->setText("Suspend Life");
        lifeTimer->start(1);
    }
    else
    {
        colony.deactive();
        ui->startLifeButton->setText("Resume Life");
        lifeTimer->stop();
    }
}

void MainWindow::on_startLifeButton_clicked(bool checked)
{
    activateLife(checked);
}

void MainWindow::on_populateColonyButton_clicked()
{
    activateLife(false);
    ui->startLifeButton->setChecked(false);
    ui->startLifeButton->setText("Start Life");

    if (!envMapMat.empty())
    {
        colony.killCreatures();
        colony.setEnvMapMat(&envMapMat);
        colony.populate(Colony::Type::A, 50);
    }
    else
    {
        QMessageBox *msgBox = new QMessageBox(0);
        msgBox->setText("Environment Map is not set");
        msgBox->exec();
        return;
    }
}

void MainWindow::on_generateEnvMapButton_clicked()
{
    on_killColonyButton_clicked();

    int pnmW = ui->perlinNoiseEnvMapWSpinBox->value();
    int pnmH = ui->perlinNoiseEnvMapHSpinBox->value();
    double pnmS = ui->perlinNoiseEnvMapSSpinBox->value();

    graphicsViewScene->setSceneRect(0.0, 0.0, pnmW, pnmH);

    generatePerlinNoiseEnvMap(cv::Size(pnmW, pnmH), pnmS);
}

void MainWindow::on_killColonyButton_clicked()
{
    activateLife(false);
    ui->startLifeButton->setChecked(false);
    ui->startLifeButton->setText("Start Life");

    lifeTimer->stop();

    if (!envMapMat.empty())
    {
        colony.deactive();
        colony.killCreatures();
    }
}
