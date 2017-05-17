#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    setMinimumWidth(1024);
    setMinimumHeight(768);

    ui->setupUi(this);

    ui->graphicsView->setRenderHint(QPainter::Antialiasing, true);
    ui->graphicsView->setDragMode(QGraphicsView::RubberBandDrag);
    ui->graphicsView->setOptimizationFlags(QGraphicsView::DontAdjustForAntialiasing | QGraphicsView::DontSavePainterState);
    ui->graphicsView->setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
    ui->graphicsView->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    ui->graphicsView->setCacheMode(QGraphicsView::CacheNone);

    graphicsViewScene = new QGraphicsScene;
    graphicsViewScene->setSceneRect(0.0, 0.0, 4000.0, 4000.0);
    ui->graphicsView->setScene(graphicsViewScene);
    ui->graphicsView->scene()->clear();

    envMapPixmapItem = new QGraphicsPixmapItem();
    ui->graphicsView->scene()->addItem(envMapPixmapItem);

    lifeTimer = new QTimer(this);
    connect(lifeTimer, SIGNAL(timeout()), this, SLOT(lifeTimerCallback()));
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

        envMapMat = envMapOriginalMat.clone();

        QPixmap envMapPixmap = QPixmap::fromImage(QImage(reinterpret_cast<uchar const*>(envMapMat.data),
                                                  envMapMat.cols,
                                                  envMapMat.rows,
                                                  QImage::Format_Grayscale8));

        envMapPixmapItem->setPixmap(envMapPixmap);
    }
}

void MainWindow::lifeTimerCallback()
{
//    QPixmap envMapPixmap = QPixmap::fromImage(QImage(reinterpret_cast<uchar const*>(envMapMat.data),
//                                              envMapMat.cols,
//                                              envMapMat.rows,
//                                              QImage::Format_Grayscale8));

//    envMapPixmapItem->setPixmap(envMapPixmap);

//    envMapMutex.lock();
//    envMapMat = envMapOriginalMat.clone();
//    envMapMutex.unlock();

    Creature *creature = colony.getCreature(0);
    if (creature != nullptr)
    {
        QGraphicsItem *creatureGI;
        creatureGI = dynamic_cast<QGraphicsItem *>(creature);
        creatureGI->setX(creatureGI->x() + 1.0);
        creatureGI->setY(creatureGI->y() + 1.0);

    //    qDebug("lifeTimerCallback >>>>> %.4f", creatureGI->x());

        ui->graphicsView->scene()->removeItem(creatureGI);
        delete colony.getCreature(0);
    }
}

void MainWindow::on_startLifeButton_clicked(bool checked)
{
    if (checked)
    {
        ui->startLifeButton->setText("Suspend Life");
        lifeTimer->start(30);
    }
    else
    {
        ui->startLifeButton->setText("Resume Life");
        lifeTimer->stop();
    }
}

void MainWindow::on_populateColonyButton_clicked()
{
    Creature *creature = new Creature(20.0, 20.0, QColor(0, 255, 0, 255));

    creature->setPos(QPointF(2000.0, 2000.0));
    creature->setRotation(0.0);
    creature->setOpacity(1.0);
    graphicsViewScene->addItem(dynamic_cast<QGraphicsItem *>(creature));

    colony.addCreature(creature);

//    delete creature;
}
