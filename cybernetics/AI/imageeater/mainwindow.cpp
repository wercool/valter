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
    graphicsViewScene->setItemIndexMethod(QGraphicsScene::NoIndex);
    ui->graphicsView->setScene(graphicsViewScene);
    ui->graphicsView->scene()->clear();

    envMapPixmapItem = new QGraphicsPixmapItem();
    ui->graphicsView->scene()->addItem(envMapPixmapItem);

    lifeTimer = new QTimer(this);
    connect(lifeTimer, SIGNAL(timeout()), this, SLOT(lifeTimerCallback()));

    colony = Colony(graphicsViewScene);
    colony.setEnvMapMutex(&envMapMutex);

    cv::namedWindow("Env Map", cv::WINDOW_KEEPRATIO);

    graphWidget = new QWidget();
    graphWidgetLayout = new QVBoxLayout;
    graphWidget->setMinimumWidth(640);
    graphWidget->setMinimumHeight(480);
    graphWidget->setLayout(graphWidgetLayout);
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

        ui->regenerateEnvMapCheckBox->setChecked(false);

        updateEnvPixmap();
    }
}

void MainWindow::lifeTimerCallback()
{
    lifeCycleCnt++;

    static int fpsc = 0;
    static int fpscEnvMap = 0;

    // Drawing
    if (fpsc > 30)
    {

        vector<Creature *> pCreatures = colony.getCreatures();

        for (unsigned int i = 0; i < pCreatures.size(); i++)
        {
            Creature *pCreature = pCreatures[i];
            if (pCreature != nullptr)
            {
                QGraphicsItem *pCreatureGI = static_cast<QGraphicsItem *>(pCreature);

                if (ui->updateCreaturesCheckBox->isChecked())
                {
                    pCreatureGI->setVisible(true);
                }
                else
                {
                    pCreatureGI->setVisible(false);
                }

                if (pCreatureGI != nullptr)
                {
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
                else
                {
                    continue;
                }
            }
            else
            {
                continue;
            }
        }


        if (fpscEnvMap > 2)
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

        fpsc = 0;
        fpscEnvMap++;
    }

    fpsc++;

    int colonySize;
    if (colony.getColonySize() > 0)
    {
        colonySize = (int)((double)colony.getStillAliveNum() / (double)colony.getColonySize() * 100);
    }
    else
    {
        colonySize = 0;
    }
    ui->colonySizeProgressBar->setValue(colonySize);

    if (colony.getSelectedType() == Colony::Type::A)
    {
        cTypeAHandler(colonySize);
    }
    if (colony.getSelectedType() == Colony::Type::B)
    {
        cTypeBHandler(colonySize);
    }

}


void MainWindow::cTypeAHandler(int colonySize)
{
    int brightness = cv::sum(envMapMat)[0];
    int darkness = (int)( ( (double)envMapOriginalMatBrightness / (double)brightness ) * 100);
    ui->envMapDarknessProgressBar->setValue(darkness);

    if (colonySize < 11)
    {
        lifeTimer->stop();
        colony.deactive();

        lifeCycleCnt = 0;

        if (ui->regenerateEnvMapCheckBox->isChecked())
        {
            on_generateEnvMapButton_clicked();
        }

        fitnessFunctionGraph[colony.getGeneration()] = (double)100 - darkness;
        updateGraph();

        updateEnvPixmap();

        colony.populateNextGeneration();

        ui->statusBar->showMessage(format_string("Generation #%d", colony.getGeneration() + 1).c_str());

        colony.active();
        lifeTimer->start();
    }
}

void MainWindow::cTypeBHandler(int colonySize)
{
    if (lifeCycleCnt > 2000 || colonySize < 25)
    {
        lifeCycleCnt = 0;

        lifeTimer->stop();
        colony.deactive();

        double maxCurFitness = colony.populateNextGeneration();

        fitnessFunctionGraph[colony.getGeneration()] = maxCurFitness;
        updateGraph();

        ui->statusBar->showMessage(format_string("Generation #%d", colony.getGeneration() + 1).c_str());

        colony.active();
        lifeTimer->start();
    }
}

void MainWindow::generatePerlinNoiseEnvMap(cv::Size size, double scale)
{
    PerlinNoiseMat pnm;
    envMapOriginalMat = pnm.generate(size, scale);

    updateEnvPixmap();
}

void MainWindow::updateEnvPixmap()
{
    if (!envMapOriginalMat.empty())
    {
        while(!envMapMutex.try_lock());
        envMapMat = envMapOriginalMat.clone();
        colony.setEnvMapMat(&envMapMat);
        envMapMutex.unlock();

        envMapOriginalMatBrightness = cv::sum(envMapMat)[0];

        envMapPixmapItem->setPixmap(cvMatToPixMap(envMapMat));
        graphicsViewScene->setSceneRect(-100.0, -100.0, envMapMat.cols + 100, envMapMat.rows + 100);
    }
    else
    {
        QMessageBox *msgBox = new QMessageBox(0);
        msgBox->setText("Env Map is not set");
        msgBox->exec();
        return;
    }
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
        lifeTimer->stop();
        colony.deactive();
        ui->startLifeButton->setText("Resume Life");
    }
}

void MainWindow::on_startLifeButton_clicked(bool checked)
{
    if (colony.getColonySize() > 0)
    {
        activateLife(checked);
    }
    else
    {
        QMessageBox *msgBox = new QMessageBox(0);
        msgBox->setText("No Colony set");
        msgBox->exec();
        ui->startLifeButton->setChecked(!checked);
        return;
    }
}

void MainWindow::on_populateColonyButton_clicked()
{
    fitnessFunctionGraph.clear();
    activateLife(false);
    ui->startLifeButton->setChecked(false);
    ui->startLifeButton->setText("Start Life");

    if (!envMapMat.empty())
    {
        colony.killCreatures();
        colony.setEnvMapMat(&envMapMat);
        switch (ui->creatureTypeComboBox->currentIndex())
        {
            case 0:
                colony.populate(Colony::Type::A, ui->colonySizeSpinBox->value());
            break;
            case 1:
                colony.populate(Colony::Type::B, ui->colonySizeSpinBox->value());
            break;
        }

        ui->statusBar->showMessage(format_string("Generation #%d", colony.getGeneration()).c_str());
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

void MainWindow::on_updateEnvMapButton_clicked()
{
    updateEnvPixmap();
}

void MainWindow::on_graphPushButton_clicked()
{
    updateGraph();
}

void MainWindow::updateGraph()
{
    QtCharts::QLineSeries *series = new QtCharts::QLineSeries();

    typedef std::map<int, double> fitnessFunctionGraphType;

    for (fitnessFunctionGraphType::iterator iterator = fitnessFunctionGraph.begin(); iterator != fitnessFunctionGraph.end(); ++iterator)
    {
        series->append(iterator->first, iterator->second);
    }

    QtCharts::QChart *chart = new QtCharts::QChart();
    chart->legend()->hide();
    chart->addSeries(series);
    chart->createDefaultAxes();
    chart->setTitle("Fitness Function");

    QtCharts::QChartView *chartView = new QtCharts::QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);

    QLayoutItem *child = graphWidgetLayout->takeAt(0);
    delete child;

    graphWidgetLayout->addWidget(chartView);

    graphWidget->show();
}
