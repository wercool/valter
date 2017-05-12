#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "neuron.h"
#include "creaturea.h"
#include "creatureb.h"
#include "colony.h"

MainWindow::MainWindow(QWidget *parent) : QWidget(parent)
{
    setMinimumWidth(1024);
    setMinimumHeight(768);

    scene = new QGraphicsScene(this);
    scene->setItemIndexMethod(QGraphicsScene::NoIndex);

    view = new View();
    view->grpahicsView()->setScene(scene);

    QVBoxLayout *layout = new QVBoxLayout;

    QHBoxLayout *toolsLayout = new QHBoxLayout;

    perlinNoiseEnvironmentMapGenerateButton = new QPushButton;
    perlinNoiseEnvironmentMapGenerateButton->setText("Generate Environment Map");

    startButton = new QToolButton;
    startButton->setCheckable(true);
    startButton->setText("Begin Life");


    toolsLayout->addWidget(perlinNoiseEnvironmentMapGenerateButton);
    toolsLayout->addWidget(startButton);
    toolsLayout->addStretch();

    layout->addLayout(toolsLayout);
    layout->addWidget(view);
    setLayout(layout);

    setWindowTitle(tr("Simple Creature Evolution"));

    lifeTimer = new QTimer(this);

    connect(lifeTimer, SIGNAL(timeout()), this, SLOT(lifeTimerCallback()));
    connect(startButton, SIGNAL(clicked(bool)), this, SLOT(startLifeCallback(bool)));
    connect(perlinNoiseEnvironmentMapGenerateButton, SIGNAL(clicked()), this, SLOT(generateEnvironmentMap()));

//    addDebugGeometry();
}

void MainWindow::generateEnvironmentMap()
{
    PerlinNoiseEnvironment *pnenv = new PerlinNoiseEnvironment();
    view->setEnvMapMat(pnenv->generate(cv::Size(2000, 2000), -0.01));

    populateColony();

//    cv::Mat *mat = view->getEnvMapMat();
//    for(int y = 0; y < mat->rows; y++)
//    {
//        for(int x = 0; x < mat->rows; x++)
//        {
//            uchar intensity = mat->at<uchar>(y, x);
//            mat->at<uchar>(y, x) = 240;
//        }
//    }
//    for(int y =  mat->rows / 2 - 50; y < mat->rows / 2 + 50; y++)
//    {
//        for(int x = mat->rows / 2 - 50; x < mat->rows / 2 + 50; x++)
//        {
//            uchar intensity = mat->at<uchar>(y, x);
//            mat->at<uchar>(y, x) = 0;
//        }
//    }
//    view->updateEnvMap();
}

void MainWindow::populateColony()
{
    colony = new Colony();

    QGraphicsItem *pCreatureGI;
    Creature *pCreature;

//    for (double x = -1000; x < 1000; x += 200)
//    {
//        for (double y = -1000; y < 1000; y += 200)
//        {
//            pCreatureGI = new CreatureA(20.0, 20.0, QColor(0, 255, 0, 125));
//            pCreature = dynamic_cast<Creature*>(pCreatureGI);
//            pCreature->setEnvMatMap(view->getEnvMapMat());
//            pCreature->setX(view->getEnvMapMat()->rows / 2);
//            pCreature->setY(view->getEnvMapMat()->cols / 2);
//            colony->addCreature(pCreature);
//            pCreatureGI->setPos(QPointF(pCreature->getX(), pCreature->getY()));
//            pCreatureGI->setRotation(pCreature->getA());
//            pCreatureGI->setOpacity(1.0);
//            scene->addItem(pCreatureGI);
//        }
//    }

    for (int i = 0; i < 100; i++)
    {
        pCreatureGI = new CreatureA(20.0, 20.0, QColor(0, 255, 0, 255));
        pCreature = dynamic_cast<Creature*>(pCreatureGI);
        pCreature->setEnvMatMap(view->getEnvMapMat());
        pCreature->setX(view->getEnvMapMat()->rows / 2);
        pCreature->setY(view->getEnvMapMat()->cols / 2);
        colony->addCreature(pCreature);
        pCreatureGI->setPos(QPointF(pCreature->getX(), pCreature->getY()));
        pCreatureGI->setRotation(pCreature->getA());
        pCreatureGI->setOpacity(1.0);
        scene->addItem(pCreatureGI);
    }
}

void MainWindow::startLifeCallback(bool state)
{
    vector<Creature *> creatures = colony->getColony();
    for (unsigned int i = 0; i < creatures.size(); i++)
    {
        Creature *pCreature = dynamic_cast<Creature *>(creatures[i]);
        pCreature->setLifeSuspended(!state);
    }

    if (state)
    {
        startButton->setText("Suspend Life");
        lifeTimer->start(1);
    }
    else
    {
        startButton->setText("Resume Life");
        lifeTimer->stop();
    }
}

void MainWindow::lifeTimerCallback()
{
    static int fpsc = 0;
    static int fpscEnv = 0;

    vector<Creature *> creatures = colony->getColony();
    CreatureA *pCreature;
    QGraphicsItem *pGCreature;

    if (fpsc > 20)
    {
//        qDebug("Colony: %d / %d", colony->getStillAliveNumber(), colony->getColonySize());
        fpsc = 0;
        for (unsigned int i = 0; i < creatures.size(); i++)
        {
            pCreature = dynamic_cast<CreatureA *>(creatures[i]);
            pGCreature = dynamic_cast<QGraphicsItem*>(creatures[i]);

            if (pCreature->getVitality() > 0.0)
            {
                pGCreature->setPos(QPointF(pCreature->getX(), pCreature->getY()));
                pGCreature->setRotation(pCreature->getA());
                pGCreature->setOpacity(pCreature->getVitality());
            }

            if (fpscEnv > 10)
            {
                view->updateEnvMap();
                fpscEnv = 0;
            }
        }

        if ((double)colony->getStillAliveNumber()/ (double)colony->getColonySize() < 0.5)
        {
//            lifeTimer->stop();

//            PerlinNoiseEnvironment *pnenv = new PerlinNoiseEnvironment();
//            view->setEnvMapMat(pnenv->generate(cv::Size(2000, 2000), -0.01));

//            vector<Creature *> susrvivedCreatures = colony->getSurvived();
//            scene->clear();
//            view->update();

//            vector<Creature *> susrvivedCreatureNewGeneration;
//            QGraphicsItem *pCreatureGI;
//            Creature *pCreature;
//            for (unsigned int i = 0; i < susrvivedCreatures.size(); i++)
//            {
//                pCreatureGI = new CreatureA(20.0, 20.0, QColor(0, 255, 0, 255));
///*
//                pCreatureGI = new CreatureA(20.0, 20.0, QColor(0, 255, 0, 255));
//                pCreature = dynamic_cast<Creature*>(pCreatureGI);
//                pCreature->setEnvMatMap(view->getEnvMapMat());
//                pCreature->setX(view->getEnvMapMat()->rows / 2);
//                pCreature->setY(view->getEnvMapMat()->cols / 2);
//                colony->addCreature(pCreature);
//                pCreatureGI->setPos(QPointF(pCreature->getX(), pCreature->getY()));
//                pCreatureGI->setRotation(pCreature->getA());
//                pCreatureGI->setOpacity(1.0);
//                scene->addItem(pCreatureGI);
//*/
//            }

//            colony = new Colony();
//            view->updateEnvMap();
        }

        fpscEnv++;
    }
    fpsc++;
}

void MainWindow::addDebugGeometry()
{
    //debug geometry
    dline1 = new DLine(Qt::magenta);
    dline2 = new DLine(Qt::cyan);
    dlineX = new DLine(Qt::red);
    dlineY = new DLine(Qt::blue);

    dline1->x1 = 0.0;
    dline1->y1 = 0.0;
    dline1->x2 = 0.0;
    dline1->y2 = 0.0;

    dline2->x1 = 0.0;
    dline2->y1 = 0.0;
    dline2->x2 = 0.0;
    dline2->y2 = 0.0;

    dlineX->x1 = -1000.0;
    dlineX->y1 = 0.0;
    dlineX->x2 = 1000.0;
    dlineX->y2 = 0.0;

    dlineY->x1 = 0.0;
    dlineY->y1 = -1000.0;
    dlineY->x2 = 0.0;
    dlineY->y2 = 1000.0;

    scene->addItem(dline1);
    scene->addItem(dline2);
    scene->addItem(dlineX);
    scene->addItem(dlineY);
}
