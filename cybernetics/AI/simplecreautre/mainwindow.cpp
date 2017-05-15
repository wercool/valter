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

    loadImageButton = new QPushButton;
    loadImageButton->setText("Load Image");

    startButton = new QToolButton;
    startButton->setCheckable(true);
    startButton->setText("Begin Life");


    toolsLayout->addWidget(perlinNoiseEnvironmentMapGenerateButton);
    toolsLayout->addWidget(loadImageButton);
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
    connect(loadImageButton, SIGNAL(clicked()), this, SLOT(loadImageCallback()));

//    addDebugGeometry();
}

void MainWindow::generateEnvironmentMap()
{
    PerlinNoiseEnvironment *pnenv = new PerlinNoiseEnvironment();
    view->setEnvMapMat(pnenv->generate(cv::Size(1000, 1000), -0.01));

    populateColony();
}

void MainWindow::loadImageCallback()
{
    string fileName = QFileDialog::getOpenFileName(this, tr("Open Image"), "/home/maska", tr("Image Files (*.png *.jpg *.jpeg *.bmp)")).toUtf8().constData();

    if (!fileName.empty())
    {
        cv::Mat srcImage = cv::imread(fileName, true);
        cv::Mat grayscaleImage;
        cv::cvtColor(srcImage, grayscaleImage, CV_BGRA2GRAY);

//        cv::imshow("Grayscaled image", grayscaleImage);

        view->setEnvMapMat(grayscaleImage);
        view->updateEnvMap();

        populateColony();
    }
}

void MainWindow::populateColony()
{
    colony = new Colony();

    QGraphicsItem *pCreatureGI;
    Creature *pCreature;

    for (int i = 0; i < 50; i++)
    {
        pCreatureGI = new CreatureA(20.0, 20.0, QColor(0, 255, 0, 255));
        pCreature = dynamic_cast<Creature*>(pCreatureGI);
        pCreature->setView(view);
        pCreature->setX((double)view->getEnvMapMat().cols / 2.0);
        pCreature->setY((double)view->getEnvMapMat().rows / 2.0);
        colony->addCreature(pCreature);
        pCreatureGI->setPos(QPointF(pCreature->getX(), pCreature->getY()));
        pCreatureGI->setRotation(pCreature->getA());
        pCreatureGI->setOpacity(1.0);
        scene->addItem(pCreatureGI);
    }

    qDebug("Generation %d [%d]", 0, colony->getColonySize());
}

void MainWindow::killColony()
{
    colony->killColony();
}

void MainWindow::startLifeCallback(bool state)
{
    if (state)
    {
        startButton->setText("Suspend Life");
        lifeTimer->start(1);
        colony->resumeLife();
    }
    else
    {
        startButton->setText("Resume Life");
        lifeTimer->stop();
        colony->suspendLife();
    }
}

void MainWindow::lifeTimerCallback()
{
    static int generation = 0;
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

        if ((double)colony->getStillAliveNumber() / (double)colony->getColonySize() < 0.5)
        {
            lifeTimer->stop();
            colony->stopLife();

            this_thread::sleep_for(std::chrono::milliseconds(100));

            Colony *nextGenerationColony = colony->getNextGenerationColony();
            colony = nextGenerationColony;

            scene->clear();
            view->update();

            PerlinNoiseEnvironment *pnenv = new PerlinNoiseEnvironment();
            view->setEnvMapMat(pnenv->generate(cv::Size(1000, 1000), -0.01));
            view->updateEnvMap();

            creatures = colony->getColony();
            for (unsigned int c = 0; c < creatures.size(); c++)
            {
                pCreature = dynamic_cast<CreatureA *>(creatures[c]);
                pGCreature = dynamic_cast<QGraphicsItem*>(creatures[c]);

                pCreature->setView(view);
                pCreature->setX((double)view->getEnvMapMat().cols / 2.0);
                pCreature->setY((double)view->getEnvMapMat().rows / 2.0);

                pGCreature->setPos(QPointF(pCreature->getX(), pCreature->getY()));
                pGCreature->setRotation(pCreature->getA());
                pGCreature->setOpacity(1.0);
                scene->addItem(pGCreature);
            }

            qDebug("Generation %d [%d]", ++generation, colony->getColonySize());

            view->update();

            lifeTimer->start();
            colony->resumeLife();
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
