#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "view.h"

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

    View *view = new View();
    view->view()->setScene(scene);

    QVBoxLayout *layout = new QVBoxLayout;

    QHBoxLayout *toolsLayout = new QHBoxLayout;
    startButton = new QToolButton;
    startButton->setCheckable(true);
    startButton->setText("Begin Life");
    toolsLayout->addWidget(startButton);
    toolsLayout->addStretch();

    layout->addLayout(toolsLayout);
    layout->addWidget(view);
    setLayout(layout);

    setWindowTitle(tr("Simple Creature Evolution"));

    lifeTimer = new QTimer(this);

    connect(lifeTimer, SIGNAL(timeout()), this, SLOT(lifeTimerCallback()));
    connect(startButton, SIGNAL(clicked(bool)), this, SLOT(startLifeCallback(bool)));

    populateColony();

    addDebugGeometry();
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
//            pCreatureGI = new CreatureA(30, 30, QColor(0, 255, 0, 255));
//            pCreature = dynamic_cast<Creature*>(pCreatureGI);
//            pCreature->setX(x);
//            pCreature->setY(y);
//            colony->addCreature(pCreature);
//            pCreatureGI->setPos(QPointF(pCreature->getGX(), pCreature->getGY()));
//            pCreatureGI->setRotation(pCreature->getA());
//            pCreatureGI->setOpacity(1.0);
//            scene->addItem(pCreatureGI);
//        }
//    }

    pCreatureGI = new CreatureA(30.0, 30.0, QColor(0, 255, 0, 125));
    pCreature = dynamic_cast<Creature*>(pCreatureGI);
    pCreature->setX(0.0);
    pCreature->setY(0.0);
    colony->addCreature(pCreature);
    pCreatureGI->setPos(QPointF(pCreature->getX(), pCreature->getY()));
    pCreatureGI->setRotation(pCreature->getA());
    pCreatureGI->setOpacity(1.0);
    scene->addItem(pCreatureGI);
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
    vector<Creature *> creatures = colony->getColony();
    CreatureA *pCreature;
    QGraphicsItem *pGCreature;
    for (unsigned int i = 0; i < creatures.size(); i++)
    {
        pCreature = dynamic_cast<CreatureA *>(creatures[i]);

//        int randInt;
//        double randDouble;

//        randInt = rand() % 1000;
//        randInt *= (randInt > 500) ? 1 : -1;
//        randDouble = (double)randInt / 1000.0;

//        pCreature->x += randDouble * ((i % 2 == 1) ? 1.0 : -1.0);

//        pCreature->y += randDouble * ((i % 2 == 1) ? 1.0 : -1.0);

        pCreature->setA(pCreature->getA() + 0.1);
//        pCreature->setA(0.0);

        Receptor *lR = pCreature->getReceptors()[0];
        Receptor *rR = pCreature->getReceptors()[1];

        dline1->x2 = lR->gx;
        dline1->y2 = lR->gy;

        dline2->x2 = rR->gx;
        dline2->y2 = rR->gy;
    }
    if (fpsc > 40)
    {
        fpsc = 0;
        for (unsigned int i = 0; i < creatures.size(); i++)
        {
            pCreature = dynamic_cast<CreatureA *>(creatures[i]);
            pGCreature = dynamic_cast<QGraphicsItem*>(creatures[i]);
            pGCreature->setPos(QPointF(pCreature->getX(), pCreature->getY()));
            pGCreature->setRotation(pCreature->getA());
        }
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
