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
}

void MainWindow::populateColony()
{
    colony = new Colony();

    QGraphicsItem *pCreatureGI;
    Creature *pCreature;

    for (double x = -1000; x < 1000; x += 200)
    {
        for (double y = -1000; y < 1000; y += 200)
        {
            pCreatureGI = new CreatureA(50, 50, QColor(0, 255, 0, 255));
            pCreature = dynamic_cast<Creature*>(pCreatureGI);
            pCreature->x = x;
            pCreature->y = y;
            colony->addCreature(pCreature);
            pCreatureGI->setPos(QPointF(pCreature->x, pCreature->y));
            pCreatureGI->setRotation(pCreature->a);
            pCreatureGI->setOpacity(1.0);
            scene->addItem(pCreatureGI);
        }
    }
}

void MainWindow::startLifeCallback(bool state)
{
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
    Creature *pCreature;
    QGraphicsItem *pGCreature;
    for (unsigned int i = 0; i < creatures.size(); i++)
    {
        pCreature = dynamic_cast<Creature*>(creatures[i]);

        int randInt;
        double randDouble;

        randInt = rand() % 1000;
        randInt *= (randInt > 500) ? 1 : -1;
        randDouble = (double)randInt / 1000.0;

        pCreature->x += randDouble * ((i % 2 == 1) ? 1.0 : -1.0);

        pCreature->y += randDouble * ((i % 2 == 1) ? 1.0 : -1.0);

        pCreature->a += randDouble;
    }
    if (fpsc > 40)
    {
        fpsc = 0;
        for (unsigned int i = 0; i < creatures.size(); i++)
        {
            pCreature = dynamic_cast<Creature*>(creatures[i]);
            pGCreature = dynamic_cast<QGraphicsItem*>(creatures[i]);
            pGCreature->setPos(QPointF(pCreature->x, pCreature->y));
            pGCreature->setRotation(pCreature->a);
        }
    }
    fpsc++;
}
