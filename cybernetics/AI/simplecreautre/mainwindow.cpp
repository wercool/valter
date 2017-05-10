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

    populateColony();

    View *view = new View();
    view->view()->setScene(scene);

    QVBoxLayout *layout = new QVBoxLayout;

    QHBoxLayout *toolsLayout = new QHBoxLayout;
    QToolButton *startButton = new QToolButton;
    startButton->setCheckable(true);
    startButton->setText("Start Life");
    toolsLayout->addWidget(startButton);
    toolsLayout->addStretch();

    layout->addLayout(toolsLayout);
    layout->addWidget(view);
    setLayout(layout);

    setWindowTitle(tr("Simple Creature Evolution"));

    lifeTimer = new QTimer(this);

    connect(lifeTimer, SIGNAL(timeout()), this, SLOT(lifeTimerCallback()));
    connect(startButton, SIGNAL(clicked(bool)), this, SLOT(startLifeCallback(bool)));
}

void MainWindow::populateColony()
{
//    for (int x = -1000; x < 1500; x += 500)
//    {
//        for (int y = -1000; y < 1500; y += 500)
//        {
//            QGraphicsItem *item = new Creature(50, 50, QColor(0, 255, 0, 255));
//            item->setPos(QPointF(x, y));
//            scene->addItem(item);
//        }
//    }

    colony = new Colony();

    QGraphicsItem *giCreature;
    Creature *pCreature;

    giCreature = new CreatureA(50, 50, QColor(0, 255, 0, 255));
    pCreature = dynamic_cast<Creature*>(giCreature);
    pCreature->x = 0.0;
    pCreature->y = 0.0;
    pCreature->a = 0.0;
    colony->addCreature(pCreature);
    giCreature->setPos(QPointF(pCreature->x, pCreature->y));
    giCreature->setRotation(pCreature->a);
    scene->addItem(giCreature);

    giCreature = new CreatureB(50, 50, QColor(0, 255, 0, 255));
    pCreature = dynamic_cast<Creature*>(giCreature);
    pCreature->x = 100.0;
    pCreature->y = 0.0;
    pCreature->a = 0.0;
    colony->addCreature(pCreature);
    giCreature->setPos(QPointF(pCreature->x, pCreature->y));
    giCreature->setRotation(pCreature->a);
    scene->addItem(giCreature);

//    creature = new CreatureB(50, 50, QColor(0, 255, 0, 255));
//    creature->setPos(QPointF(0, 0));
//    scene->addItem(creature);

    //    Neuron *n = new Neuron(Neuron::NeuronType::Hidden, Neuron::NeuronFunction::Perceptron);
}

void MainWindow::startLifeCallback(bool state)
{
    if (state)
    {
        lifeTimer->start(10);
    }
    else
    {
        lifeTimer->stop();
    }
}

void MainWindow::lifeTimerCallback()
{
    vector<Creature *> creatures = colony->getColony();
    for (unsigned int i = 0; i < creatures.size(); i++)
    {
        Creature *pCreature = dynamic_cast<Creature*>(creatures[i]);
        pCreature->x += 0.1;
        pCreature->y += 0.1;
        pCreature->a += 0.5;
        QGraphicsItem *pGCreature = dynamic_cast<QGraphicsItem*>(creatures[i]);
        pGCreature->setPos(QPointF(pCreature->x, pCreature->y));
        pGCreature->setRotation(pCreature->a);
    }
}
