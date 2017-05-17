#include "colony.h"

Colony::Colony()
{

}

Colony::Colony(QGraphicsScene *graphicsViewScene)
{
    this->graphicsViewScene = graphicsViewScene;
}

void Colony::populate(Colony::Type type, unsigned int size)
{
    for (unsigned i = 0; i < size; i++)
    {
        Creature *pCreature;

        switch (type)
        {
            case Type::A:
            {
                CreatureA *pCreatureA = new CreatureA(20.0, 20.0, QColor(0, 255, 0, 255));

                pCreatureA->initNN(/*hidden neurons*/ 5, /*output neurons*/3);

                pCreatureA->setX((double)envMapMat->cols / 2.0);
                pCreatureA->setY((double)envMapMat->rows / 2.0);
                pCreatureA->setA(0.0);

                pCreature = dynamic_cast<Creature *>(pCreatureA);
            }
            break;
            case Type::B:
                //...
            break;
            case Type::C:
                //...
            break;
        }

        pCreature->id = i;
        pCreature->setEnvMapMat(envMapMat);
        pCreature->setEnvMapMutex(envMapMutex);
        addCreature(pCreature);
    }
}

void Colony::active()
{
    Creature *creature;
    for (unsigned int i = 0; i < creatures.size(); i++)
    {
        creature = creatures[i];
        creature->startLife();
    }
}

void Colony::deactive()
{
    Creature *creature;
    for (unsigned int i = 0; i < creatures.size(); i++)
    {
        creature = creatures[i];
        creature->suspendLife();
    }
}

cv::Mat *Colony::getEnvMapMat() const
{
    return envMapMat;
}

void Colony::setEnvMapMat(cv::Mat *value)
{
    envMapMutex->lock();
    envMapMat = value;
    envMapMutex->unlock();
}

void Colony::setEnvMapMutex(std::mutex *value)
{
    envMapMutex = value;
}

void Colony::addCreature(Creature *pCreature)
{
    QGraphicsItem *pCreatureGI = dynamic_cast<QGraphicsItem *>(pCreature);

    pCreatureGI->setPos(QPointF(pCreature->getX(), pCreature->getY()));
    pCreatureGI->setRotation(pCreature->getA());
    pCreatureGI->setOpacity(1.0);

    graphicsViewScene->addItem(pCreatureGI);

    creatures.push_back(pCreature);
}

Creature *Colony::getCreature(int idx)
{
    return creatures.empty() ? nullptr : creatures.at(idx);
}

int Colony::getStillAliveNum()
{
    int sillAlive = 0;
    for (unsigned int i = 0 ; i < creatures.size(); i++)
    {
        Creature *c = creatures[i];
        if (c->getVitality() > 0.0)
        {
            sillAlive++;
        }
    }
    return sillAlive;
}

int Colony::getColonySize()
{
    return (int)creatures.size();
}

void Colony::killCreatures()
{
    Creature *pCreature;
    QGraphicsItem *pCreatureGI;
    for (unsigned int i = 0; i < creatures.size(); i++)
    {
        pCreature = creatures[i];

        pCreature->suspendLife();

        pCreatureGI = dynamic_cast<QGraphicsItem *>(pCreature);
        graphicsViewScene->removeItem(pCreatureGI);

        delete pCreature;
    }

    creatures.clear();
}

std::vector<Creature *> Colony::getCreatures() const
{
    return creatures;
}
