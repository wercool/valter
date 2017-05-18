#include "colony.h"

Colony::Colony()
{

}

Colony::Colony(QGraphicsScene *graphicsViewScene)
{
    this->graphicsViewScene = graphicsViewScene;
}

void Colony::populate(Colony::Type type, unsigned int size, vector<NeuralNetwork *> nns)
{
    for (unsigned i = 0; i < size; i++)
    {
        Creature *pCreature;

        switch (type)
        {
            case Type::A:
            {
                CreatureA *pCreatureA = new CreatureA(20.0, 20.0, QColor(0, 255, 0, 255));

                if (nns.size() == 0)
                {
                    setGeneration(0);
                    pCreatureA->initNN(/*hidden neurons*/ 5, /*output neurons*/3);
                }
                else
                {
                    pCreatureA->setNN(nns[i]);
                }

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
        pCreature->setDLifeTime(pCreature->getDLifeTime() * getColonySize());
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
    envMapMat = value;
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

vector<Creature *> Colony::getSurvivedCreatures()
{
    vector<Creature *> survivedCreatures;

    for (unsigned int i = 0 ; i < creatures.size(); i++)
    {
        Creature *c = creatures[i];
        if (c->getVitality() > 0.0)
        {
            survivedCreatures.push_back(c);
        }
    }

    return survivedCreatures;
}

void Colony::populateNextGeneration()
{
    vector<Creature *> pCreatures = sortedByPathLengthAndSaturation(getSurvivedCreatures());
    int selectedSize = (int)pCreatures.size();

//    vector<Creature *> pCreatures = sortedByPathLength(getCreatures());
//    vector<Creature *> pCreatures = sortedByPathLengthAndSaturation(getCreatures());
//    int selectedSize = round((double)pCreatures.size() * 0.3);

    vector<NeuralNetwork *> selectedNNs;

    for (int i = 0; i < selectedSize; i++)
    {
        Creature *pCreature = pCreatures[i];
        selectedNNs.push_back(new NeuralNetwork(pCreature->getNN()));
    }

    int selectedNNsSize = (int)selectedNNs.size();
    int lackingNNsSize = getColonySize() - selectedNNsSize;

    for (int i = 0; i < lackingNNsSize; i++)
    {
        int randNNIdx = std::rand() % selectedNNs.size();
        NeuralNetwork *nn = selectedNNs[randNNIdx];

        NeuralNetwork *mnn = new NeuralNetwork(nn);
        mnn->mutateNeurons();
        selectedNNs.push_back(mnn);
    }

    killCreatures();

    populate(Type::A, (int)selectedNNs.size(), selectedNNs);

    setGeneration(getGeneration() + 1);
}

vector<Creature *> Colony::sortedByVitality(vector<Creature *> creatures)
{
    std::sort(creatures.begin(), creatures.end(), Colony::sortByVitality);
    return creatures;
}

vector<Creature *> Colony::sortedBySaturation(vector<Creature *> creatures)
{
    std::sort(creatures.begin(), creatures.end(), Colony::sortBySaturation);
    return creatures;
}

vector<Creature *> Colony::sortedByPathLength(vector<Creature *> creatures)
{
    std::sort(creatures.begin(), creatures.end(), Colony::sortByPathLength);
    return creatures;
}

vector<Creature *> Colony::sortedByPathLengthAndSaturation(vector<Creature *> creatures)
{
    std::sort(creatures.begin(), creatures.end(), Colony::sortByPathLengthAndSaturation);
    return creatures;
}

bool Colony::sortByVitality(Creature *c1, Creature *c2)
{
    return (c1->getVitality() > c2->getVitality());
}

bool Colony::sortBySaturation(Creature *c1, Creature *c2)
{
    return (c1->getSaturation() > c2->getSaturation());
}

bool Colony::sortByPathLength(Creature *c1, Creature *c2)
{
    return (c1->getPathLength() > c2->getPathLength());
}

bool Colony::sortByPathLengthAndSaturation(Creature *c1, Creature *c2)
{
    double c1W = 0.0;
    double c2W = 0.0;
    c1W += c1->getPathLength() + c1->getSaturation();
    c2W += c2->getPathLength() + c2->getSaturation();
    return (c1W > c2W);
}

int Colony::getGeneration() const
{
    return generation;
}

void Colony::setGeneration(int value)
{
    generation = value;
}

