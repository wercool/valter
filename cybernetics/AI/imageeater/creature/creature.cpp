#include "creature.h"

Creature::Creature(double rx, double ry, const QColor &color): CreatureGI(rx, ry, color)
{
    nn = new NeuralNetwork();
}

Creature::~Creature()
{
    qDebug("Creature #%d is being killed...", id);



    Receptor *receptor;
    for (unsigned int i = 0; i < receptors.size(); i++)
    {
        receptor = receptors[i];
        delete receptor;
    }

    delete nn;

    qDebug("Creature #%d killed", id);
}

void Creature::initNN(int hiddenNeuronsNum, int outputNeuronsNum)
{
    Q_UNUSED(hiddenNeuronsNum)
    Q_UNUSED(outputNeuronsNum)
    // to be override in nested
}

void Creature::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    CreatureGI::paint(painter, option, widget);
}

void Creature::setEnvMapMat(cv::Mat *value)
{
    envMapMat = value;
}

void Creature::setEnvMapMutex(std::mutex *value)
{
    envMapMutex = value;
}

void Creature::startLife()
{
    alive = true;
    if (lifeThread == nullptr)
    {
        lifeThread = new std::thread(&Creature::lifeThreadProcess, this);
    }
}

void Creature::suspendLife()
{
    if (lifeThread != nullptr)
    {
        alive = false;
        lifeThread->join();
        delete lifeThread;
    }
}

NeuralNetwork *Creature::getNN() const
{
    return nn;
}

void Creature::lifeThreadProcess()
{
    // to be override in nested
}

int Creature::getDLifeTime() const
{
    return dLifeTime;
}

void Creature::setDLifeTime(int value)
{
    dLifeTime = value;
}

double Creature::getSaturation() const
{
    return saturation;
}

double Creature::getVitality() const
{
    return vitality;
}
