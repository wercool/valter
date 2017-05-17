#include "colony.h"

Colony::Colony()
{

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

void Colony::addCreature(Creature *c)
{
    creatures.push_back(c);
}

Creature *Colony::getCreature(int idx)
{
    return creatures.at(idx);
}
