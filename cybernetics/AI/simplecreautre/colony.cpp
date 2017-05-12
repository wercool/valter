#include "colony.h"

Colony::Colony()
{

}

void Colony::addCreature(Creature *pCreature)
{
    creatures.push_back(pCreature);
}

vector<Creature *> Colony::getColony()
{
    return creatures;
}

void Colony::killColony()
{
    creatures.clear();
}

int Colony::getColonySize()
{
    return (int)creatures.size();
}

int Colony::getStillAliveNumber()
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

vector<Creature *> Colony::getSurvived()
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
