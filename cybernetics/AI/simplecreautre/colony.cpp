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
