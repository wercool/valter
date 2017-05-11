#ifndef COLONY_H
#define COLONY_H

#include <creature.h>

using namespace std;

class Colony
{
public:
    Colony();

    void addCreature(Creature *pCreature);
    vector<Creature *> getColony();
    void killColony();

private:
    vector<Creature *> creatures;
};

#endif // COLONY_H
