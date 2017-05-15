#ifndef COLONY_H
#define COLONY_H

#include <creature.h>

#include "creaturea.h"

using namespace std;

class Colony
{
public:
    Colony();

    void addCreature(Creature *pCreature);
    vector<Creature *> getColony();

    Colony *getNextGenerationColony();

    void killColony();

    int getColonySize();
    int getStillAliveNumber();

    void stopLife();

    void suspendLife();
    void resumeLife();

    vector<Creature *> getSurvived();

private:
    vector<Creature *> creatures;

    static bool sortByVitality(Creature * c1, Creature * c2);
    static bool sortByPathLength(Creature * c1, Creature * c2);
};

#endif // COLONY_H
