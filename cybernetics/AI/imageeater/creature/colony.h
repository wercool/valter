#ifndef COLONY_H
#define COLONY_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <mutex>

#include <utils/generic-utils.h>

#include <creature/creaturegi.h>
#include <creature/creature.h>

#include <creature/creaturea.h>
#include <creature/creatureb.h>
#include <creature/creaturec.h>

class Colony
{
public:
    Colony();
    Colony(QGraphicsScene *graphicsViewScene);

    enum Type
    {
        A,
        B,
        C
    };

    void populate(Type type, unsigned int size, vector<NeuralNetwork *> nns = {});
    void active();
    void deactive();

    cv::Mat *getEnvMapMat() const;
    void setEnvMapMat(cv::Mat *value);

    void setEnvMapMutex(std::mutex *value);

    void addCreature(Creature *pCreature);
    Creature *getCreature(int idx = 0);

    int getStillAliveNum();
    int getColonySize();

    void killCreatures();

    std::vector<Creature *> getCreatures() const;
    vector<Creature *> getSurvivedCreatures();

    vector<double> populateNextGeneration();

    vector<Creature *> sortedByVitality(vector<Creature *> creatures);
    vector<Creature *> sortedBySaturation(vector<Creature *> creatures);
    vector<Creature *> sortedByPathLength(vector<Creature *> creatures);
    vector<Creature *> sortedByPathLengthAndSaturation(vector<Creature *> creatures);
    vector<Creature *> sortedByDistanceToTargetPoint(vector<Creature *> creatures);
    vector<Creature *> sortedByDistanceToTargetPointAndEffectiveness(vector<Creature *> creatures);

    static bool sortByVitality(Creature * c1, Creature * c2);
    static bool sortBySaturation(Creature * c1, Creature * c2);
    static bool sortByPathLength(Creature * c1, Creature * c2);
    static bool sortByPathLengthAndSaturation(Creature * c1, Creature * c2);
    static bool sortByDistanceToTargetPoint(Creature * c1, Creature * c2);
    static bool sortByDistanceToTargetPointAndEffectiveness(Creature * c1, Creature * c2);

    int getGeneration() const;
    void setGeneration(int value);

    Colony::Type getSelectedType() const;

protected:
    QGraphicsScene *graphicsViewScene;
    cv::Mat *envMapMat;
    std::mutex *envMapMutex = nullptr;

    std::vector<Creature *> creatures;

    Colony::Type selectedType;

private:
    int generation = 0;
    bool goalIsReached = false;
};

#endif // COLONY_H
