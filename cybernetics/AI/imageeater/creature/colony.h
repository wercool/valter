#ifndef COLONY_H
#define COLONY_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <mutex>

#include <utils/generic-utils.h>

#include <creature/creaturegi.h>
#include <creature/creature.h>

#include <creature/creaturea.h>

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

    void populate(Type type, unsigned int size);
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

protected:
    QGraphicsScene *graphicsViewScene;
    cv::Mat *envMapMat;
    std::mutex *envMapMutex = nullptr;

    std::vector<Creature *> creatures;
};

#endif // COLONY_H
