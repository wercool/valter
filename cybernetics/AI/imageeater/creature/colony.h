#ifndef COLONY_H
#define COLONY_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <mutex>

#include <creature/creaturegi.h>
#include <creature/creature.h>

class Colony
{
public:
    Colony();

    cv::Mat *getEnvMapMat() const;
    void setEnvMapMat(cv::Mat *value);

    void setEnvMapMutex(std::mutex *value);

    void addCreature(Creature *c);
    Creature *getCreature(int idx = 0);

protected:
    cv::Mat *envMapMat;
    std::mutex* envMapMutex = nullptr;

    std::vector<Creature *> creatures;
};

#endif // COLONY_H
