#ifndef CREATURE_H
#define CREATURE_H

#include <chrono>
#include <thread>
#include <mutex>
#include <math.h>

#include <creature/creaturegi.h>
#include <creature/receptor.h>
#include <creature/neuralnetwork.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

class Creature : public CreatureGI
{
public:
    Creature(double rx, double ry, const QColor &color);
    ~Creature();

    int id = 0;

    virtual void initNN(int hiddenNeuronsNum, int outputNeuronsNum);
    NeuralNetwork *getNN() const;
    void setNN(NeuralNetwork *value);

    void setEnvMapMat(cv::Mat *value);
    void setEnvMapMutex(mutex *value);

    void startLife();
    void suspendLife();

    double getVitality() const;
    double getSaturation() const;
    double getPathLength() const;

    void setTargetPoint(const cv::Point2f &value);
    void setDistanceToTargetPoint(double value);
    double getDistanceToTargetPoint() const;

    void setDLifeTime(int value);
    int getDLifeTime() const;

    // QGraphicsItem interface
public:
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

protected:
    vector<Receptor *> receptors;
    NeuralNetwork *nn;

    cv::Mat *envMapMat;
    std::mutex *envMapMutex;

    thread *lifeThread = nullptr;
    virtual void lifeThreadProcess();
    bool alive = false;

    double vitality = 1.0;
    double saturation = 0.0;
    double pathLength = 0.0;

    cv::Point2f targetPoint;
    double distanceToTargetPoint = 0.0;

    int dLifeTime = 100;
};

#endif // CREATURE_H
