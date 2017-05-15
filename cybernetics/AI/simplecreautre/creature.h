#ifndef CREATURE_H
#define CREATURE_H

#include <QPainter>
#include <QColor>
#include <QGraphicsItem>

#include <chrono>
#include <thread>
#include <mutex>
#include <math.h>

#include "view.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <utils/utils.h>

#include <neuralnetwork.h>

class Creature : public QGraphicsItem
{
public:
    Creature(double rx, double ry, const QColor &color);

public:
    // QGraphicsItem interface
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)Q_DECL_OVERRIDE;
    QRectF boundingRect() const Q_DECL_OVERRIDE;

    //Graphics properties
    double rx = 30.0;
    double ry = 30.0;
    QColor color;

    QColor fillColor;

    //Neural Network
    NeuralNetwork *nn;

    thread *lifeThread;

    double getX() const;
    int getIntX() const;
    void setX(double value);

    double getY() const;
    int getIntY() const;
    void setY(double value);

    double getA() const;
    void setA(double value);

    bool getSuspended() const;
    void setSuspended(bool value);

    bool getLifeSuspended() const;
    void setLifeSuspended(bool value);

    cv::Mat getEnvMatMap() const;
    void setEnvMatMap(cv::Mat value);

    int getDLifeTime() const;
    void setDLifeTime(int value);

    double getVitality() const;
    void setVitality(double value);

    bool getLifeStopped() const;
    void setLifeStopped(bool value);

    View *getView() const;
    void setView(View *value);

protected:
    // QGraphicsItem interface
    void mousePressEvent(QGraphicsSceneMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) Q_DECL_OVERRIDE;

    double vitality = 1.0;
    std::mutex env_map_mutex;

private:

    double x = 0.0;
    double y = 0.0;
    double a = 0.0;

    int dLifeTime = 5;

    bool lifeSuspended = true;
    bool lifeStopped = false;

    View *view;

    virtual void lifeThreadProcess();
};

#endif // CREATURE_H
