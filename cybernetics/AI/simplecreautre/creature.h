#ifndef CREATURE_H
#define CREATURE_H

#include <QPainter>
#include <QColor>
#include <QGraphicsItem>

#include <chrono>
#include <thread>
#include <math.h>

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

    QColor fillColor;

    //Neural Network
    NeuralNetwork *nn;

    thread *lifeThread;

    double getX() const;
    void setX(double value);

    double getY() const;
    void setY(double value);

    double getA() const;
    void setA(double value);

    bool getSuspended() const;
    void setSuspended(bool value);

    bool getLifeSuspended() const;
    void setLifeSuspended(bool value);

    int getDLiefeTime() const;
    void setDLiefeTime(int value);

protected:
    // QGraphicsItem interface
    void mousePressEvent(QGraphicsSceneMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) Q_DECL_OVERRIDE;

private:
    //Graphics properties
    QColor color;

    double x = 0.0;
    double y = 0.0;
    double a = 0.0;

    int dLiefeTime = 10;

    bool lifeSuspended = true;

    virtual void lifeThreadProcess();
};

#endif // CREATURE_H
