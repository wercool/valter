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
    Creature(double w, double h, const QColor &color);

public:
    // QGraphicsItem interface
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)Q_DECL_OVERRIDE;
    QRectF boundingRect() const Q_DECL_OVERRIDE;

    //Graphics properties
    double w = 50.0;
    double h = 50.0;
    double x = 0.0;
    double y = 0.0;
    double a = 0.0;
    QColor fillColor;

    //Neural Network
    NeuralNetwork *nn;

    thread *lifeThread;

protected:
    // QGraphicsItem interface
    void mousePressEvent(QGraphicsSceneMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) Q_DECL_OVERRIDE;

private:
    //Graphics properties
    QColor color;

    virtual void lifeThreadProcess();
};

#endif // CREATURE_H
