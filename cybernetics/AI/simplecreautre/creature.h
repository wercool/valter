#ifndef CREATURE_H
#define CREATURE_H

#include <QPainter>
#include <QColor>
#include <QGraphicsItem>

#include <chrono>
#include <thread>

#include <neuralnetwork.h>

class Creature : public QGraphicsItem
{
public:
    Creature(int w, int h, const QColor &color);

public:
    // QGraphicsItem interface
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)Q_DECL_OVERRIDE;
    QRectF boundingRect() const Q_DECL_OVERRIDE;

    //Graphics properties
    int w;
    int h;
    double x;
    double y;
    double a;
    QColor color;
    QColor fillColor;
    QPointF mousePos;

    //Neural Network properties
    void createNeuralNetwork();

protected:
    // QGraphicsItem interface
    void mousePressEvent(QGraphicsSceneMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) Q_DECL_OVERRIDE;

private:
    //Neural Network properties
    NeuralNetwork nn;
};

#endif // CREATURE_H
