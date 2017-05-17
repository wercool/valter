#ifndef CREATUREGI_H
#define CREATUREGI_H

#include <QtWidgets>
#include <QPainter>
#include <QColor>
#include <QGraphicsItem>

class CreatureGI : public QGraphicsItem
{
public:
    CreatureGI(double rx, double ry, const QColor &color);

public:
    // QGraphicsItem interface
    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

public:
    double getX() const;
    int getIntX() const;
    void setX(double value);

    double getY() const;
    int getIntY() const;
    void setY(double value);

    double getA() const;
    void setA(double value);

    double getRx() const;

    double getRy() const;

protected:
    // QGraphicsItem interface
    void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;

protected:
    double rx = 30.0;
    double ry = 30.0;

    double rxI = 30.0;
    double ryI = 30.0;

    QColor color;
    QColor fillColor;
    double x = 0.0;
    double y = 0.0;
    double a = 0.0;
};

#endif // CREATUREGI_H
