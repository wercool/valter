#ifndef VALTER3D_H
#define VALTER3D_H

#include <QWidget>
#include <QtQuick/QQuickView>

class Valter3d : public QWidget
{
    Q_OBJECT
public:
    QQuickView *valter3dView;
    explicit Valter3d(QWidget *parent = 0);

    void setValterGroupRotationY(double angle);

    void setLink1ZAngle(double angle);
    void setLink2ZAngle(double angle);
    void setManTiltZAngle(double angle);

signals:

public slots:

private:

};

#endif // VALTER3D_H
