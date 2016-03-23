#ifndef ACCELEROMETERRENDERSCENE_H
#define ACCELEROMETERRENDERSCENE_H

#include <QGLWidget>

class AccelerometerRenderScene : public QGLWidget
{
public:
    AccelerometerRenderScene(QWidget *parent = 0);
    ~AccelerometerRenderScene();

    // QOpenGLWidget interface
protected:
    void initializeGL();
    void paintGL();
};

#endif // ACCELEROMETERRENDERSCENE_H
