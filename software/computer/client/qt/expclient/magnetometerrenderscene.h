#ifndef MAGNETOMETERRENDERSCENE_H
#define MAGNETOMETERRENDERSCENE_H

#include <QGLWidget>

class MagnetometerRenderScene : public QGLWidget
{
public:
    MagnetometerRenderScene(QWidget *parent = 0);
    ~MagnetometerRenderScene();

    // QOpenGLWidget interface
protected:
    void initializeGL();
    void paintGL();
};

#endif // MAGNETOMETERRENDERSCENE_H
