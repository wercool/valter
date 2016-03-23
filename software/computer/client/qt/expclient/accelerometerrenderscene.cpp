#include "accelerometerrenderscene.h"

AccelerometerRenderScene::AccelerometerRenderScene(QWidget *parent): QGLWidget(parent)
{

}

AccelerometerRenderScene::~AccelerometerRenderScene()
{

}


void AccelerometerRenderScene::initializeGL()
{
    glClearColor(0,0,0,1);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHTING);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
}

void AccelerometerRenderScene::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glBegin(GL_LINE_STRIP);
    glVertex2f(0, 0);
    for (float x = 1; x < 100; x += 1)
    {
      glVertex2f(x / 100, rand() % 2);
    }
    glVertex2f(100, 0);
    glEnd();

    update();
}
