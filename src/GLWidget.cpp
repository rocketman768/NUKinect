/*
* This file is part of NUKinect.
* 
* Copyright 2011 by the Authors:
* Jiang Wang, <wangjiangb@gmail.com>
* Jiang Xu, <jiangxu2011@u.northwestern.edu>
* Philip G. Lee, <rocketman768@gmail.com>
* 
* NUKinect is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* NUKinect is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
* 
* You should have received a copy of the GNU Lesser General Public License
* along with NUKinect.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <QtGui>
#include <QtOpenGL>

#include <math.h>

#include "GLWidget.h"
#include "PtCloud.h"

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif

GLWidget::GLWidget(QWidget *parent)
  : QGLWidget(QGLFormat(QGL::SampleBuffers), parent)
{
  _viewSize = 400;
  xRot = 0;
  yRot = 0;
  zRot = 0;
  _horiTranslation = 0;
  _vertTranslation = 0;

  //qtGreen = QColor::fromCmykF(0.40, 0.0, 1.0, 0.0);
  qtPurple = QColor::fromCmykF(0.39, 0.39, 0.0, 0.0);
}

int GLWidget::loadPtCloud(const float* src, int numPts) {

  return _cloud.setPoints(src, numPts);
}

int GLWidget::loadPtCloud(const PtCloud & cloud_in) {

  return _cloud.loadPtCloud(cloud_in);
}

QSize GLWidget::minimumSizeHint() const
{
  return QSize(500, 400);
}

QSize GLWidget::sizeHint() const
{
  return QSize(500, 400);
}

static void qNormalizeAngle(int &angle)
{
  while (angle < 0)
    angle += 360 * 16;
  while (angle > 360 * 16)
    angle -= 360 * 16;
}

void GLWidget::setHoriTranslation(int t) {
  if (_horiTranslation != t) {
    _horiTranslation = t;
    updateGL();
  }
}

void GLWidget::setVertTranslation(int t) {
  if (_vertTranslation != t) {
    _vertTranslation = t;
    updateGL();
  }
}

void GLWidget::setViewSize(int s) {
  if (s != _viewSize && s <= viewSizeMax && s >= viewSizeMin) {
    _viewSize = s;
    emit viewSizeChanged(s);
    updateGL();
  }
}

void GLWidget::setXRotation(int angle)
{
  qNormalizeAngle(angle);
  if (angle != xRot) {
    xRot = angle;
    emit xRotationChanged(angle);
    updateGL();
  }
}

void GLWidget::setYRotation(int angle)
{
  qNormalizeAngle(angle);
  if (angle != yRot) {
    yRot = angle;
    emit yRotationChanged(angle);
    updateGL();
  }
}

void GLWidget::setZRotation(int angle)
{
  qNormalizeAngle(angle);
  if (angle != zRot) {
    zRot = angle;
    emit zRotationChanged(angle);
    updateGL();
  }
}

void GLWidget::initializeGL()
{
  qglClearColor(qtPurple.dark());


  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  glEnable(GL_MULTISAMPLE);
}

void GLWidget::paintGL()
{

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
#ifdef QT_OPENGL_ES_1
  glOrthof(-_viewSize/100., _viewSize/100., -_viewSize/100., _viewSize/100., -1.0, 20.0);
#else
  glOrtho(-_viewSize/100., _viewSize/100., -_viewSize/100., _viewSize/100., -1.0, 20.0);
#endif
  glMatrixMode(GL_MODELVIEW);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();
  glTranslatef(-_horiTranslation/10.0, -_vertTranslation/10.0, -10.0);
  glRotatef(xRot / 16.0, 1.0, 0.0, 0.0);
  glRotatef(yRot / 16.0, 0.0, 1.0, 0.0);
  glRotatef(zRot / 16.0, 0.0, 0.0, 1.0);
  _cloud.draw();
}

void GLWidget::resizeGL(int width, int height)
{
  int side = qMin(width, height);
  glViewport((width - side) / 2, (height - side) / 2, side, side);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
#ifdef QT_OPENGL_ES_1
  glOrthof(-_viewSize/100., _viewSize/100., -_viewSize/100., _viewSize/100., -1.0, 20.0);
#else
  glOrtho(-_viewSize/100., _viewSize/100., -_viewSize/100., _viewSize/100., -1.0, 20.0);
#endif
  glMatrixMode(GL_MODELVIEW);
}

void GLWidget::wheelEvent(QWheelEvent *event) {
  setViewSize(_viewSize + event->delta());
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
  lastPos = event->pos();
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
  int dx = event->x() - lastPos.x();
  int dy = event->y() - lastPos.y();

  if (event->buttons() & Qt::LeftButton) {
    setXRotation(xRot + 8 * dy);
    setYRotation(yRot + 8 * dx);
  } else if (event->buttons() & Qt::RightButton) {
    setXRotation(xRot + 8 * dy);
    setZRotation(zRot + 8 * dx);
  }
  lastPos = event->pos();
}

/*
void GLWidget::keyPressEvent(QKeyEvent *e) {

  switch(e->key()) {

  case Qt::Key_Left:
    printf("L");
    break;
  case Qt::Key_Right:

    printf("R");

    break;
  case Qt::Key_Down:

    printf("D");
    break;
  case Qt::Key_Up:

    printf("U");
    break;
  case Qt::Key_Space:

    printf("S");
    break;

  default:
    QWidget::keyPressEvent(e);
  }

}
*/
