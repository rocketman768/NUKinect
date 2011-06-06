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

#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include "PtCloud.h"


const int viewSizeMin = 10;
const int viewSizeMax = 1000;

//! OpenGL widget in Qt
class GLWidget : public QGLWidget
{
    Q_OBJECT

public:
    GLWidget(QWidget *parent = 0);
    //~GLWidget();

    //! Load the point cloud data from /b src.
    int loadPtCloud(const float* src, int numPts);

    //! Load the point cloud data from point cloud /b cloud.
    int loadPtCloud(const PtCloud & cloud);

    QSize minimumSizeHint() const;
    QSize sizeHint() const;

public slots:
    void setHoriTranslation(int t);
    void setVertTranslation(int t);
    void setXRotation(int angle);
    void setYRotation(int angle);
    void setZRotation(int angle);
    void setViewSize(int s);

signals:
    void xRotationChanged(int angle);
    void yRotationChanged(int angle);
    void zRotationChanged(int angle);
    void viewSizeChanged(int s);

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
    //    void keyPressEvent(QKeyEvent *event);

private:
    PtCloud _cloud;
    int _viewSize;

    //! Horizontal translation, used by horiTranslation/10
    int _horiTranslation;

    //! Vertical translation, used by vertTranslation/10
    int _vertTranslation;

    int xRot;
    int yRot;
    int zRot;
    QPoint lastPos;

     QColor qtPurple;
};


#endif
