/****************************************************************************
**
** Copyright (C) 2010 Nokia Corporation and/or its subsidiary(-ies).
** All rights reserved.
** Contact: Nokia Corporation (qt-info@nokia.com)
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:LGPL$
** Commercial Usage
** Licensees holding valid Qt Commercial licenses may use this file in
** accordance with the Qt Commercial License Agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and Nokia.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 2.1 as published by the Free Software
** Foundation and appearing in the file LICENSE.LGPL included in the
** packaging of this file.  Please review the following information to
** ensure the GNU Lesser General Public License version 2.1 requirements
** will be met: http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
**
** In addition, as a special exception, Nokia gives you certain additional
** rights.  These rights are described in the Nokia Qt LGPL Exception
** version 1.1, included in the file LGPL_EXCEPTION.txt in this package.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License version 3.0 as published by the Free Software
** Foundation and appearing in the file LICENSE.GPL included in the
** packaging of this file.  Please review the following information to
** ensure the GNU General Public License version 3.0 requirements will be
** met: http://www.gnu.org/copyleft/gpl.html.
**
** If you have questions regarding the use of this file, please contact
** Nokia at qt-info@nokia.com.
** $QT_END_LICENSE$
**
****************************************************************************/

#ifndef KINECT_WINDOW_H
#define KINECT_WINDOW_H


#include "Mutex.h"

#include <QGLWidget>
#include <QWidget>
#include <QApplication>

QT_BEGIN_NAMESPACE
class QSlider;
class QLabel;
QT_END_NAMESPACE
//! [0]
class GLWidget;

class NUBufferSpec {
public:
  enum BufferMode {
    RGB,
    LUMINANCE
  };

  NUBufferSpec(BufferMode m) { mode = m; }
  BufferMode getMode() const { return mode; }

protected:
  BufferMode mode;

};

// Singleton
class KinectWindow : public QWidget {
    Q_OBJECT

public:
    static KinectWindow & instance();
    int destroyInstance();

    //int startDrawLoop();
    int loadPtCloud(const GLfloat* src, int numPts);
    int setBuffer(const uchar * buffer_ptr, const NUBufferSpec & spec, int buffer_ind);

protected:
    void keyPressEvent(QKeyEvent *event);
    void paintEvent(QPaintEvent *event);

    KinectWindow(QWidget *parent=0);

    Mutex _mutex;

private:
    QSlider *createSlider();
    QSlider *createViewSizeSlider();

    QLabel *_leftImageLabel;
    QLabel *_rightImageLabel;

    QImage _leftImage;
    QImage _rightImage;

    QPixmap _leftPixmap;
    QPixmap _rightPixmap;

    bool _leftUpdate;
    bool _rightUpdate;

    GLWidget *glWidget;
    QSlider *xSlider;
    QSlider *ySlider;
    QSlider *zSlider;
    QSlider *viewSizeSlider;

    static KinectWindow * _pInstance;
    //static QApplication * _pQapp;
};
//! [0]

#endif
