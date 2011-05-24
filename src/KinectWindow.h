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

#include "KinectInputWidget.h"
#include "Mutex.h"

#include <QGLWidget>
#include <QWidget>
#include <QApplication>

QT_BEGIN_NAMESPACE
class QSlider;
class QLabel;
class QPushButton;
class QGridLayout;
QT_END_NAMESPACE

class GLWidget;

//! The specifications of a 2D image buffer.
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

//! \b KinectWindow is a Singleton class
class KinectWindow : public QWidget {
  Q_OBJECT

    public:
  
  //! Obtain the singleton instance of /b KinectWindow.
  static KinectWindow & instance();

  //! Destroy the singleton instance of /b KinectWindow.
  int destroyInstance();

  //int startDrawLoop();

  //! Load the point cloud from \b src.
  int loadPtCloud(const GLfloat* src, int numPts);

  //! Load 2D image from \b buffer_ptr.
  int loadBuffer(const uchar * buffer_ptr, const NUBufferSpec & spec, int buffer_ind);
    

  /*! Takes \b lastTimestamp as input.
   *  Returns a shared pointer \b ret by reference that points to a COPY of the depth data.
   *  Also, update  \b lastTimestamp by reference. If the internal timestamp
   *  is the same as \b lastTimestamp , \b ret does not change.
   *  The bool return value is false if the data in \b ret may be invalid.
   */
  bool getDepth(uint32_t& lastTimestamp,
		boost::shared_array<uint8_t>& ret);

  /*! Takes \b lastTimestamp as input.
   *  Returns a shared pointer \b ret by reference that points to a COPY of the RGB data.
   *  Also, update  \b lastTimestamp by reference. If the internal timestamp
   *  is the same as \b lastTimestamp , \b ret does not change.
   *  The bool return value is false if the data in \b ret may be invalid.
   */
  bool getRgb(uint32_t& lastTimestamp,
	      boost::shared_array<uint8_t>& ret);


  public slots:
  void setKinectInput();
  void setFileInput();

  protected:
  void keyPressEvent(QKeyEvent *event);
  void paintEvent(QPaintEvent *event);

  KinectWindow(QWidget *parent=0);

  Mutex _mutex;
  Mutex _mutexInputWidget;

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
  KinectInputWidget *_inputWidget;

  QPushButton *_kinectInputButton;
  QPushButton *_fileInputButton;

  QGridLayout *_mainLayout;

  static KinectWindow * _pInstance;
  //static QApplication * _pQapp;
};

#endif
