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

#ifndef KINECT_WINDOW_H
#define KINECT_WINDOW_H

#include "VerboseSlider.h"
#include "KinectInputWidget.h"
#include "Mutex.h"
#include "PtCloud.h"
#include <QGLWidget>
#include <QWidget>
#include <QApplication>
#include <vector>

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
  int loadPtCloud(const float* src, int numPts);

  //! Load the point cloud from PtCloud \b cloud.
  int loadPtCloud(const PtCloud & cloud);

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

  //! Get the number of control sliders.
  int getNumControlSlider() const;

  /*! Set the format of one control slider. 
   *  the index of the control slider is determined by \b ind, 
   *  the minimal and maximal values of the control slider are 
   *  \b minval and \b maxval, the steps are determined by
   *  \b singleStep and \b pageStep, and the tick interval is 
   *  determined by \b tickInterval.
   */
  int setControlSliderFormat(int ind, const QString & title, int minval = 0, int maxval = 100, int singleStep = 1, int pageStep = 10, int tickInterval = 10);

  //! Set the value of the control slider with index \b ind
  int setControlSliderValue(int ind, int val);

  //! Get the value \b val of the control slider with index \b ind
  int getControlSliderValue(const int ind, int & val) const;


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
  QSlider *createControlSlider();

  QLabel *_leftImageLabel;
  QLabel *_rightImageLabel;

  QImage _leftImage;
  QImage _rightImage;

  QPixmap _leftPixmap;
  QPixmap _rightPixmap;

  bool _leftUpdate;
  bool _rightUpdate;

  QSlider *_horiSlider;
  QSlider *_vertSlider;
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
  
  std::vector <VerboseSlider *> _controlSlider;
};

#endif
