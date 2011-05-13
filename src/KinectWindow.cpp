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

#include <QtGui>

#include "GLWidget.h"
#include "KinectWindow.h"

static const int imageWidth(640);
static const int imageHeight(480);
static const QSize imageSize(imageWidth, imageHeight);
static const int numImageLabels(2);

KinectWindow * KinectWindow::_pInstance(NULL);
//QApplication * KinectWindow::_pQapp(NULL);

KinectWindow & KinectWindow::instance() {
  if (_pInstance == NULL) {
    //static QApplication app(argc, argv);
    //_pQapp = &app;
    _pInstance = new KinectWindow;
    _pInstance->show();
  }

  return *_pInstance;
}

int KinectWindow::destroyInstance() {
  if (_pInstance != NULL) {
    delete _pInstance;
    _pInstance = NULL;
  }

  return 1;
}

int KinectWindow::setBuffer(const uchar * buffer_ptr, const NUBufferSpec & spec, int buffer_ind) {

  if (buffer_ind >= numImageLabels) {
    return 1;
  }


  // RGB case only now
  switch (spec.getMode()) {

  case NUBufferSpec::RGB:

    break;

  case NUBufferSpec::LUMINANCE:

    break;

  default:
    ;

  } // switch spec.getMode()

  _mutex.lock();
  QImage img(buffer_ptr, imageWidth, imageHeight, QImage::Format_RGB888);
  _mutex.unlock();

  switch (buffer_ind) {
  case 0:
    _leftImage = img;
    _leftUpdate = true;
    break;
  case 1:
    _rightImage = img;
    _rightUpdate = true;
    break;

  default:
    ;
  } // switch buffer_ind

  this->update();
  //  this->repaint();
  return 0;
}

//int KinectWindow::startDrawLoop() {
//    if (_pInstance == NULL) {
//        return 1;
//    }

//    this->show();
//    //_pQapp->exec();
//    return 0;
//}


KinectWindow::KinectWindow(){

  //  _mutex.lock();

  _leftImageLabel = new QLabel;
  _leftImageLabel->setMinimumSize(imageSize);
  _rightImageLabel = new QLabel;
  _rightImageLabel->setMinimumSize(imageSize);

  glWidget = new GLWidget;

  xSlider = createSlider();
  ySlider = createSlider();
  zSlider = createSlider();

  connect(xSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setXRotation(int)));
  connect(glWidget, SIGNAL(xRotationChanged(int)), xSlider, SLOT(setValue(int)));
  connect(ySlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setYRotation(int)));
  connect(glWidget, SIGNAL(yRotationChanged(int)), ySlider, SLOT(setValue(int)));
  connect(zSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setZRotation(int)));
  connect(glWidget, SIGNAL(zRotationChanged(int)), zSlider, SLOT(setValue(int)));


  QGridLayout *mainLayout = new QGridLayout;
  mainLayout->addWidget(_leftImageLabel, 0, 0, 1, 1  );
  mainLayout->addWidget(_rightImageLabel, 0, 1, 1, 6  );
  mainLayout->addWidget(glWidget, 1, 0, 1, 1);
  mainLayout->addWidget(xSlider, 1, 1, 1, 1);
  mainLayout->addWidget(ySlider, 1, 2, 1, 1);
  mainLayout->addWidget(zSlider, 1, 3, 1, 1);
  mainLayout->setSizeConstraint(QLayout::SetFixedSize);
  setLayout(mainLayout);

  xSlider->setValue(15 * 16);
  ySlider->setValue(345 * 16);
  zSlider->setValue(0 * 16);
  setWindowTitle(tr("Depth Vision - Computational Vision Group, Northwestern Univeristy"));


  QImage blackImg(imageWidth, imageHeight, QImage::Format_RGB888);
  blackImg.fill(0);

  _leftImage = blackImg;
  _rightImage = blackImg;


  _leftUpdate = false;
  _rightUpdate = false;

  //  _mutex.unlock();

}

QSlider *KinectWindow::createSlider()
{
  QSlider *slider = new QSlider(Qt::Vertical);
  slider->setRange(0, 360 * 16);
  slider->setSingleStep(16);
  slider->setPageStep(15 * 16);
  slider->setTickInterval(15 * 16);
  slider->setTickPosition(QSlider::TicksRight);
  return slider;
}

void KinectWindow::keyPressEvent(QKeyEvent *e)
{
  // _mutex.lock();

  if (e->key() == Qt::Key_Escape)
    close();
  else
    QWidget::keyPressEvent(e);
  // _mutex.unlock();
}

void KinectWindow::paintEvent(QPaintEvent *e)
{
  //_mutex.lock();

  if (_leftUpdate) {
    _leftImageLabel->setPixmap(QPixmap::fromImage(_leftImage));
    _leftUpdate = false;
  }

  if (_rightUpdate) {
    _rightImageLabel->setPixmap(QPixmap::fromImage(_rightImage));
    _rightUpdate = false;
  }

  QWidget::paintEvent(e);

  //_mutex.unlock();
}
