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

#include "FileKinectInputWidget.h"
#include "LiveKinectInputWidget.h"
#include "GLWidget.h"
#include "KinectWindow.h"

static const int imageWidth(640);
static const int imageHeight(480);
static const QSize imageSize(imageWidth, imageHeight);
static const int numImageLabels(2);


static const int numControlSlider(8);


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


int KinectWindow::loadPtCloud(const float* src, int numPts) {
  _mutex.lock();

  int r = glWidget->loadPtCloud(src, numPts);

  this->update();

  _mutex.unlock();
  return r;
}



int KinectWindow::loadPtCloud(const PtCloud & cloud) {
  _mutex.lock();

  int r = glWidget->loadPtCloud(cloud);

  this->update();

  _mutex.unlock();
  return r;
}

int KinectWindow::loadBuffer(const uchar * buffer_ptr, const NUBufferSpec & spec, int buffer_ind) {


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


  QImage img(buffer_ptr, imageWidth, imageHeight, QImage::Format_RGB888);

  _mutex.lock();

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

  _mutex.unlock();

  return 0;
}


KinectWindow::KinectWindow(QWidget* parent) : QWidget(parent) {

  _leftImageLabel = new QLabel(this);
  _leftImageLabel->setMinimumSize(imageSize);
  _rightImageLabel = new QLabel(this);
  _rightImageLabel->setMinimumSize(imageSize);

  glWidget = new GLWidget(this);
  _inputWidget = new FileKinectInputWidget(this);
  
  _kinectInputButton = new QPushButton("Kinect Input", this);
  _fileInputButton = new QPushButton("File Input", this);

  _horiSlider = new QSlider(Qt::Vertical, this);
  _horiSlider->setRange(-100, 100);
  _horiSlider->setSingleStep(1);
  _horiSlider->setPageStep(10);
  _horiSlider->setTickInterval(10);
  _horiSlider->setTickPosition(QSlider::TicksRight);

  _vertSlider = new QSlider(Qt::Vertical, this);
  _vertSlider->setRange(-100, 100);
  _vertSlider->setSingleStep(1);
  _vertSlider->setPageStep(10);
  _vertSlider->setTickInterval(10);
  _vertSlider->setTickPosition(QSlider::TicksRight);


  xSlider = createSlider();
  ySlider = createSlider();
  zSlider = createSlider();
  viewSizeSlider = createViewSizeSlider();

  _controlSlider.resize(this->getNumControlSlider());
  for (int i = 0; i < this->getNumControlSlider(); i++) {
    _controlSlider[i] = new VerboseSlider(this);//createControlSlider();
  }

  
  connect(_horiSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setHoriTranslation(int)));
  connect(_vertSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setVertTranslation(int)));
  connect(xSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setXRotation(int)));
  connect(glWidget, SIGNAL(xRotationChanged(int)), xSlider, SLOT(setValue(int)));
  connect(ySlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setYRotation(int)));
  connect(glWidget, SIGNAL(yRotationChanged(int)), ySlider, SLOT(setValue(int)));
  connect(zSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setZRotation(int)));
  connect(glWidget, SIGNAL(zRotationChanged(int)), zSlider, SLOT(setValue(int)));
  connect(viewSizeSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setViewSize(int)));
  connect(glWidget, SIGNAL(viewSizeChanged(int)), viewSizeSlider, SLOT(setValue(int)));
  connect(_kinectInputButton, SIGNAL( clicked() ), this, SLOT( setKinectInput() ) );
  connect(_fileInputButton, SIGNAL( clicked() ), this, SLOT( setFileInput() ) );

  _mainLayout = new QGridLayout(this);
  _mainLayout->addWidget(_leftImageLabel, 0, 0, 1, 6  );
  _mainLayout->addWidget(_rightImageLabel, 0, 10, 1, 4  );
  _mainLayout->addWidget(glWidget, 1, 0, this->getNumControlSlider(), 1);
  _mainLayout->addWidget(_horiSlider, 1, 1, this->getNumControlSlider(), 1);
  _mainLayout->addWidget(_vertSlider, 1, 2, this->getNumControlSlider(), 1);
  _mainLayout->addWidget(xSlider, 1, 3, this->getNumControlSlider(), 1);
  _mainLayout->addWidget(ySlider, 1, 4, this->getNumControlSlider(), 1);
  _mainLayout->addWidget(zSlider, 1, 5, this->getNumControlSlider(), 1);
  _mainLayout->addWidget(viewSizeSlider, 1, 6,this->getNumControlSlider(), 1);

  _mainLayout->addWidget(_kinectInputButton, 1, 11, 1, 1);
  _mainLayout->addWidget(_fileInputButton, 2, 11, 1, 1);
  _mainLayout->addWidget(_inputWidget, 3, 11, 1, 1);

  for (int i = 0; i < this->getNumControlSlider(); i++) {
    _mainLayout->addWidget( _controlSlider[i], i+1, 10, 1, 1);
  }

  _mainLayout->setSizeConstraint(QLayout::SetFixedSize);
  setLayout(_mainLayout);

  xSlider->setValue(15 * 16);
  ySlider->setValue(345 * 16);
  zSlider->setValue(0 * 16);
  viewSizeSlider->setValue(5 * 100);
  setWindowTitle(tr("Depth Vision - Computational Vision Group, Northwestern Univeristy"));


  QImage blackImg(imageWidth, imageHeight, QImage::Format_RGB888);
  blackImg.fill(0);

  _mutex.lock();

  _leftImage = blackImg;
  _rightImage = blackImg;

  _mutex.unlock();

  _leftUpdate = false;
  _rightUpdate = false;

}

int KinectWindow::getNumControlSlider() const {
  return numControlSlider;
}

int KinectWindow::setControlSliderFormat(int ind, const QString & title, int minval, int maxval, int singleStep, int pageStep, int tickInterval) {

  if (ind < 0 || ind >= this->getNumControlSlider()) {
    printf("Error: The ind in setControlSliderFormat is out of range.\n");
    return 1;
  }

  return _controlSlider[ind]->setSliderFormat(title, minval, maxval, singleStep, pageStep, tickInterval);
  
}

bool KinectWindow::getDepth(uint32_t& lastTimestamp,
			    boost::shared_array<uint8_t>& ret) {
  _mutexInputWidget.lock();
  bool r = _inputWidget->getDepth(lastTimestamp, ret);
  _mutexInputWidget.unlock();

  return r;
}


bool KinectWindow::getRgb(uint32_t& lastTimestamp,
			  boost::shared_array<uint8_t>& ret) {
  _mutexInputWidget.lock();
  bool r = _inputWidget->getRgb(lastTimestamp, ret);
  _mutexInputWidget.unlock();

  return r;
}

QSlider *KinectWindow::createSlider()
{
  QSlider *slider = new QSlider(Qt::Vertical, this);
  slider->setRange(0, 360 * 16);
  slider->setSingleStep(16);
  slider->setPageStep(15 * 16);
  slider->setTickInterval(15 * 16);
  slider->setTickPosition(QSlider::TicksRight);
  return slider;
}

QSlider *KinectWindow::createViewSizeSlider() {
  QSlider *slider = new QSlider(Qt::Vertical, this);
  slider->setRange(viewSizeMin, viewSizeMax);
  slider->setSingleStep(10);
  slider->setPageStep(10 * 10);
  slider->setTickInterval(10 * 10);
  slider->setTickPosition(QSlider::TicksRight);
  return slider;
}

int KinectWindow::setControlSliderValue(int ind, int val) {
  if (ind < 0 || ind >= this->getNumControlSlider()) {
    printf("Error: The ind in setControlSliderValue is out of range.\n");
    return 1;
  }

  _controlSlider[ind]->setValue(val);

  return 0;
}

int KinectWindow::getControlSliderValue(const int ind, int & val) const{ 

  if (ind < 0 || ind >= this->getNumControlSlider()) {
    printf("Error: The ind in getControlSliderValue is out of range.\n");
    return 1;
  }

  val = _controlSlider[ind]->value();

  return 0;
  
}



void KinectWindow::keyPressEvent(QKeyEvent *e)
{
  //_mutex.lock();

  if (e->key() == Qt::Key_Escape)
    close();
  else
    QWidget::keyPressEvent(e);
  //_mutex.unlock();
}

void KinectWindow::paintEvent(QPaintEvent *e) {
  _mutex.lock();

  if (_leftUpdate) {
    _leftImageLabel->setPixmap(QPixmap::fromImage(_leftImage));
    _leftUpdate = false;
  }

  if (_rightUpdate) {
    _rightImageLabel->setPixmap(QPixmap::fromImage(_rightImage));
    _rightUpdate = false;
  }

  QWidget::paintEvent(e);

  _mutex.unlock();
}

void KinectWindow::setKinectInput() {

  _mutexInputWidget.lock();

  if (_inputWidget) {
    delete _inputWidget;
    _inputWidget = NULL;
  }

  _inputWidget = new LiveKinectInputWidget(this);
  
  _mutexInputWidget.unlock();
}


void KinectWindow::setFileInput() {

  _mutexInputWidget.lock();

  if (_inputWidget) {
    delete _inputWidget;
    _inputWidget = NULL;
  }

  _inputWidget = new FileKinectInputWidget(this);
  _mainLayout->addWidget(_inputWidget, 3, 11, 1, 1);
  _mainLayout->setSizeConstraint(QLayout::SetFixedSize);
  setLayout(_mainLayout);

  _mutexInputWidget.unlock();

  static_cast<FileKinectInputWidget*>(_inputWidget)->openFile();
}
