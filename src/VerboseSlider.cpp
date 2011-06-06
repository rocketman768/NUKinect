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

#include "VerboseSlider.h"

#include <QGridLayout>
#include <QSlider>
#include <QLabel>
#include <stdio.h>

VerboseSlider::VerboseSlider(QWidget* parent) : QWidget(parent) {
  _slider = new QSlider(Qt::Horizontal, this);
  _slider->setRange(0, 100);
  _slider->setSingleStep(1);
  _slider->setPageStep(10);
  _slider->setTickInterval(10);
  _slider->setTickPosition(QSlider::TicksAbove);

  _minValueLabel = new QLabel(this);
  _maxValueLabel = new QLabel(this);
  _titleLabel = new QLabel(this);
  _currentValueLabel = new QLabel(this);


  connect(_slider, SIGNAL(valueChanged(int)), this, SLOT(setValue(int)));

  QGridLayout * layout = new QGridLayout(this);
  layout->addWidget(_slider, 0, 3, 1, 1);
  layout->addWidget(_titleLabel, 0, 0, 1, 1);
  layout->addWidget(_minValueLabel, 0, 2, 1, 1);
  layout->addWidget(_maxValueLabel, 0, 4, 1, 1);
  layout->addWidget(_currentValueLabel, 0, 1, 1, 1);
  setLayout(layout);

}

int VerboseSlider::setSliderFormat(const QString & title, int minval, int maxval, int singleStep, int pageStep, int tickInterval) {
  
  _slider->setRange(minval, maxval);
  _slider->setSingleStep(singleStep);
  _slider->setPageStep(pageStep);
  _slider->setTickInterval(tickInterval);

  _minValueLabel->setText(QString::number(minval));
  _maxValueLabel->setText(QString::number(maxval));
  _titleLabel->setText(title);
  _currentValueLabel->setText(QString::number(this->value()));

  return 0;
}


void VerboseSlider::setValue(int val) {

  _slider->setValue(val);
  _currentValueLabel->setText(QString::number(val));
}

int VerboseSlider::value() const {
  return _slider->value();
}
