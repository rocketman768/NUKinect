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

#ifndef VERBOSE_SLIDER_H
#define VERBOSE_SLIDER_H


#include <QWidget>

QT_BEGIN_NAMESPACE
class QSlider;
class QLabel;
QT_END_NAMESPACE

//! \b VerboseSlider looks like QSlider, but has some display.
class VerboseSlider : public QWidget {
  Q_OBJECT

    public:

  VerboseSlider(QWidget *parent=0);
 
  //! get the value of the slider
  int value() const;

  //! set the format of the slider
  int setSliderFormat(const QString & title, int minval, int maxval, int singleStep, int pageStep, int tickInterval);
 
  public slots:

  //! set the value of the slider to \b val
  void setValue(int val);

 private:

  QSlider * _slider;
  QLabel * _titleLabel;
  QLabel * _minValueLabel;
  QLabel * _maxValueLabel;
  QLabel * _currentValueLabel;

};



#endif // VERBOSE_SLIDER_H
