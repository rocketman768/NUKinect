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
