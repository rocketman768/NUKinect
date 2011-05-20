#ifndef KINECT_INPUT_WIDGET_H
#define KINECT_INPUT_WIDGET_H

#include <boost/shared_array.hpp>

#include <QWidget>
#include <inttypes.h>

class KinectInputWidget : public QWidget {
  Q_OBJECT

public:
  KinectInputWidget(QWidget *parent = 0):QWidget(parent) {}
  
  virtual bool getDepth(uint32_t& lastTimestamp,
		       boost::shared_array<uint8_t>& ret) = 0;
  virtual bool getRgb(uint32_t& lastTimestamp,
		     boost::shared_array<uint8_t>& ret) = 0;

};

#endif // KINECT_INPUT_WIDGET_H
