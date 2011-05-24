#ifndef KINECT_INPUT_WIDGET_H
#define KINECT_INPUT_WIDGET_H

#include <boost/shared_array.hpp>

#include <QWidget>
#include <inttypes.h>

//! The input interface for both realtime (\b LiveKinectInputWidget) and file (\b FileKinectInputWidget).
class KinectInputWidget : public QWidget {
  Q_OBJECT

public:
  KinectInputWidget(QWidget *parent = 0):QWidget(parent) {}
  
  /*! Takes \b lastTimestamp as input.
   *  Returns a shared pointer \b ret by reference that points to a COPY of the depth data.
   *  Also, update  \b lastTimestamp by reference. If the internal timestamp
   *  is the same as \b lastTimestamp , \b ret does not change.
   *  The bool return value is false if the data in \b ret may be invalid.
   */
  virtual bool getDepth(uint32_t& lastTimestamp,
		       boost::shared_array<uint8_t>& ret) = 0;

  /*! Takes \b lastTimestamp as input.
   *  Returns a shared pointer \b ret by reference that points to a COPY of the depth data.
   *  Also, update  \b lastTimestamp by reference. If the internal timestamp
   *  is the same as \b lastTimestamp , \b ret does not change.
   *  The bool return value is false if the data in \b ret may be invalid.
   */
  virtual bool getRgb(uint32_t& lastTimestamp,
		     boost::shared_array<uint8_t>& ret) = 0;

};

#endif // KINECT_INPUT_WIDGET_H
