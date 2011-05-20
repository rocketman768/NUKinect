#ifndef LIVE_KINECT_INPUT_WIDGET_H
#define LIVE_KINECT_INPUT_WIDGET_H

#include "KinectInputWidget.h"

//! Realtime Kinect input.
class LiveKinectInputWidget : public KinectInputWidget {
  Q_OBJECT

public:
  LiveKinectInputWidget(QWidget *parent = 0):KinectInputWidget(parent) {}


  /*! Takes \b lastTimestamp as input.
   *  Returns a shared pointer \b ret by reference that points to a COPY of the depth data.
   *  Also, update  \b lastTimestamp by reference. If the internal timestamp
   *  is the same as \b lastTimestamp , \b ret does not change.
   *  The bool return value is false if the data in \b ret may be invalid.
   */
  virtual bool getDepth(uint32_t& lastTimestamp,
		       boost::shared_array<uint8_t>& ret);

  /*! Takes \b lastTimestamp as input.
   *  Returns a shared pointer \b ret by reference that points to a COPY of the depth data.
   *  Also, update  \b lastTimestamp by reference. If the internal timestamp
   *  is the same as \b lastTimestamp , \b ret does not change.
   *  The bool return value is false if the data in \b ret may be invalid.
   */
  virtual bool getRgb(uint32_t& lastTimestamp,
		     boost::shared_array<uint8_t>& ret);
};

#endif // LIVE_KINECT_INPUT_WIDGET_H
