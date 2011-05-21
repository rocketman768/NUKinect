#include "LiveKinectInputWidget.h"

#include "KinectIO.h"
#include <QtGui>

bool LiveKinectInputWidget::getDepth(uint32_t& lastTimestamp, boost::shared_array<uint8_t>& ret) {

  return KinectIO::instance().kinect().getDepth(lastTimestamp, ret);

}


bool LiveKinectInputWidget::getRgb(uint32_t& lastTimestamp, boost::shared_array<uint8_t>& ret) {

  return false;
}
