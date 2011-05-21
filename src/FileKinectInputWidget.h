#ifndef FILE_KINECT_INPUT_WIDGET_H
#define FILE_KINECT_INPUT_WIDGET_H

#include "KinectInputWidget.h"

#include "Mutex.h"
#include "DepthMap.h"
#include "stdio.h"


QT_BEGIN_NAMESPACE
class QPushButton;
QT_END_NAMESPACE

//! File input.
class FileKinectInputWidget : public KinectInputWidget {
  Q_OBJECT

    public:
  FileKinectInputWidget(QWidget *parent = 0);
  
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

  public slots:

  //! Open a video file by a dialog.
  void openFile();

  //! Go to the next frame.
  void nextFrame();
 protected:
  int clear();

 private: 

  QPushButton *_openVideoFileButton;
  QPushButton *_nextFrameButton;

  char* _filePath;
  FILE* _fp;

  int _nFrames;
  int _nCols;
  int _nRows;

  uint32_t _frameIndex;
  CDepthMap  _depthMap;
  uint8_t * _depth;
  uint8_t * _rgb;

  Mutex _mutex;
  
};

#endif // FILE_KINECT_INPUT_WIDGET_H
