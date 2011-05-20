#ifndef FILE_KINECT_INPUT_WIDGET_H
#define FILE_KINECT_INPUT_WIDGET_H

#include "KinectInputWidget.h"

#include "Mutex.h"
#include "DepthMap.h"
#include "stdio.h"


QT_BEGIN_NAMESPACE
class QPushButton;
QT_END_NAMESPACE

class FileKinectInputWidget : public KinectInputWidget {
  Q_OBJECT

    public:
  FileKinectInputWidget(QWidget *parent = 0);
  
  virtual bool getDepth(uint32_t& lastTimestamp,
		       boost::shared_array<uint8_t>& ret);
  virtual bool getRgb(uint32_t& lastTimestamp,
		     boost::shared_array<uint8_t>& ret);

  public slots:
  void openFile();
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
