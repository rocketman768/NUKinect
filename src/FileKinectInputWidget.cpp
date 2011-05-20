#include "FileKinectInputWidget.h"

#include <libfreenect.hpp>
#include "DepthMapBinFileIO.h"

#include <QtGui>

FileKinectInputWidget:: FileKinectInputWidget(QWidget *parent):KinectInputWidget(parent){

  _filePath = NULL;
  _fp = NULL;
  _depth = NULL;

  _openVideoFileButton = new QPushButton("Open File", this);
  _nextFrameButton = new QPushButton("Next Frame", this);

  connect(_openVideoFileButton, SIGNAL( clicked() ), this, SLOT( openFile() ) );
 connect(_nextFrameButton, SIGNAL( clicked() ), this, SLOT( nextFrame() ) );

  QGridLayout *mainLayout = new QGridLayout(this);
  mainLayout->addWidget(_openVideoFileButton, 0, 0, 1, 1);
  mainLayout->addWidget(_nextFrameButton, 0, 1, 1, 1);
  setLayout(mainLayout);

}

// Caution: mutex needed when call clear()
int FileKinectInputWidget::clear() {

  if (_fp) {
    fclose(_fp);
    _fp = NULL;
  }

  if (_depth) {
    delete [] _depth;
    _depth = NULL;
  }
  
  return 0;
}

bool FileKinectInputWidget::getDepth(uint32_t& lastTimestamp, boost::shared_array<uint8_t>& ret) {
  // Lock b/c we need to access _depth and _depthTimestamp.
  _mutex.lock();
    
  // Don't change 'ret' if it is the same frame.
  if( lastTimestamp == _frameIndex || _depth == 0 )
  {
    _mutex.unlock();
    return (_depth!=0);
  }
  
  // Otherwise, change the last timestamp.
  lastTimestamp = _frameIndex;
  
  // Return a shared array that points to a COPY
  // of our _depth data. Since it is shared, it will
  // automatically de-allocate when no more references
  // to the memory location exist.
  uint8_t* doNotCopy = new uint8_t[FREENECT_DEPTH_11BIT_SIZE];
  ret = boost::shared_array<uint8_t>(
          reinterpret_cast<uint8_t*>(
            memcpy(doNotCopy, _depth, FREENECT_DEPTH_11BIT_SIZE * sizeof(uint8_t))
          )
        );
  doNotCopy = 0; // Protect from stupidity.
    
  // Release lock.
  _mutex.unlock();
  return true;  

}


bool FileKinectInputWidget::getRgb(uint32_t& lastTimestamp, boost::shared_array<uint8_t>& ret) {

  return false;
}

void FileKinectInputWidget::openFile() {

    
  QString path = QFileDialog::getOpenFileName(
					      this,
					      "Choose a file to open",
					      QString::null,
					      QString::null);

  if (path.isEmpty()) {
    return;
  }

  QByteArray ba = path.toLocal8Bit();


  _mutex.lock();

  _filePath = ba.data();

  this->clear();
  _fp = fopen(_filePath, "rb");

  if (_fp == NULL) {
    printf("Cannot open %s\n", _filePath);
    _mutex.unlock();
    return;
  }



  ReadDepthMapBinFileHeader(_fp, _nFrames, _nCols, _nRows);
  printf("%d, %d, %d\n", _nFrames, _nCols, _nRows);

  _depthMap.SetSize(_nCols, _nRows);
  ReadDepthMapBinFileNextFrame(_fp, _nCols, _nRows, _depthMap);




  _frameIndex = 0;

  _depth = new uint8_t[_nCols*_nRows];
  _depthMap.convertToChar(_depth);

  _mutex.unlock();
}


void FileKinectInputWidget::nextFrame() {


  _mutex.lock();


  if (_fp == NULL) {
    printf("No file loaded\n");
    _mutex.unlock();
    return;
  }

  if (_frameIndex == static_cast<uint32_t>( _nFrames)-1 ) {
    printf("frame index (start from 0): %d, # of frames: %d: already reach the end\n", _frameIndex, _nFrames);
    _mutex.unlock();
    return;
  }


  ReadDepthMapBinFileNextFrame(_fp, _nCols, _nRows, _depthMap);
  _depthMap.convertToChar(_depth);

  _frameIndex++;

  _mutex.unlock();
}
