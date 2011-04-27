#include "Kinect.h"
#include "Mutex.h"

/*! \b KinectIO is a singleton class.
 */
class KinectIO
{
public:
  ~KinectIO();
  
  //! Get the \b KinectIO instance.
  static KinectIO& instance();
  
  //! Get the \b Kinect variable.
  Kinect& kinect();
  
protected:
  //! Hidden constructor.
  KinectIO();
  //! Hidden copy constructor.
  KinectIO( const KinectIO& other );
  //! Hidden assignment operator.
  KinectIO& operator = ( KinectIO const& other );
  
private:
  
  static KinectIO* _instance;
  Kinect _kinect;
  Mutex _instanceMutex;
};