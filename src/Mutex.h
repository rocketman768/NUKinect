#ifndef _MUTEX_H
#define _MUTEX_H

#include <pthread.h>

class Mutex
{
public:
    Mutex()
    {
      pthread_mutex_init( &_mutex, 0 );
    }
    
    ~Mutex()
    {
    }
    
    void lock()
    {
      pthread_mutex_lock( &_mutex );
    }
    
    void unlock()
    {
      pthread_mutex_unlock( &_mutex );
    }
    
private:
  pthread_mutex_t _mutex;
};

#endif /* _MUTEX_H */
