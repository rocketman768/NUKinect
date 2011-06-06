/*
* This file is part of NUKinect.
* 
* Copyright 2011 by the Authors:
* Jiang Wang, <wangjiangb@gmail.com>
* Jiang Xu, <jiangxu2011@u.northwestern.edu>
* Philip G. Lee, <rocketman768@gmail.com>
* 
* NUKinect is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* NUKinect is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
* 
* You should have received a copy of the GNU Lesser General Public License
* along with NUKinect.  If not, see <http://www.gnu.org/licenses/>.
*/

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
