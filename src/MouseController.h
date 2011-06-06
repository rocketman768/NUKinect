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

#ifndef _MOUSE_CONTROLLER_H
#define _MOUSE_CONTROLLER_H
#include <X11/X.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include<stdio.h>
#include<string.h>
//! class for controlling the mouse
class MouseController
{
public:
	static MouseController & instance();

	void init();
    //! get the position of the mouse, output \b x is the x coordinate, and output \b y is the y coordinate.
	void GetCursorPos(int &x,int &y);
    //! set the position of the mouse, \input \b x is the x coordinate, and output \b y is the y coordinate
	void SetCursorPos(int x,int y);
    //! press a button, if \b button is 1, press the left button. If \b button is 2 press the right button.
	void mousePress(int button);
    //!release a button, if \b button is 1, release the left button. If \b button is 2  release  the  right button.
	void mouseRelease(int button);

protected:
	MouseController();
	~MouseController();
private:
	Display* _Pdisplay;
	Window _root;
	static MouseController* _pInstance;
};

#endif
