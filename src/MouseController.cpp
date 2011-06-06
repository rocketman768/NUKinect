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

#include "MouseController.h"
MouseController* MouseController::_pInstance = NULL;

MouseController::MouseController()
{
	init();
}

MouseController::~MouseController()
{
	XCloseDisplay(_Pdisplay);
}

void MouseController::init()
{
	if ((_Pdisplay = XOpenDisplay(NULL)) == NULL) {
		fprintf(stderr, "Cannot open local X-display.\n");
		return;
	}
	_root = DefaultRootWindow(_Pdisplay);
}

void MouseController::GetCursorPos( int &x,int &y )
{
	int tmp;unsigned int tmp2;
	Window fromroot, tmpwin;
	XQueryPointer(_Pdisplay, _root, &fromroot, &tmpwin, &x, &y, &tmp, &tmp, &tmp2);
}

void MouseController::SetCursorPos( int x,int y )
{
	XWarpPointer(_Pdisplay, None, _root, 0, 0, 0, 0, x, y);
	XFlush(_Pdisplay);
}

void MouseController::mousePress( int button )
{
	Display *display = XOpenDisplay(NULL);
	if(display == NULL)
	{
		printf("Errore nell'apertura del Display !!!\n");
		return;
	}
	XEvent event;
	memset(&event, 0x00, sizeof(event)); 
	event.type = ButtonPress;
	event.xbutton.button = button;
	event.xbutton.same_screen = True;
	XQueryPointer(display, RootWindow(display, DefaultScreen(display)), &event.xbutton.root, &event.xbutton.window, &event.xbutton.x_root, &event.xbutton.y_root, &event.xbutton.x, &event.xbutton.y, &event.xbutton.state);

	event.xbutton.subwindow = event.xbutton.window;

	while(event.xbutton.subwindow)
	{
		event.xbutton.window = event.xbutton.subwindow;

		XQueryPointer(display, event.xbutton.window, &event.xbutton.root, &event.xbutton.subwindow, &event.xbutton.x_root, &event.xbutton.y_root, &event.xbutton.x, &event.xbutton.y, &event.xbutton.state);
	}

	if(XSendEvent(display, PointerWindow, True, 0xfff, &event) == 0) printf("Errore nell'invio dell'evento !!!\n");

	XFlush(display);
	XCloseDisplay(display);
}

void MouseController::mouseRelease( int button )
{
	Display *display = XOpenDisplay(NULL);
	if(display == NULL)
	{
		printf("Errore nell'apertura del Display !!!\n");
		return;
	}
	XEvent event;
	memset(&event, 0x00, sizeof(event)); 
	event.type = ButtonRelease;
	event.xbutton.button = button;
	event.xbutton.same_screen = True;
	XQueryPointer(display, RootWindow(display, DefaultScreen(display)), &event.xbutton.root, &event.xbutton.window, &event.xbutton.x_root, &event.xbutton.y_root, &event.xbutton.x, &event.xbutton.y, &event.xbutton.state);

	event.xbutton.subwindow = event.xbutton.window;

	while(event.xbutton.subwindow)
	{
		event.xbutton.window = event.xbutton.subwindow;

		XQueryPointer(display, event.xbutton.window, &event.xbutton.root, &event.xbutton.subwindow, &event.xbutton.x_root, &event.xbutton.y_root, &event.xbutton.x, &event.xbutton.y, &event.xbutton.state);
	}

	if(XSendEvent(display, PointerWindow, True, 0xfff, &event) == 0) printf("Errore nell'invio dell'evento !!!\n");

	XFlush(display);
	XCloseDisplay(display);
}

MouseController & MouseController::instance()
{
	if (_pInstance == NULL) {
		_pInstance = new MouseController();		
	}

	return *_pInstance;
}
