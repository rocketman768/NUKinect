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
