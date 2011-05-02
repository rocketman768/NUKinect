#ifndef _MOUSE_CONTROLLER_H
#define _MOUSE_CONTROLLER_H
#include <X11/X.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include<stdio.h>
#include<string.h>
class MouseController
{
public:
	static MouseController & instance();

	void init();

	void GetCursorPos(int &x,int &y);
	void SetCursorPos(int x,int y);

	void mousePress(int button);
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