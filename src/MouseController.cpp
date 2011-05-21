#include "MouseController.h"

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
	int tmp;
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
