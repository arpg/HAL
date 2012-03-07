/*
 * File:   GLConsole.h
 * Author: jmf
 *
 * Created on February 28, 2012, 6:09 PM
 */

#ifndef GLCONSOLE_H
#define	GLCONSOLE_H

#include <FLConsole/FLConsole.h>
#include <FL/Fl_Gl_Window.H>

#include <boost/thread.hpp>

#include "GLHelpers.h"
#include "Common.h"

#define dFPS 0.05 // ---> 1/20 = 20 Hz

class GLWindow : public Fl_Gl_Window
{
    /// timer callback drives animation and forces drawing at 20 hz
    static void _Timer(void *userdata) {
        GLWindow *pWin = (GLWindow*)userdata;
        pWin->redraw();
        Fl::repeat_timeout( dFPS, _Timer, userdata );
    }

    public:

    /// Constructor.
    GLWindow(int x,int y,int w,int h,const char *l=0) : Fl_Gl_Window(x,y,w,h,l)
    {
        Fl::add_timeout( dFPS, _Timer, (void*)this );
    }

    /// Init OpenGL
    void Init()
    {
        // OpenGL settings
        glShadeModel( GL_SMOOTH );
        glEnable( GL_DEPTH_TEST );
        glDepthFunc( GL_LEQUAL );
        glEnable(GL_LINE_SMOOTH);
        glClearColor( 0.0, 0.0, 0.0, 1.0 );
    }

    /// Main function called by FLTK to draw the scene.
    void draw() {

        if( !context() ) {
            return;
        }

        CheckForGLErrors();

        // handle reshaped viewport
        if ( !valid() ) {
            ReshapeViewport( w(), h() );
        }

        // Initialization
        static bool bInitialized = false;
        if ( !bInitialized || !context() ) {
            bInitialized = true;
            Init();
            return;
        }

        // Clear
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

        m_Console.draw();

        CheckForGLErrors();
    }


    /// Pass input to FLConsole.
    int handle( int e )
	{
		if( m_Console.IsOpen() ) {
			return m_Console.handle( e );
		} else {
			switch ( e ) {
			case FL_KEYBOARD:
				switch( Fl::event_key() ) {
				case '`':
					m_Console.OpenConsole();
					break;
				}
				return 1;
			}
		}
	return 0;
    }

    private:
        FLConsoleInstance 	   			m_Console;
};


class GLConsole {

public:
    GLConsole()
	{
		m_bIsStarted = false;
	}

    virtual ~GLConsole()
	{}

	void Start() {
		if( m_bIsStarted == false ) {
			m_pThread = new boost::thread(GLConsole::_ThreadFunction, this);
			m_bIsStarted = true;
		}
	}

	void Stop()
	{
		if( m_bIsStarted == true ) {
			m_pThread->interrupt();
			m_pThread->join();

			m_bIsStarted = false;
		}
	}

private:
	static void _ThreadFunction(GLConsole* /* pConsole */)
	{
		GLWindow* pWin = new GLWindow( 0, 0, 1024, 512, "Mochaccino" );
		pWin->end();
		pWin->resizable( pWin );
		pWin->show();
		Fl::run(); // will loop forever
		exit(0);
	}

private:
	bool										m_bIsStarted;
	boost::thread*								m_pThread;


};


#endif	/* GLCONSOLE_H */