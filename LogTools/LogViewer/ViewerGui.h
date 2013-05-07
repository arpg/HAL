/* 
 * File:   ViewerGui.h
 * Author: alonso
 *
 * Created on May 7, 2013, 3:28 PM
 */

#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>
#include "GLThumbnails.h"

class ViewerGui
{
    
public:

    ViewerGui(const std::string  name   = "BEAVER",
              const unsigned int width  = 640,
              const unsigned int height = 480)
            : m_sWindowName( name ),
              m_uWindowWidth( width ),
              m_uWindowHeight( height ){}

    void Init( int argc, char** argv );
    void Run();

private:
    void _MouseMotion( int x, int y );
    void _RegisterKeyboardCallbacks();
    void _RegisterCVars();

private:


    std::string  m_sWindowName;
    unsigned int m_uWindowWidth;
    unsigned int m_uWindowHeight;

    // GL Objects
    // TODO

    // Images
    std::vector< SceneGraph::ImageView > m_vSelectedImages;
    GLThumbnails                         m_glThumbnails;

};

/////////////////////////////////////////////////////////////////////////
void ViewerGui::Init( int argc, char **argv )
{
    // Register keyboard callbacks
    _RegisterKeyboardCallbacks();

    // Register CVARS
    _RegisterCVars();

    // Create OpenGL window in single line thanks to GLUT
    pangolin::CreateGlutWindowAndBind(m_sWindowName,m_uWindowWidth,m_uWindowHeight );

    // Add views to window
    // e.g. [pangolin::DisplayBase().AddDisplay( m_View3d );]

    // Set up SceneGraphs
    SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
    glClearColor(0.0,0.0,0.0,1.0);
}

///////////////////////////////////////////////////////////////////////////////
void ViewerGui::Run()
{
    // Default hooks for exiting (Esc) and fullscreen (tab).
    while( 1 ) {

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Swap frames and Process Events
        pangolin::FinishGlutFrame();
 
        usleep(1000000 / 120);
    }
}

/////////////////////////////////////////////////////////////////////////
void ViewerGui::_RegisterCVars()
{
    // no cvars yet
}

///////////////////////////////////////////////////////////////////////////////
void ViewerGui::_RegisterKeyboardCallbacks()
{
    // no keyboard callbacks yet
    // e.g. [pangolin::RegisterKeyPressCallback( 'h', boost::bind( &Gui::_KEYBOARD_H, this ) );]
}

/////////////////////////////////////////////////////////////////////////
void ViewerGui::_MouseMotion( int x, int y )
{
    // nothing to capture yet
}


