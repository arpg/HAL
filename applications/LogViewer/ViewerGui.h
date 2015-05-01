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
#include <Widgets/GLWidgetView.h>
#include "GLThumbnailView.h"

class ViewerGui
{
    
public:

    ViewerGui(const std::string  sName   = "BEAVER",
              const unsigned int nWidth  = 640,
              const unsigned int nHeight = 480
             )
             : m_sWindowName( sName ),
               m_nWindowWidth( nWidth ),
               m_nWindowHeight( nHeight ){}

    void Init( int argc, char** argv );
    void Run();

private:
    void _MouseMotion( int x, int y );
    void _RegisterKeyboardCallbacks();
    void _RegisterCVars();

private:

    // Gui vars
    std::string  m_sWindowName;
    unsigned int m_nWindowWidth;
    unsigned int m_nWindowHeight;

    // Pangolin views
    GLWidgetView          m_WidgetView;
    GLThumbnailView       m_ThumbnailView;
    SceneGraph::ImageView m_ImageView;

    // SceneGraph GLObjects

    // Widget vars
    bool m_bCheckBox1;
    bool m_bCheckBox2;

};

/////////////////////////////////////////////////////////////////////////
void ViewerGui::Init( int argc, char **argv )
{
    // Register keyboard callbacks
    _RegisterKeyboardCallbacks();

    // Register CVARS
    _RegisterCVars();

    // Create OpenGL window in single line thanks to GLUT
    pangolin::CreateGlutWindowAndBind( m_sWindowName,
                                       m_nWindowWidth,
                                       m_nWindowHeight );

    // Add views to window
    // e.g. [pangolin::DisplayBase().AddDisplay( m_View3d );]

    // Set up SceneGraphs
    SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
    glClearColor(0.0,0.0,0.0,1.0);

    // Set up Panel
    m_WidgetView.Init(0, 1, 0, Attach::Pix(200),
        [&] (nv::GlutUIContext& context,nv::Rect& rect) {
            context.beginFrame(nv::GroupFlags_GrowDownFromLeft,rect);
                 context.doCheckButton(nv::Rect(),"Checkbox1:", &m_bCheckBox1);
                 context.doCheckButton(nv::Rect(),"Checkbox2:", &m_bCheckBox2);
                 context.beginGroup(nv::GroupFlags_GrowRightFromTop);
                     context.doLabel(nv::Rect(),"123");
                     context.doLabel(nv::Rect(),"456");
                 context.endGroup();
            context.endFrame();
        }
    );

    pangolin::DisplayBase().AddDisplay( m_WidgetView );

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
    // e.g. [pangolin::RegisterKeyPressCallback( 'h', std::bind( &Gui::_KEYBOARD_H, this ) );]
}

/////////////////////////////////////////////////////////////////////////
void ViewerGui::_MouseMotion( int x, int y )
{
    // nothing to capture yet
}


