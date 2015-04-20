#pragma once

#include <pangolin/pangolin.h>

/////////////////////////////////////////////////////////////////////////////
// Code to render the vehicle path
class GLThumbnailView : public pangolin::View
{

public:

    void Init();
    void Render();

private:
    
    unsigned int m_nWindowSize;
    unsigned int m_nThumbWidth;
    unsigned int m_nThumbHeight;
};

/////////////////////////////////////////////////////////////////////////////
void GLThumbnailView::Init()
{
    // TODO
}

/////////////////////////////////////////////////////////////////////////////
//! Render this view.
void GLThumbnailView::Render()
{
    // Call base View implementation
    pangolin::View::Render();

}

