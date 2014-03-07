#pragma once

#include <SceneGraph/SceneGraph.h>
#include <pangolin/gldraw.h>
#include <eigen3/Eigen/Core>

namespace gui
{

////////////////////////////////////////////////////////////////////////////////
inline void glDrawEllipse(int segments, GLfloat width, GLfloat height,
                          GLfloat x1, GLfloat y1, bool filled)
{
    glPushMatrix();
    glTranslated(x1, y1, 0.0f);
    GLfloat vertices[segments*2];
    GLfloat factor = M_PI / 180.0f;
    int count=0;
    for( GLfloat i = 0; i < 360.0f; i+=(360.0f/segments) ){
        vertices[count++] = (cos(factor*i)*width);
        vertices[count++] = (sin(factor*i)*height);
    }
    glVertexPointer( 2, GL_FLOAT , 0, vertices );
    glEnableClientState( GL_VERTEX_ARRAY );
    glDrawArrays( (filled) ? GL_TRIANGLE_FAN : GL_LINE_LOOP, 0, segments );
    glDisableClientState( GL_VERTEX_ARRAY );

    glPopMatrix();
}

////////////////////////////////////////////////////////////////////////////////
inline void drawCircle(GLfloat x1, GLfloat y1,
                       GLfloat radius_x, GLfloat radius_y,
                       bool filled = false)
{
    glDrawEllipse(30, radius_x, radius_y, x1, y1, filled);
}

////////////////////////////////////////////////////////////////////////////////
inline void drawSolidCircle(GLfloat x1,
                            GLfloat y1,
                            GLfloat radius_x,
                            GLfloat radius_y)
{
    drawCircle(x1, y1, radius_x, radius_y,  true);
}

////////////////////////////////////////////////////////////////////////////////
// parallel to the x-z plane
inline void drawCircle3D(GLfloat x1, GLfloat y1,  GLfloat radius, GLfloat z)
{
    int segments = 30;
    int count = 0;
    GLfloat vertices[segments*3];
    glTranslated(x1, y1, z);
    for(int angle = 0; angle <= 360; angle+=(360 / segments))
    {
        GLfloat radians = M_PI*(GLfloat)angle/180;
        vertices[count++] = x1 + sin(radians) * radius;
        vertices[count++] = y1 + cos(radians) * radius;
        vertices[count++] = z;
    }
    glVertexPointer (3, GL_FLOAT , 0, vertices);
    glDrawArrays (GL_LINE_LOOP, 0, segments);
}

////////////////////////////////////////////////////////////////////////////////
// Parallel to the x,y,z axis
inline void glDrawRectPerimeter3D(GLfloat x1, GLfloat y1,
                                  GLfloat x2, GLfloat y2, GLfloat z)
{
    pangolin::glDrawLine(x1,y1,z,x1,y2,z);
    pangolin::glDrawLine(x1,y1,z,x2,y1,z);
    pangolin::glDrawLine(x2,y2,z,x1,y2,z);
    pangolin::glDrawLine(x2,y2,z,x2,y1,z);
}

////////////////////////////////////////////////////////////////////////////////
// Parallel to the x,y,z axis
inline void drawWiredBox(GLfloat x1, GLfloat y1, GLfloat z1,
                         GLfloat x2, GLfloat y2, GLfloat z2)
{
    glDrawRectPerimeter3D(x1,y1,x2,y2,z1);
    glDrawRectPerimeter3D(x1,y1,x2,y2,z2);
    pangolin::glDrawLine(x1,y1,z1,x1,y1,z2);
    pangolin::glDrawLine(x2,y2,z1,x2,y2,z2);
    pangolin::glDrawLine(x1,y2,z1,x1,y2,z2);
    pangolin::glDrawLine(x2,y1,z1,x2,y1,z2);
}

////////////////////////////////////////////////////////////////////////////////
inline void drawPoint( float x, float y)
{
    GLfloat verts[] = { x,y };
    glEnableClientState( GL_VERTEX_ARRAY );
    glVertexPointer( 2, GL_FLOAT, 0, verts );
    glDrawArrays( GL_POINTS, 0, 1 );
    glDisableClientState( GL_VERTEX_ARRAY );
}

////////////////////////////////////////////////////////////////////////////////
inline void drawPoint( float x, float y, float z)
{
    GLfloat verts[] = { x, y, z };
    glEnableClientState( GL_VERTEX_ARRAY );
    glVertexPointer( 3, GL_FLOAT, 0, verts );
    glDrawArrays( GL_POINTS, 0, 1 );
    glDisableClientState( GL_VERTEX_ARRAY );
}

////////////////////////////////////////////////////////////////////////////////
inline void glDrawPolygon2d( const GLfloat* verts, const size_t nverts)
{
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(2, GL_FLOAT, 0, verts);
    glDrawArrays(GL_TRIANGLE_FAN, 0, nverts);
    glDisableClientState(GL_VERTEX_ARRAY);
}

}




