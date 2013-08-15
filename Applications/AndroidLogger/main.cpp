#include <pangolin/pangolin.h>
#include <pangolin/gldraw.h>

#include <HAL/Camera/CameraDevice.h>
#include <HAL/IMU/IMUDevice.h>

#include <PbMsgs/Logger.h>
#include <PbMsgs/Matrix.h>

// global logger
bool        g_Log = false;
pb::Logger& g_Logger = pb::Logger::GetInstance();


void HandleIMU(pb::ImuMsg& IMUdata)
{
    if( g_Log ) {
        pb::Msg msg;
        msg.set_timestamp( IMUdata.device_time() );
        msg.mutable_imu()->Swap(&IMUdata);
        g_Logger.LogMessage(msg);
    }
}


int main( int /*argc*/, char** /*argv*/ )
{
    pangolin::CreateWindowAndBind("Main");

    ////////////////////////////////////////////////////////////////////

//    const std::string video_uri = "android://";
    const std::string video_uri = "raisin://";
//    const std::string imu_uri = "raisin://";
    hal::Camera camera(video_uri);
    pb::ImageArray images;

//    hal::IMU imu(imu_uri);
//    imu.RegisterIMUDataCallback(HandleIMU);

    const size_t N = camera.NumChannels();
    const size_t w = camera.Width();
    const size_t h = camera.Height();

    LOGI("Preview video: %d x %d x %d\n", N, w, h);

    g_Logger.SetMaxBufferSize( 2000 );

    ///---------------------------

    // prepare logger
    time_t g_t = time(NULL);
    struct tm tm_result;
    localtime_r(&g_t, &tm_result);
    char prefix[256];
    strftime(prefix, sizeof(prefix), "%Y%b%d_%H%M%S", &tm_result);
//    std::string fullPath = g_Logger.LogToFile( "/data/data/edu.gwu.robotics.AndroidLogger/files/", prefix );
    std::string fullPath = g_Logger.LogToFile( "/sdcard/", prefix );
    LOGI("Logger started at: %s",fullPath.c_str());

    ////////////////////////////////////////////////////////////////////
    // Setup GUI

    const int PANEL_WIDTH=180;

    // Make things look prettier...
//    glEnable(GL_LINE_SMOOTH);
//    glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );
//    glHint( GL_LINE_SMOOTH_HINT, GL_NICEST );
//    glEnable (GL_BLEND); // Doesn't seem to work on Android
//    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//    glDepthFunc( GL_LEQUAL );
//    glEnable( GL_DEPTH_TEST );
//    glLineWidth(1.7);

//    pangolin::OpenGlRenderState stacks;
//    stacks.SetProjectionMatrix(pangolin::ProjectionMatrixRDF_TopLeft(640,480,420,420,320,240,0.01,1E6));
//    stacks.SetModelViewMatrix(pangolin::ModelViewLookAtRDF(0,0,-0.5, 0,0,0, 0, -1, 0) );

    pangolin::Panel panel("ui");
    panel.SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(PANEL_WIDTH));
    pangolin::DisplayBase().AddDisplay(panel);

    pangolin::View container;
    container.SetBounds(0.0,1.0, pangolin::Attach::Pix(PANEL_WIDTH),1.0)
             .SetLayout(pangolin::LayoutEqual);

    for(size_t c=0; c < N; ++c) {
        container.AddDisplay( pangolin::CreateDisplay().SetAspect(w/(float)h) );
    }

    /*
    pangolin::Handler3D handler(stacks);
    pangolin::View v3D;
    v3D.SetBounds(0.0, 1.0, pangolin::Attach::Pix(PANEL_WIDTH), 1.0, -640.0f/480.0f);
    v3D.SetHandler(&handler);
    container.AddDisplay(v3D);
    */
    pangolin::DisplayBase().AddDisplay(container);

    // OpenGl Texture for video frame
    pangolin::GlTexture tex(w,h,GL_LUMINANCE,true,0,GL_LUMINANCE,GL_UNSIGNED_BYTE);

    ////////////////////////////////////////////////////////////////////
    // Display Variables

    pangolin::Var<bool> log("ui.LOG", false, true);
    pangolin::Var<bool> run("ui.PLAY", false, false);


    //********************************************************************


    for( unsigned int nFrame = 0; !pangolin::ShouldQuit(); nFrame++ )
    {
        if( Pushed(run) ) {
            pangolin::Quit();
            break;
        }

        if( !camera.Capture(images) ) {
            // error!
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        for(size_t iI = 0; iI < N; ++iI)
        {
            if(container[iI].IsShown()) {
                container[iI].ActivateIdentity();
                glColor3f(1,1,1);

                // Display camera image
                tex.Upload(images[iI].data(),GL_LUMINANCE,GL_UNSIGNED_BYTE);
                pangolin::glDrawTextureFlipY(GL_TEXTURE_2D,tex.tid);

                // Setup orthographic pixel drawing
            //    glMatrixMode(GL_PROJECTION);
              //  glLoadIdentity();
               // glOrtho(-0.5,w-0.5,h-0.5,-0.5,0,1.0);
                //glMatrixMode(GL_MODELVIEW);
            }
        }

        /*
        if(v3D.IsShown()) {
            v3D.ActivateAndScissor(stacks);
        }
        */

//        if( log && (nFrame % 2 == 0) ) {
        g_Log = log;
        if( log ) {
            g_Log = true;
            pb::Msg msg;
            msg.set_timestamp(nFrame);
            msg.mutable_camera()->Swap(&images.Ref());
            if( g_Logger.LogMessage(msg) == false ) {
                log = false;
                LOGI("LOGGER WAS STOPPED!");
            }
        }

        // Process window events via GLUT
        pangolin::FinishFrame();
    }

    LOGV("-main");
}
