#include <pangolin/pangolin.h>
#include <pangolin/timer.h>

#include <HAL/Camera/CameraDevice.h>
#include <HAL/Utils/GetPot>

#include <PbMsgs/Logger.h>

int main( int argc, char* argv[] )
{
    GetPot cl(argc,argv);
    const std::string logfile = cl.follow("proto.log", "-log");    
    
    hal::Camera camera(cl.follow("", "-cam" ));

    // Capture first image
    pb::ImageArray imgs;

    // N cameras, each w*h in dimension, greyscale
    const size_t N = camera.NumChannels();
    const size_t w = camera.Width();
    const size_t h = camera.Height();
    
    std::cout << "Opening camera with " << N << " channel(s)." << std::endl;
    for(size_t i=0; i<N; ++i) {
        std::cout << "  " << camera.Width(i) << "x" << camera.Height(i) << std::endl;
    }

    // Setup OpenGL Display (based on GLUT)
    pangolin::CreateGlutWindowAndBind(__FILE__,N*w,h);

    glPixelStorei(GL_PACK_ALIGNMENT,1);
    glPixelStorei(GL_UNPACK_ALIGNMENT,1);
    pangolin::GlTexture tex(w,h,GL_RGB8);

    // Create Smart viewports for each camera image that preserve aspect
    pangolin::View& container = pangolin::CreateDisplay().SetLayout(pangolin::LayoutEqual);
    for(size_t i=0; i < N; ++i ) {
        container.AddDisplay(pangolin::CreateDisplay().SetAspect((double)w/h));
    }

    bool run = true;
    bool step = false;
    bool log = false;

    pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + GLUT_KEY_RIGHT, [&step](){step=true;} );
    pangolin::RegisterKeyPressCallback(' ', [&](){run = !run;} );
    
    pangolin::RegisterKeyPressCallback('l', [&](){log = !log;} );

    pangolin::Timer timer;
    
    pb::Logger::GetInstance().OpenLogFile(logfile);

    for(unsigned long frame=0; !pangolin::ShouldQuit(); frame++)
    {
        const bool go = run || pangolin::Pushed(step);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glColor3f(1,1,1);

        if(go) {
            if( !camera.Capture(imgs) ) {
                run = false;
            }

            if(frame%30 == 0) {
                char buffer[1024];
                sprintf(buffer,"CameraViewer (FPS: %f)", 30.0 / timer.Elapsed_s() );
                glutSetWindowTitle(buffer);
                timer.Reset();
            }
        }

        for(size_t i=0; i<imgs.Size(); ++i ) {
            container[i].Activate();
            tex.Upload(
                imgs[i].data(),
                imgs[i].Format(), imgs[i].Type()
            );
            tex.RenderToViewportFlipY();
        }
        
        if(log && run) {
            pb::Msg msg;
            msg.set_timestamp(frame);                
            msg.mutable_camera()->Swap(&imgs.Ref());
            pb::Logger::GetInstance().LogMessage(msg);
        }

        pangolin::FinishGlutFrame();
    }
}
