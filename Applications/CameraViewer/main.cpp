#include <pangolin/pangolin.h>
#include <pangolin/timer.h>

#include <HAL/Camera/CameraDevice.h>
#include <HAL/Utils/GetPot>

int main( int argc, char* argv[] )
{
    GetPot cl(argc,argv);
    hal::Camera camera(cl.follow("", "-cam" ));

    // Capture first image
    pb::ImageArray imgs;
    camera.Capture(imgs);

    // N cameras, each w*h in dimension, greyscale
    const int N = imgs.Size();
    const int w = imgs[0].Width();
    const int h = imgs[0].Height();

    // Setup OpenGL Display (based on GLUT)
    pangolin::CreateGlutWindowAndBind(__FILE__,N*w,h);

    glPixelStorei(GL_PACK_ALIGNMENT,1);
    glPixelStorei(GL_UNPACK_ALIGNMENT,1);
    pangolin::GlTexture tex(w,h,GL_RGB8);

    // Create Smart viewports for each camera image that preserve aspect
    pangolin::View& container = pangolin::CreateDisplay().SetLayout(pangolin::LayoutEqual);
    for(int i=0; i<N; ++i ) {
        container.AddDisplay(pangolin::CreateDisplay().SetAspect((double)w/h));
    }

    bool run = true;
    bool step = false;

    pangolin::RegisterKeyPressCallback(' ', [&run](){run = !run;} );
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + GLUT_KEY_RIGHT, [&step](){step=true;} );

    pangolin::Timer timer;

    for(unsigned long frame=0; !pangolin::ShouldQuit();)
    {
        const bool go = run || pangolin::Pushed(step);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glColor3f(1,1,1);

        if(go) {
            if(frame>0) {
                camera.Capture(imgs);
            }

            frame++;

            if(frame%30 == 0) {
                std::cout << "FPS: " << frame / timer.Elapsed_s() << "\r";
                std::cout.flush();
                frame = 0;
                timer.Reset();
            }
        }

        for(int i=0; i<N; ++i ) {
            container[i].Activate();
            tex.Upload(
                imgs[i].data(),
                        imgs[i].Format(), imgs[i].Type()
            );
            tex.RenderToViewportFlipY();
        }

        pangolin::FinishGlutFrame();
    }
}
