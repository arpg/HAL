#include <iostream>

#include <HAL/Camera/CameraDevice.h>
#include <HAL/Car/CarDevice.h>

#include <pangolin/pangolin.h>

int main(/*int argc, char **argv*/)
{
  hal::Camera cam("openni:[name=LCamera,rgb=1,depth=1]//");
//  hal::Car car("car://");//Uri doesn't has any significance.
  int winw = cam.NumChannels() * cam.Width();
  int winh = cam.Height();

  pangolin::CreateWindowAndBind("TestSim", winw, winh);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glPixelStorei(GL_UNPACK_ALIGNMENT,1);

  pangolin::View& container = pangolin::CreateDisplay()
                                        .SetLayout(pangolin::LayoutEqual);
  for (int display = 0; display < cam.NumChannels(); ++display)
  {
    container.AddDisplay(pangolin::CreateDisplay()
                         .SetAspect(cam.Width()/(double)cam.Height()));
  }

  pangolin::GlTexture tex[cam.NumChannels()];
  bool run = true;
  std::shared_ptr<pb::ImageArray> imgs = pb::ImageArray::Create();
  for (int frame = 0; !pangolin::ShouldQuit(); ++frame)
  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glColor3f(1,1,1);

    if(run) {
      std::cout<<"Trying to capture"<<std::endl;
        if( !cam.Capture( *imgs) ) {
            run = false;
            std::cout<<"No image recieved, Quiting."<<std::endl;
            break;
        }
    }

    for(size_t i=0; i<imgs->Size(); ++i ) {
      pb::Image img = *(imgs->at(i));
      if(!tex[i].tid) {
        tex[i].Reinitialise(img.Width(), img.Height(),
                              img.Format(), true, 0,
                              img.Format(), img.Type(), 0);
      }
        container[i].Activate();
        tex[i].Upload(
            img.data(),
            img.Format(), img.Type()
        );
        tex[i].RenderToViewportFlipY();
    }
    pangolin::FinishFrame();
  }
}
