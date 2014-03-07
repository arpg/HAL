#include <gui-engine.h>

GuiEngine::GuiEngine(){}

GuiEngine::~GuiEngine(){}

////////////////////////////////////////////////////////////////////////////////
void GuiEngine::Init(const std::string window_name,
                const unsigned int window_width,
                const unsigned int window_height,
                const bool play_at_start)
{
  is_playing_  = play_at_start;
  is_stepping_ = !is_playing_;

  // Create OpenGL window
  pangolin::CreateWindowAndBind(window_name, window_width, window_height);

  // Add views
  pangolin::DisplayBase().AddDisplay(camera_view_);
  pangolin::DisplayBase().AddDisplay(view_3d_);
  pangolin::DisplayBase().AddDisplay(imu_view_);
  pangolin::DisplayBase().AddDisplay(tile_view_);


  // Set views properties
  render_state_.SetModelViewMatrix(
        pangolin::ModelViewLookAt(0, 50, -50, 0, 0, 0, pangolin::AxisNegZ));
  render_state_.SetProjectionMatrix(
        pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000000));
  graph_3d_.AddChild(&gl_grid_);
  graph_3d_.AddChild(&car_mesh_);
  view_3d_.SetHandler(new SceneGraph::HandlerSceneGraph(graph_3d_, render_state_, pangolin::AxisNegZ))
          .SetDrawFunction(SceneGraph::ActivateDrawFunctor(graph_3d_, render_state_));
  imu_view_.AddDisplay(pangolin::CreatePlotter("Accel", &plot_log_accel_))
           .AddDisplay(pangolin::CreatePlotter("Gyro", &plot_log_gyro_))
           .AddDisplay(pangolin::CreatePlotter("Mag", &plot_log_mag_));


  camera_view_.SetBounds(0.8,1.0,0.0,1.0);
  view_3d_.SetBounds(0.0,0.8,0.0,0.7,-640.0/480.0);
  //view_3d_.SetLock(pangolin::LockLeft,pangolin::LockBottom);
  imu_view_.SetBounds(0.5,0.8,0.7,1.0);
  tile_view_.SetBounds(0.0,0.5,0.7,1.0);
  tile_view_.SetZoom(17);

  // [TEST] Create a gps trajectory
  tile_view_.AddGPSPoint(38.903973, -77.048918);
  tile_view_.AddGPSPoint(38.903004, -77.048875);
  tile_view_.AddGPSPoint(38.902152, -77.048875);
//  tile_view.AddGPSPoint(38.901735, -77.047781);
//  tile_view.UpdateMap(38.901368, -77.046687);
//  tile_view.UpdateMap(38.902052, -77.046708);
//  tile_view.UpdateMap(38.902904, -77.046687);


  // Issue specific OpenGl we might need
  glEnable (GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glPixelStorei(GL_PACK_ALIGNMENT,1);
  glPixelStorei(GL_UNPACK_ALIGNMENT,1);
}

////////////////////////////////////////////////////////////////////////////////
void GuiEngine::Run()
{
  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glColor4f(1.0f,1.0f,1.0f,1.0f);
    usleep(20000);
    pangolin::FinishGlutFrame();
  }
}

////////////////////////////////////////////////////////////////////////////////
void GuiEngine::ImuHandler(pb::ImuMsg& imu_data)
{

  if ( imu_data.has_accel() ) {
    plot_log_accel_.Log( imu_data.accel().data(0),
                        imu_data.accel().data(1),
                        imu_data.accel().data(2) );
  }
  if ( imu_data.has_gyro() ) {
    plot_log_gyro_.Log( imu_data.gyro().data(0),
                       imu_data.gyro().data(1),
                       imu_data.gyro().data(2) );
  }
  if ( imu_data.has_mag() ) {
    plot_log_mag_.Log( imu_data.mag().data(0),
                      imu_data.mag().data(1),
                      imu_data.mag().data(2) );
  }
}

////////////////////////////////////////////////////////////////////////////////
void GuiEngine::PosysHandler(pb::PoseMsg& pose_data)
{
  double lat = pose_data.pose().data(0);
  double lon = pose_data.pose().data(1);
  tile_view_.AddGPSPoint(lat,lon);
}

////////////////////////////////////////////////////////////////////////////////
void GuiEngine::LoadCarMesh(std::string filename)
{
  try
  {
    car_mesh_.Init( filename );
    car_mesh_.SetPerceptable( true );
    car_mesh_.SetScale( 0.1 );
    car_mesh_.SetPose( 0, 0, 1, -M_PI/2, 0, M_PI/2);
    car_mesh_.SetAlpha(0.4);
    std::cout << "MeshLogger: Mesh '" << filename << "' loaded." << std::endl;
  } catch( std::exception ) {
    std::cerr << "error: Cannot load mesh. Check file exists." << std::endl;
    exit(EXIT_FAILURE);
  }
}

////////////////////////////////////////////////////////////////////////////////
void GuiEngine::LidarHandler(pb::LidarMsg& lidar_data)
{

}
