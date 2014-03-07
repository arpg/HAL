#include <string>
#include <pangolin/pangolin.h>
#include <PbMsgs/Imu.pb.h>
#include <PbMsgs/Lidar.pb.h>
#include <PbMsgs/Pose.pb.h>
#include <PbMsgs/Messages.pb.h>
#include <pangolin_tile_view.h>

class GuiEngine {

public:
  GuiEngine();
  ~GuiEngine();

  void Init( const std::string  window_name,
             const unsigned int window_width = 1024,
             const unsigned int window_height = 768,
             const bool         play_at_start = true);

  void Run();

  void LidarHandler(pb::LidarMsg& lidar_data);
  void ImuHandler(pb::ImuMsg& imu_data);
  void PosysHandler(pb::PoseMsg& pose_data);
  void LoadCarMesh(std::string filename);

private:
  bool is_playing_;
  bool is_stepping_;
  bool have_camera_;
  bool have_imu_;
  bool have_gps_;
  bool have_velodyne;


  // For displaying the cameras
  pangolin::View camera_view_;


  // For displaying the imu
  pangolin::View imu_view_;
  pangolin::DataLog plot_log_accel_;
  pangolin::DataLog plot_log_gyro_;
  pangolin::DataLog plot_log_mag_;

  // For displaying the velodine and the car
  pangolin::View view_3d_;
  pangolin::OpenGlRenderState render_state_;
  SceneGraph::GLMesh car_mesh_;
  SceneGraph::GLSceneGraph graph_3d_;
  SceneGraph::GLGrid gl_grid_;

  // For displaying GPS location
  pangolin::TileView tile_view_;


};

