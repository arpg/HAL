#pragma once

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <curl/curl.h>
#include <pangolin/pangolin.h>
#include <gl_tile.h>
#include <gl_ways.h>
#include <OpenStreetMap.h>
#include <GLSimpleObjects.h>

namespace pangolin {

  class TileView : public View
  {

    // Structure that contains variables that determine the position
    // and extent of each component in the view
    struct TileViewLayout{

      TileViewLayout():
        tile_pixel_size_in_meters(0.0f),
        tile_size_in_window(80.0f),
        top_panel_size_in_window(50.0f),
        gl_map_top(0.0f),
        gl_map_left(0.0f),
        gl_map_left_border(50.0f),
        gl_map_right_border(50.0f),
        gl_map_top_border(50.0f),
        gl_map_bottom_border(50.0f),
        num_map_cell_cols(0),
        num_map_cell_rows(0)
      {}

      float tile_pixel_size_in_meters;
      float tile_size_in_window;
      float top_panel_size_in_window;
      float gl_map_top;
      float gl_map_left;
      float gl_map_left_border;
      float gl_map_right_border;
      float gl_map_top_border;
      float gl_map_bottom_border;
      int   num_map_cell_cols;
      int   num_map_cell_rows;
    };

    // Structure that groups varibles used for storing the display
    // characteristics of the components of the view (effects, colors, etc)
    struct TileViewDisplayVars {

      TileViewDisplayVars():
        show_map_grid(true),
        min_tile_row_id(-1),
        min_tile_col_id(-1),
        max_tile_row_id(-1),
        max_tile_col_id(-1),
        tile_neighbours_distance(1),
        current_position_rad(2),
        current_position_min_rad(2),
        current_position_max_rad(12),
        current_position_rad_step(0.1f)
      {}

      // display map grid
      bool show_map_grid;

      // variables for controlling which tiles are displayed
      int min_tile_row_id;
      int min_tile_col_id;
      int max_tile_row_id;
      int max_tile_col_id;
      int tile_neighbours_distance;

      // variables for the pulsating current position effect
      float current_position_rad;
      float current_position_min_rad;
      float current_position_max_rad;
      float current_position_rad_step;

    };

  public:

    // default constructor
    TileView(int zoom=18, int src=1):
      zoom_(zoom),
      src_(src)
    {
      // value taken from:
      // http://wiki.openstreetmap.org/wiki/
      // Slippy_map_tilenames#Resolution_and_Scale
      base_resolution_ = 156543.034;
    }

    // default destructor
    ~TileView()
    {
      for (unsigned int ii=0; ii < tiles_.size(); ++ii) {
        delete tiles_[ii];
      }
      tiles_.clear();
    }

    void Render();

    virtual void Resize(const pangolin::Viewport& parent);

    void AddGPSPoint(const float latitude,
                     const float longitude,
                     const float altitude = 0.0f);

    void SetSource(const int source) {
      src_ = source;
    }

    void SetZoom(const int zoom ) {
      zoom_ = zoom;
    }

    void ToggleGrid() {
      display_.show_map_grid = !display_.show_map_grid;
    }


  private:

    void _compute_layout(osm::GPSPoint &pnt);

    void _draw_grid(float cell_size_scale,
                    float line_width,
                    SceneGraph::GLColor color, bool draw_vertices, bool draw_cell_size);

    void _draw_gps_trajectory();

    void _shift_grid_if_at_the_border();

    void _gps_to_cartesian(osm::GPSPoint &pnt, int zoom);

    void _get_tile_and_neighbours(const osm::GPSPoint &pnt);

    void _get_tile(osm::GPSPoint& pnt, gui::GLTile* tile);

    void _read_gps_file(const std::string filename);

    void _download_tile_to_mat(const int tile_x,
                               const int tile_y,
                               cv::Mat &tile);

    void _download_osm_data(const double west, const double south,
                            const double east, const double north,
                            std::vector<osm::Way>& ways_out);

    void _check_ways_ids(std::vector<int> ways_ids,
                         std::vector<osm::Way>& ways);

    void _convert_ways_gps_to_utm(std::vector<osm::Way>& ways);

    static size_t  _write_data( char *ptr, size_t size, size_t nmemb,
                                void *userdata );


  private:

    int zoom_;
    int src_;
    float base_resolution_;

    osm::GPSPoint origin_;
    std::vector<osm::GPSPoint> gps_points_;
    std::vector<gui::GLTile*> tiles_;
    std::vector<int> ways_ids_;
    gui::GLWays gl_ways_;

    TileViewLayout layout_;
    TileViewDisplayVars display_;

    // GLObjects
    SceneGraph::GLText text_;

    // Projection matrix
    pangolin::OpenGlMatrix ortho_;

    // mutex for blocking drawing while updating data
    std::mutex mutex_;

  };

}
