
#include <pangolin_tile_view.h>

namespace pangolin {

  ///
  /// \brief TileView::Draw
  ///
  void TileView::Render()
  {
    std::unique_lock<std::mutex> lock(mutex_);

    pangolin::GlState state;
    state.glDisable(GL_LIGHTING);
    state.glDisable(GL_DEPTH_TEST);

    // Activate viewport
    this->Activate();

    // Load orthographic projection matrix to match image
    glMatrixMode(GL_PROJECTION);
    ortho_.Load();

    // Reset ModelView matrix
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Draw bounding box
    glColor3f(1.0,1.0,1.0);
    pangolin::glDrawRectPerimeter(-v.w/2 + 5, -v.h/2 + 5, v.w/2 - 5, v.h/2 - 5);
    pangolin::glDrawLine(-v.w/2 + 5, -v.h/2 + layout_.top_panel_size_in_window,
                          v.w/2 - 5, -v.h/2 + layout_.top_panel_size_in_window);

    float lon, lat, alt;

    if( gps_points_.size() > 0) {

      // draw current position
      lon = gps_points_.back().lon;
      lat = gps_points_.back().lat;
      alt = gps_points_.back().z;

      for (unsigned int ii=0; ii < tiles_.size(); ++ii) {
        if (tiles_[ii]->IsInitialized() &&
            tiles_[ii]->col >= display_.min_tile_col_id &&
            tiles_[ii]->row >= display_.min_tile_row_id &&
            tiles_[ii]->col <= display_.max_tile_col_id &&
            tiles_[ii]->row <= display_.max_tile_row_id) {
          // compute position

          // draw tile at position
          int xoffset = tiles_[ii]->col - display_.min_tile_col_id;
          int yoffset = tiles_[ii]->row - display_.min_tile_row_id;
          GLfloat xpos = layout_.gl_map_left + xoffset*layout_.tile_size_in_window;
          GLfloat ypos = layout_.gl_map_top + yoffset*layout_.tile_size_in_window;

          glTranslatef(xpos,ypos,0.0f);
          tiles_[ii]->Draw();
          glTranslatef(-xpos,-ypos,0.0f);
        }
      }

      if (display_.show_map_grid) {
        _draw_grid(0.25, 0.5f, SceneGraph::GLColor(0,200,200,50),false, false);
        _draw_grid(1.0, 1.0f, SceneGraph::GLColor(255,254,210,150), true, true);
      }
      _draw_gps_trajectory();
    } else {
      lat = lon = alt = 0.0;
      // draw no gps data
      GLfloat cy = layout_.gl_map_top + (v.h/2 - 5 - layout_.gl_map_top)/2;
      GLfloat x1 = -50;
      GLfloat x2 = 50;
      GLfloat y1 = cy - 10;
      GLfloat y2 = cy + 10;
      glColor3f(255.0/255.0,106.0/255.0,74.0/255.0);
      pangolin::glDrawRect(x1,y1,x2,y2);
      glColor3f(1.0,1.0,1.0);
      text_.Draw("No GPS data",-45,cy + 5);
    }

    // Draw top panel info
    GLfloat x = -v.w/2 + 20;
    GLfloat y = -v.h/2 + layout_.top_panel_size_in_window/2 - 2;
    char buffer[30];
    glColor3f(1.0,1.0,1.0);
    text_.Draw("Lat:", x, y);
    sprintf(buffer,"%f", lat);
    text_.Draw(buffer, x+35, y);
    text_.Draw("Lon:", x, y+18);
    sprintf(buffer,"%f", lon);
    text_.Draw(buffer, x+35, y+18); x+=135;
    text_.Draw("Alt:", x, y); x+=35;
    sprintf(buffer,"%0.3f", alt);
    text_.Draw(buffer, x, y); x+=80;
    text_.Draw("Dist:", x, y); x+=45;
    sprintf(buffer,"%.3f m", 0.0);
    text_.Draw(buffer, x, y);

  }

  ///
  /// \brief TileView::Resize
  /// \param[in] parent
  ///
  void TileView::Resize(const pangolin::Viewport& parent)
  {
    pangolin::View::Resize(parent);
    ortho_ = pangolin::ProjectionMatrixOrthographic(-v.w/2, v.w/2, v.h/2, -v.h/2, 0, 1E4);
    if(gps_points_.size()){
      _compute_layout(gps_points_.back());
    }
    _shift_grid_if_at_the_border();
  }

  ///
  /// \brief TileView::UpdateMap.
  ///        Download a new tile if needed that includes the GPS point
  /// \param[in] lat
  /// \param[in] lon
  ///
  void TileView::AddGPSPoint(const float latitude,
                             const float longitude,
                             const float altitude /*=0.0*/)
  {

    //get tile coordinates
    osm::GPSPoint pnt;
    pnt.lon = longitude;
    pnt.lat = latitude;
    pnt.z   = altitude;
    _gps_to_cartesian(pnt, zoom_);

    if (tiles_.empty()) {

      origin_ = pnt;
      // compute map layout
      _compute_layout( origin_ );

      layout_.tile_pixel_size_in_meters =
          base_resolution_ * cos(latitude*M_PI/180.0)/(1 << zoom_);
    }

    // get tile and neighbours
    _get_tile_and_neighbours(pnt);

    gps_points_.push_back(pnt);

    _shift_grid_if_at_the_border();
  }

  void TileView::_compute_layout(osm::GPSPoint& pnt)
  {
    // window coordinates : [right -w/2, left +w/2, top -h/2, bottom +h/2 ]
    float top = -v.h/2;
    float left = -v.w/2;
    layout_.gl_map_top = top + layout_.top_panel_size_in_window + layout_.gl_map_top_border;
    layout_.gl_map_left = left + layout_.gl_map_left_border;

    // compute number of cells
    float map_width = v.w - layout_.gl_map_left_border - layout_.gl_map_right_border;
    float map_height = v.h - layout_.top_panel_size_in_window -
        layout_.gl_map_top_border - layout_.gl_map_bottom_border;
    layout_.num_map_cell_cols = floor( map_width / layout_.tile_size_in_window);
    layout_.num_map_cell_rows = floor( map_height /layout_.tile_size_in_window);

    // select center cell
    int cx = layout_.num_map_cell_cols/2;
    int cy = layout_.num_map_cell_rows/2;

    // compute tile limits
    display_.min_tile_col_id = pnt.tx - cx;
    display_.min_tile_row_id = pnt.ty - cy;
    display_.max_tile_col_id = pnt.tx + (layout_.num_map_cell_cols - cx - 1);
    display_.max_tile_row_id = pnt.ty + (layout_.num_map_cell_rows - cy - 1);

  }

  ///
  /// \brief TileView::_draw_grid
  ///
  void TileView::_draw_grid(float cell_size_scale,
                            float line_width,
                            SceneGraph::GLColor color,
                            bool draw_vertices,
                            bool draw_cell_size)
  {

    if (tiles_.size() == 0) {
      return;
    }

    GLfloat x_start = layout_.gl_map_left;
    GLfloat y_start = layout_.gl_map_top;
    GLfloat x_end   = x_start + layout_.num_map_cell_cols * layout_.tile_size_in_window;
    GLfloat y_end   = y_start + layout_.num_map_cell_rows * layout_.tile_size_in_window;

    glLineWidth(line_width);
    color.Apply();

    GLfloat cell_size = layout_.tile_size_in_window * cell_size_scale;

    // draw grid
    for (GLfloat y = y_start; y <= y_end + 1; y += cell_size) {
      pangolin::glDrawLine(x_start, y, x_end, y);
    }

    for (GLfloat x = x_start; x <= x_end + 1; x += cell_size) {
      pangolin::glDrawLine(x, y_start, x, y_end);
    }

    if (draw_cell_size) {
      pangolin::glDrawLine(x_start, y_end + 10, x_start + cell_size, y_end + 10);
      pangolin::glDrawLine(x_start, y_end + 5, x_start, y_end + 15);
      pangolin::glDrawLine(x_start + cell_size, y_end + 5 , x_start + cell_size, y_end + 15);
      char buffer[20];
      sprintf(buffer, "%0.1f m", layout_.tile_pixel_size_in_meters * 255);
      text_.Draw(buffer, x_start + 5, y_end + 30);
    }

    // draw vertices
    if (draw_vertices) {
      GLfloat rad = 3;
      for (GLfloat y = y_start; y <= y_end + 1; y += cell_size) {
        for (GLfloat x = x_start; x <= x_end + 1; x += cell_size) {
          pangolin::glDrawRectPerimeter(x-rad,y-rad,x+rad,y+rad);
        }
      }
    }
  }

  ///
  /// \brief TileView::_drawGPSPoints
  ///
  void TileView::_draw_gps_trajectory()
  {
    if(gps_points_.size() == 0) {
      return;
    }

    pangolin::GlState state;
    state.glDisable(GL_LIGHTING);

    double x1,x2,y1,y2;

    state.glLineWidth(2);
    glColor3f(0.0,1.0,1.0);

    GLfloat tsize = layout_.tile_size_in_window;
    GLfloat top = layout_.gl_map_top;
    GLfloat left = layout_.gl_map_left;
    GLfloat right = left + layout_.num_map_cell_cols * tsize;
    GLfloat bottom = top + layout_.num_map_cell_rows * tsize;

    // draw trajectory
    state.glPointSize(2);
    for (unsigned int ii=1; ii < gps_points_.size(); ii++) {
      int xoffset = gps_points_[ii-1].tx - display_.min_tile_col_id;
      int yoffset = gps_points_[ii-1].ty - display_.min_tile_row_id;
      x1 = left + (xoffset + gps_points_[ii-1].xpos) * tsize;
      y1 = top + (yoffset + gps_points_[ii-1].ypos) * tsize;
      xoffset = gps_points_[ii].tx - display_.min_tile_col_id;
      yoffset = gps_points_[ii].ty - display_.min_tile_row_id;
      x2 = left + (xoffset + gps_points_[ii].xpos) * tsize;
      y2 = top + (yoffset + gps_points_[ii].ypos) * tsize;

      // don't draw the line if a point falls outside the grid
      if(x1 < left || x2 < left || x1 > right || x2 > right ||
         y1 < top  || y2 < top  || y1 > bottom || y2 > bottom ) {
        continue;
      }
      pangolin::glDrawLine(x1, y1, 0, x2, y2, 0);
      glBegin(GL_POINTS);
      glVertex2d(x1,y1);
      glEnd();
    }

    // draw the last point
    state.glPointSize(3);
    glColor3f(255.0/255.0,106.0/255.0,74.0/255.0);
    int xoffset = gps_points_.back().tx - display_.min_tile_col_id;
    int yoffset = gps_points_.back().ty - display_.min_tile_row_id;
    x1 = left + (xoffset + gps_points_.back().xpos) * tsize;
    y1 = top + (yoffset + gps_points_.back().ypos) * tsize;
    glBegin(GL_POINTS);
    glVertex2d(x1,y1);
    glEnd();
    gui::drawCircle(x1,y1,display_.current_position_rad,
                    display_.current_position_rad);
    display_.current_position_rad += display_.current_position_rad_step;
    if(display_.current_position_rad > display_.current_position_max_rad) {
      display_.current_position_rad = display_.current_position_min_rad;
    }

    glColor3f(1.0,1.0,1.0);
  }

  ///
  /// \brief TileView::_shift_grid_if_at_the_border
  ///
  void TileView::_shift_grid_if_at_the_border()
  {
    // Check if the last GPS point is near the border of the grid. If so,
    // move the grid to keep the current position visible.
    if(!gps_points_.size()) {
      return;
    }

    osm::GPSPoint pnt = gps_points_.back();
    int col_shift = 0;
    int row_shift = 0;

    // left - right shift
    if (pnt.tx == display_.min_tile_col_id && pnt.xpos < 0.25) {
      col_shift = -1;
    } else if (pnt.tx == display_.max_tile_col_id && pnt.xpos > 0.75) {
      col_shift = 1;
    }

    // top - bottom shift
    if (pnt.ty == display_.min_tile_row_id && pnt.ypos < 0.25) {
      row_shift = -1;
    } else if (pnt.ty == display_.max_tile_row_id && pnt.ypos > 0.75) {
      row_shift = 1;
    }

    // update limits
    display_.min_tile_col_id += col_shift;
    display_.max_tile_col_id += col_shift;
    display_.min_tile_row_id += row_shift;
    display_.max_tile_row_id += row_shift;
  }

  ///
  /// \brief TileView::_gps_to_cart.  Compute gps point tile coordinate
  ///        and relative position
  /// \param[in/out] pnt
  /// \param[in] zoom
  ///
  void TileView::_gps_to_cartesian(osm::GPSPoint &pnt, int zoom)
  {
    int shift = 1 << zoom;
    double yoffset = (8.0/(1 << (18 - zoom)))/256.0;
    double lat   = pnt.lat;
    double lon   = pnt.lon;
    double aux   = log( tan( lat * M_PI / 180.0 ) + 1.0/cos( lat * M_PI / 180.0 ) );
    double xpos  = (( lon + 180.0 ) / 360.0) * shift;
    double ypos  = (( 1.0 - aux / M_PI ) / 2.0) * shift + yoffset;

    pnt.tx   = (int)xpos;
    pnt.ty   = (int)ypos;

    // compute location of point in the tile
    pnt.xpos = xpos - pnt.tx;
    pnt.ypos = ypos - pnt.ty;
  }

  ///
  /// \brief TileView::_get_tile_and_neighbours
  /// \param tile_x
  /// \param tile_y
  ///
  void TileView::_get_tile_and_neighbours(const osm::GPSPoint& pnt)
  {
    osm::GPSPoint aux;
    gui::GLTile* tile;

    // get tile and neighbours
    bool found;
    int xtile_tmp, ytile_tmp, idx;
    int rad = display_.tile_neighbours_distance;

    for (int ii=-rad; ii <= rad; ++ii) {
      for (int jj=-rad; jj <= rad; ++jj) {
        found = false;
        xtile_tmp = pnt.tx + jj;
        ytile_tmp = pnt.ty + ii;
        for (unsigned int kk=0; kk < tiles_.size(); ++kk) {
          if (xtile_tmp == tiles_[kk]->col && ytile_tmp == tiles_[kk]->row) {
            // we have the tile, modify elevation and visible surface
            found = true;
            idx    = kk;
            break;
          }
        }

        if (found) {
          // we have the tile, just update it
          tiles_[idx]->Update(pnt.xpos-jj, pnt.ypos-ii, -pnt.z);
        } else {
          // we don't have the tile ->get it
          aux.tx = xtile_tmp;
          aux.ty = ytile_tmp;
          tile = new gui::GLTile();
          _get_tile(aux, tile);
          tile->Update(pnt.xpos-jj, pnt.ypos-ii, -pnt.z);
          tiles_.push_back(tile);
        }
      }
    }
  }

  ///
  /// \brief TileView::_get_tile
  /// \param pnt
  /// \param tile
  ///
  void TileView::_get_tile(osm::GPSPoint& pnt, gui::GLTile* tile)
  {
    cv::Mat img(256,256,CV_8UC3);
    std::vector<char> tile_buffer;
    osm::FetchTile(pnt.tx, pnt.ty, zoom_,src_, tile_buffer);
    img = cv::imdecode(cv::Mat(tile_buffer),1);
    tile_buffer.clear();

    // first init tile info
    tile->col        = pnt.tx;
    tile->row        = pnt.ty;
    tile->zoom       = zoom_;
    tile->resolution = layout_.tile_size_in_window / 255;
    tile->east       = (float)osm::TileXToLongitude(pnt.tx+1,zoom_);
    tile->west       = (float)osm::TileXToLongitude(pnt.tx,zoom_);
    tile->north      = (float)osm::TileYToLatitude(pnt.ty,zoom_);
    tile->south      = (float)osm::TileYToLatitude(pnt.ty+1,zoom_);

    tile->Init(img);
  }


  ///
  /// \brief TileView::_download_osm_data
  /// \param[in] dWest
  /// \param[in] dSouth
  /// \param[in] dEast
  /// \param[in] dNorth
  /// \param[out] ways
  ///
  void  TileView::_download_osm_data(const double west,
                                     const double south,
                                     const double east,
                                     const double north,
                                     std::vector<osm::Way>& ways_out)
  {
    //    std::vector<osm::node> nodes;
    //    std::vector<osm::way> ways;
    //    std::vector<osm::relation> relations;

    //    char buffer[100];
    //    sprintf(buffer,"http://api.openstreetmap.org/api/0.6/map?bbox=%f,%f,%f,%f",
    //            west, south, east, north);

    //    std::string url(buffer);

    //    osm::FetchOpenStreetMapData(url, nodes, ways, relations);
    //    osm::ParseOpenStreetMapData(origin_, resolution_, zoom_,
    //                                nodes, ways, relations, ways_out);

    //  _check_ways_ids(ways_ids_,ways_out);
    //  _convertWaysGPSToUTM(ways_out);
  }

  ///
  /// \brief TileView::_convert_ways_gps_to_utm
  /// \param[in/out] ways
  ///
  void TileView::_convert_ways_gps_to_utm(std::vector<osm::Way>& ways)
  {
    //    double x,y;
    //    double ox = origin_.tx + origin_.xpos;
    //    double oy = origin_.ty + origin_.ypos;
    //    double tile_size   = resolution_ * 255;

    //    for (unsigned int ii=0; ii < ways.size(); ++ii) {
    //      for (unsigned int jj=0; jj < ways[ii].waypoints.size(); ++jj) {
    //        x = (osm::LongitudeToTileX(ways[ii].waypoints[jj].lon,zoom_) - ox)*tile_size;
    //        y = (oy - osm::LatitudeToTileY(ways[ii].waypoints[jj].lat,zoom_) )*tile_size;
    //        ways[ii].waypoints[jj].lon =  y;
    //        ways[ii].waypoints[jj].lat =  x;
    //      }
    //    }
  }

  ///
  /// \brief TileView::_check_ways_ids
  /// \param ways_ids
  /// \param ways
  ///
  void TileView::_check_ways_ids(std::vector<int> ways_ids,
                                 std::vector<osm::Way>& ways)
  {

    //    std::vector<osm::Way>::iterator it_way;
    //    std::vector<int>::iterator      it_id;

    //    for ( it_way = ways.begin(); it_way != ways.end(); ++it_way) {
    //      it_id = std::find(ways_ids.begin(), ways_ids.end(), it_way->wayID);
    //      if (it_id == ways_ids.end()) {
    //        ways_ids.push_back(it_way->wayID);
    //      } else {
    //        it_way = ways.erase(it_way);
    //      }
    //    }
  }

  ///
  /// \brief TileView::_read_gps_file. The file is a two column file
  ///        containing latitude and longitude data
  /// \param[in] filename
  ///
  void TileView::_read_gps_file(const std::string filename)
  {

    std::ifstream fin;

    fin.open(filename.c_str());

    osm::GPSPoint pnt;
    pnt.lon = 0.0;
    pnt.lat = 0.0;

    double lon,lat;

    while (!fin.eof()) {

      fin >> lat >> lon;

      if(lat < -180.0 || lat > 180.0 || lon < -180.0 || lon > 180.0){
        //std::cerr << "Wrong GPS coordinate! =>  lat: " << lat << " lon: "
        //          << lon << std::endl;
        exit(1);
      }

      if(lat != pnt.lat || lon != pnt.lon){
        pnt.lat = lat;
        pnt.lon = lon;
        gps_points_.push_back(pnt);
      }

    }

    fin.close();

    //std::cout << "Loaded " << gps_points_.size() << " points " << std::endl;
  }

}
