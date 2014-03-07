#include <gl_ways.h>

namespace gui {

  ///
  /// \brief GLWays::draw
  ///
  void GLWays::Draw()
  {

    if ( gl_init_complete_ == false ){

      if( _gl_init() ){

        return;
      }
    }

    glPushAttrib(GL_ENABLE_BIT);

    glDisable( GL_LIGHTING );
    glEnable( GL_DEPTH_TEST );

    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);

    glLineWidth(1);

    glEnable(GL_LINE_SMOOTH);
    glLineWidth(1);

    for(unsigned int ii=0; ii<ways_.size(); ii++){
      DrawWay(ways_[ii]);
    }

    glPopAttrib();
  }

  ///
  /// \brief GLWays::Clear
  ///
  void GLWays::Clear() {
    ways_.clear();
  }

  ///
  /// \brief GLWays::Size
  /// \return
  ///
  unsigned int GLWays::Size() {
    return ways_.size();
  }

  ///
  /// \brief GLWays::Update
  /// \param[in] tile_row
  /// \param[in] tile_col
  ///
  void GLWays::Update(int tile_row, int tile_col)
  {
    //a loop over all of the ways to mark the active tiles and nodes
    for (unsigned int ii = 0; ii < ways_.size(); ++ii) {
      for (unsigned int jj = 0; jj < ways_[ii].tiles.size(); ++jj) {
        if ((ways_[ii].tiles[jj](0) ==  tile_row) &&
            (ways_[ii].tiles[jj](1) ==  tile_col) &&
            (ways_[ii].tiles[jj](2) == 0)) {
          ways_[ii].tiles[jj](2) = 1 ; //active tile or already passed.
          for (unsigned int kk = 0; kk < ways_[ii].waypoints.size(); ++kk) {
            if(ways_[ii].waypoints[kk].tx == tile_row &&
               ways_[ii].waypoints[kk].ty == tile_col) {
              ways_[ii].waypoints[kk].active = true;
            }
          }
        }
      }
    }
  }

  ///
  /// \brief GLWays::Add
  /// \param[in] ways
  ///
  void GLWays::Add(std::vector<osm::Way> ways){

    int flag = 0;
    for (unsigned int ii = 0; ii < ways.size(); ++ii) {
      for (unsigned int jj = 0; jj < ways_ids_.size(); ++jj) {
        if (ways[ii].wayID == ways_ids_[jj]) {
          flag = 1;
        }
      }
      if(flag == 0) {
        ways_.push_back(ways[ii]);
      }
      flag = 0;
    }
  }

  ///
  /// \brief GLWays::DrawWay
  /// \param[in] way
  ///
  void GLWays::DrawWay(osm::Way way)
  {

    //Color Map for different tags
    switch(way.type) {
      case 0: glColor3ub( 255,000,051 ); break;
      case 1: glColor3ub( 000,204,225 ); break;
      case 2: glColor3ub( 000,102,204 ); break;
      case 3: glColor3ub( 255,255,000 ); break;
      default:  glColor3ub( 102,000,255 );
    }

    glBegin( GL_LINE_STRIP );
    for (unsigned  int jj = 0; jj < way.waypoints.size(); ++jj ) {
      if (way.waypoints[jj].active == true) {
        glVertex3d( way.waypoints[jj].y, way.waypoints[jj].x, 0 );
      }
    }
    glEnd();

    glPointSize(3.5f);

    glBegin( GL_POINTS );
    for (unsigned  int jj = 0; jj <  way.waypoints.size(); ++jj) {
      if (way.waypoints[jj].active == true) {
        glVertex3d(  way.waypoints[jj].y, way.waypoints[jj].x, 0 );
      }
    }
    glEnd();

  }

  ///
  /// \brief GLWays::Init
  /// \param ways
  ///
  void GLWays::Init( std::vector< osm::Way > ways )
  {
    ways_ = ways;

  }

  ///
  /// \brief GLWays::_gl_init
  /// \return
  ///
  bool GLWays::_gl_init()
  {
    draw_list_id_ = glGenLists(1);

    // compile drawlist for a pose
    glNewList( draw_list_id_, GL_COMPILE );

    glDepthMask( GL_TRUE );

    glEnable(GL_DEPTH_TEST);

    glEnable( GL_BLEND );
    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

    glBegin( GL_LINES );

    glColor4f( 1.0, 0, 0, 0.5 );
    glVertex3f( 0.0, 0.0, 0.0 );
    glVertex3f( 1.0, 0.0, 0.0 );

    glColor4f( 0, 0, 1.0, 0.5 );
    glVertex3f( 0.0, 0.0, 0.0 );
    glVertex3f( 0.0, 0.0, 1.0 );

    glColor4f( 0, 1.0, 0, 0.5 );
    glVertex3f( 0.0, 0.0, 0.0 );
    glVertex3f( 0.0, 1.0, 0.0 );

    glEnd();
    glEndList();
    gl_init_complete_ = true;
    return true;
  }

  ///
  /// \brief GLWays::destroy
  ///
  void GLWays::_destroy_gl()
  {
    // bad idea? needs to happen in the GLThread...
    glDeleteLists( draw_list_id_, 1 );
  }

}

