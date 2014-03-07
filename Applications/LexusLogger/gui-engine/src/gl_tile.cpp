
#include <gl_tile.h>

namespace gui {

  ///
  /// \brief GLTile::GLTile
  ///
  GLTile::GLTile():
    width(0),
    height(0),
    tile_vertices_(nullptr),
    tile_colors_(nullptr),
    tile_alpha_(nullptr),
    is_initialized_(false),
    use_fog_effect_(false),
    use_height_effect_(false),
    use_grid_effect_(false)
  {}

  ///
  /// \brief GLTile::GLTile
  /// \param[in] tile
  ///
  GLTile::GLTile(const GLTile& tile):
    width(tile.width),
    height(tile.height),
    east(tile.east),
    west(tile.west),
    north(tile.north),
    south(tile.south),
    col(tile.col),
    row(tile.row),
    zoom(tile.zoom),
    resolution(tile.resolution),
    is_initialized_(false),
    use_fog_effect_(tile.use_fog_effect_),
    use_height_effect_(tile.use_height_effect_),
    use_grid_effect_(tile.use_grid_effect_)
  {

    if (width > 0 && height > 0){
      tile_vertices_ = new float[width*height*3];
      tile_colors_   = new float[width*height*3];
      tile_alpha_    = new float[width*height];
      tile.CopyTo(tile_vertices_,tile_colors_,tile_alpha_);
      is_initialized_ = true;
    } else {
      tile_vertices_ = nullptr;
      tile_colors_ = nullptr;
      tile_alpha_ = nullptr;
    }
  }

  ///
  /// \brief GLTile::~GLTile
  ///
  GLTile::~GLTile()
  {
    Clear();
  }

  ///
  /// \brief GLTile::Clear
  ///
  void GLTile::Clear()
  {
    if(tile_vertices_)  delete tile_vertices_;
    if(tile_colors_)    delete tile_colors_;
    if(tile_alpha_)     delete tile_alpha_;
  }

  ///
  /// \brief GLTile::CopyTo
  /// \param[out] vertices
  /// \param[out] colors
  /// \param[out] alpha
  ///
  void GLTile::CopyTo(float* vertices, float* colors, float* alpha) const
  {
    int size = width*height;

    for (int i=0; i < size; ++i) {
      alpha[i] = tile_alpha_[i];
    }

    for (int i=0; i < size*3; ++i){
      vertices[i] = tile_vertices_[i];
      colors[i]   = tile_colors_[i];
    }
  }

  ///
  /// \brief GLTile::SetEffects
  /// \param[in] bFogEffect
  /// \param[in] bHeightEffect
  /// \param[in] bGridEffect
  ///
  void GLTile::SetEffects(const bool bFogEffect,
                           const bool bHeightEffect,
                           const bool bGridEffect)
  {
    use_fog_effect_    = bFogEffect;
    use_height_effect_ = bHeightEffect;
    use_grid_effect_   = bGridEffect;
  }

  ///
  /// \brief GLTile::Init
  /// \param[in] img
  ///
  void GLTile::Init(cv::Mat& img)
  {
    if (width != img.cols || height != img.rows) {
      Clear();

      width  = img.cols;
      height = img.rows;

      tile_vertices_ = new float[width*height*3];
      tile_colors_   = new float[width*height*3];
      tile_alpha_    = new float[width*height];
    }

    int idx;

    cv::Vec3b* im_row;

    for(int i=0; i<height; i++){
      im_row = img.ptr<cv::Vec3b>(i);
      for(int j=0; j < width; j++){
        // compute index
        idx = 3*(i*width + j);
        // compute color
        tile_colors_[idx]   = (float)im_row[j][0]/255.0;
        tile_colors_[idx+1] = (float)im_row[j][1]/255.0;
        tile_colors_[idx+2] = (float)im_row[j][2]/255.0;
        // compute vertices
        tile_vertices_[idx]   = resolution*j;
        tile_vertices_[idx+1] = resolution*i;
        tile_vertices_[idx+2] = 0.0;                // height
        // alpha
        idx /= 3;
        tile_alpha_[idx] = (use_fog_effect_)?0.0:0.7;
      }
    }

//    gl_tex_.Reinitialise(img.cols, img.rows,
//                           GL_RGB, true, 0,
//                           GL_RGB, GL_UNSIGNED_BYTE, 0);

//    gl_tex_.Upload(img.data, GL_RGB, GL_UNSIGNED_BYTE);

    is_initialized_ = true;
  }

  ///
  /// \brief GLTile::Update
  /// \param[in] x
  /// \param[in] y
  /// \param[in] h
  ///
  void GLTile::Update(const float x, const float y, const float h)
  {
    // update height map and visible surface
    int col = (int)(x*width);
    int row = (int)(y*height);

    // apply gaussian mask arround
    int idx, irow, icol;
    float weightF, weightH;
    float radm    = 30.0;                       // meters
    int   rad     = (int)( radm / resolution);  // grid points
    float sigmaH  = 10 / resolution;            // std for height effect
    float sigmaF  = 15 / resolution;            // std for fog effect
    float sigmaH2 = 2*(sigmaH*sigmaH);
    float sigmaF2 = 2*(sigmaF*sigmaF);

    for (int i=-rad; i<=rad; ++i){
      irow = row+i;

      if (irow < 0) {
        continue;
      }
      if (irow >= height) {
        break;
      }

      for (int j=-rad; j<=rad; j++) {
        icol = col + j;
        if(icol < 0 || icol >= width) {
          continue;
        }

        // compute gaussian weight;
        weightF = exp(-(i*i + j*j)/sigmaF2 );
        weightH = exp(-(i*i + j*j)/sigmaH2 );

        // compute idx
        idx = (irow*width + icol);

        if (use_fog_effect_ && weightF*0.7 > tile_alpha_[idx]) {
          tile_alpha_[idx] = weightF*0.7;
        }

        if (use_height_effect_) {
          tile_vertices_[idx*3+2] =
              (1-weightH)*tile_vertices_[idx*3+2] + weightH*h;
        }
      }
    }

  }

  ///
  /// \brief GLTile::UpdateVertices
  ///
  void GLTile::UpdateVertices()
  {
    for(int i=0; i<height; i++){
      for(int j=0; j < width; j++){
        // compute index
        int idx = 3*(i*width + j);
        // compute vertices
        tile_vertices_[idx]   = resolution*j;
        tile_vertices_[idx+1] = resolution*i;
      }
    }
  }

  ///
  /// \brief GLTile::Draw
  ///
  void GLTile::Draw()
  {
    pangolin::GlState state;

    state.glDisable(GL_LIGHTING);
    state.glEnable( GL_BLEND );

    if (use_grid_effect_) {
      glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
    }

    // generate n-1 strips
    int   idx;
    float alpha;

    for (int i = 0 ; i < height-1; i++) {
      glBegin(GL_TRIANGLE_STRIP);
      for (int j = 0;j < width; j++) {

        idx = i*width + j;
        alpha = tile_alpha_[idx];
        idx *= 3;

        glColor4f( tile_colors_[idx],
                   tile_colors_[idx+1],
            tile_colors_[idx+2],alpha);

        glVertex3f( tile_vertices_[idx],
                    tile_vertices_[idx+1],
            tile_vertices_[idx+2]);

        idx   = (i+1)*width + j;
        alpha = tile_alpha_[idx];
        idx  *= 3;

        glColor4f(tile_colors_[idx],
                  tile_colors_[idx+1],
            tile_colors_[idx+2],alpha);

        glVertex3f(tile_vertices_[idx],
                   tile_vertices_[idx+1],
            tile_vertices_[idx+2]);

      }
      glEnd();
    }

    if(use_grid_effect_) {
      glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
    }

  }
}
