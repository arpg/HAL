#pragma once

#include <opencv/cv.h>
#include <SceneGraph/SceneGraph.h>

namespace gui {

  /////////////////////////////////////////////////////////////////////////////
  class GLTile
  {
  public:

    // def constructor
    GLTile();

    // copy constructor
    GLTile(const GLTile& tile);

    // def destructor
    ~GLTile();

    // clear allocated memory
    void Clear();

    // initialize tile data
    void Init(cv::Mat& img);

    // draw tile
    void Draw();

    // copy tile data
    void CopyTo(float* vertices, float* colors, float* alpha) const;

    // update height at a specific point
    void Update(const float x, const float y, const float h);

    // recompute rendering vertices
    void UpdateVertices();

    // setters and getters
    bool IsInitialized()       { return is_initialized_; }
    void ToggleGridEffect()   { use_grid_effect_ = !use_grid_effect_; }
    void ToggleHeightEffect() { use_height_effect_ = !use_height_effect_; }
    void ToggleFogEffect()    { use_fog_effect_ = !use_fog_effect_; }

    void SetEffects(const bool bFogEffect,
                     const bool bHeightEffect,
                     const bool bGridEffect);


  public:
    // tile info
    int     col;        // tile col in grid at zoom level
    int     row;        // tile row in grit at zoom level
    int     zoom;       // map zoom level
    int     width;
    int     height;
    float   east;       //east lon
    float   west;       //west lon
    float   north;      //north lat
    float   south;      //south lat
    float   resolution; // pixel size in meters

  private:

    float*  tile_vertices_;
    float*  tile_colors_;
    float*  tile_alpha_;

    // GL
    bool    is_initialized_;
    bool    is_loaded_;

    bool    use_fog_effect_;
    bool    use_height_effect_;
    bool    use_grid_effect_;

    pangolin::GlTexture gl_tex_;
  };

}
