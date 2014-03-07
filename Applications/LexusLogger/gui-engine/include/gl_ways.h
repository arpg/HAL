#pragma once

#include <pangolin/pangolin.h>
#include <OpenStreetMap.h>

namespace gui {

  /////////////////////////////////////////////////////////////////////////////
  // Code to draw roads
  class GLWays
  {
  public:
    GLWays():
      gl_init_complete_(false) {}

    ~GLWays(){}

    void Init(std::vector< osm::Way > ways);

    void Clear();

    void Draw();

    void DrawWay(osm::Way way);

    unsigned int Size();

    void Update(int tile_row, int tile_col);

    void Add(std::vector<osm::Way> ways);


  private:
    bool _gl_init();
    void _destroy_gl();

  private:
    GLuint                draw_list_id_;
    bool                  gl_init_complete_;
    std::vector<osm::Way> ways_;
    std::vector<int>      ways_ids_;
  };

}

