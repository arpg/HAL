#pragma once

typedef struct image_t {
  unsigned int texture_id;

  int framecount;
  double timestamp;
  int width, height;
  unsigned char *buffer;
} image_t;
