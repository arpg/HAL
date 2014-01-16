#pragma once

#include "./Image.h"
struct camera_t;

struct camera_t *camera_alloc(int id);

void camera_play(camera_t *camera);

void camera_lock_buffer(camera_t *camera,
                        image_t *buffer);

void camera_unlock_buffer(camera_t *camera,
                          image_t *buffer);

void camera_stop(camera_t *camera);

void camera_free(camera_t *camera);
