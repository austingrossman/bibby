#pragma once

#include <stdbool.h>

#include "lvgl.h"

// Kettle cutaway graphic: steel walls, wort, optional grain bed, and the two
// immersion elements glowing with their delivered power. Drawn on an
// lv_canvas; update repaints only when an input changed.
lv_obj_t *ui_kettle_create(lv_obj_t *parent, int32_t w, int32_t h);

// bright1/bright2 in [0,1] (smoothed element activity).
void ui_kettle_update(float bright1, float bright2, bool grain_in);
