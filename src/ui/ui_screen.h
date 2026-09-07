#pragma once

#include "../config.h"
#include "../state.h"

// Build the main bibby screen (widgets, styles, chart) and start its periodic
// refresh timer. Called once by ui_run after the display is up.
void ui_screen_create(BibbyState *st, const BibbyConfig *cfg);
