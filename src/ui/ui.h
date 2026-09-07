#pragma once

#include "../config.h"
#include "../state.h"

// Run the touchscreen UI on the calling (main) thread until st->running goes
// false. The UI only reads the state snapshot and writes: setpoint, mode,
// manual power, grain_in, simulate_zc. It owns no control or safety logic.
// Returns 0, or -1 if the display could not be initialized.
int ui_run(BibbyState *st, const BibbyConfig *cfg);

// Same, but with the panel bring-up test screen (rotation + touch check)
// instead of the controller UI. Invoked by `bibby --ui-test`.
int ui_run_mode(BibbyState *st, const BibbyConfig *cfg, bool test_screen);
