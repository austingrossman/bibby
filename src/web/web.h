#pragma once
#include <stdbool.h>

#include "../config.h"
#include "../state.h"

// Optional LAN web interface: an HTTP mirror of the touchscreen plus a browser
// for the CSV run logs. It is off unless [web] enable is set AND a password is
// configured, and it owns no control or safety logic of its own — remote taps
// are injected into LVGL as pointer events, so they reach the controller only
// through the same widgets and interlocks a finger at the panel does.
//
// active_log_path is the file the CSV logger opened for this run (may be NULL),
// used only to flag the live log in the listing. Returns 0 if the server is
// listening, -1 if it is disabled or could not bind.
int  web_start(BibbyState *st, const BibbyConfig *cfg, const char *active_log_path);
void web_stop(void);
