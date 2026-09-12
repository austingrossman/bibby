#pragma once
#include <stdbool.h>
#include <stddef.h>

#include "http.h"

// Log-file browsing for the web interface. Everything here is read-only and
// confined to the CSV tree the logger writes (~/bibby/logs): client-supplied
// paths are resolved and checked against that root before anything is opened,
// so the web server can never be talked into serving a file elsewhere.

// Absolute path of the log root.
const char *web_logs_root(void);

// JSON array of every .csv under the root, newest first. `active_path` (the
// run currently being logged, or NULL) is flagged so the page can label it.
void web_logs_list_json(Buf *out, const char *active_path);

// Resolve a client-supplied relative path to an absolute file inside the log
// root. False if it escapes the root, is not a regular .csv file, or is absent.
bool web_logs_resolve(const char *rel_path, char *out, size_t out_size);

// Decimated series JSON for one log, at most `max_points` points per series
// (a whole-file streaming average, so a two-hour high-rate log plots in about a
// second instead of shipping tens of megabytes to the browser).
bool web_logs_series_json(Buf *out, const char *rel_path, int max_points);
