#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Screen mirror: the same pixels the panel is showing, plus a path for pointer
// events to travel the other way.
//
// The mirror reads /dev/fb0 directly, so the web page shows exactly what the
// operator at the kettle sees — including anything the UI draws that no JSON
// API knows about. Injected pointer events are handed to LVGL through a queue
// drained by a second pointer indev on the UI thread, in panel (framebuffer)
// coordinates, exactly like the evdev touchscreen: LVGL applies the display
// rotation to both, so a click on the web image lands where it looks like it
// lands. The web path therefore reaches the controller only through the same
// widgets a finger does, and inherits every interlock the UI already has.

// Open the framebuffer and start the capture thread. `rotation` is ui.rotation:
// the mirror undoes it so the browser sees the panel the way the operator does.
// Returns 0 on success.
int  web_screen_start(const char *fb_path, int fps, int rotation);
void web_screen_stop(void);

// True once the framebuffer is mapped (the UI may have failed to start).
bool web_screen_available(void);

// Size of the mirrored image (panel size with ui.rotation applied), which is
// also the coordinate space web_screen_queue_pointer() expects.
void web_screen_size(int *w, int *h);

// Block until a frame newer than `since_seq` exists, at most timeout_s. On
// success returns a malloc'd PNG the caller frees. False on timeout (screen
// unchanged) or if the mirror is unavailable. `scale` is 1 or 2 (2 halves each
// dimension with a box filter, for slow links).
bool web_screen_png(uint32_t since_seq, int scale, double timeout_s,
                    unsigned char **png, size_t *len, uint32_t *seq);

// ── Pointer injection (called from web threads) ──────────────────────────────
// Queue one pointer sample in mirrored-image coordinates. pressed=false is a
// release.
void web_screen_queue_pointer(int x, int y, bool pressed);

// True once the LVGL indev exists — i.e. remote taps will actually land. False
// when the UI never started (headless fallback) or control is disabled.
bool web_screen_input_ready(void);

// Create the LVGL indev that drains the queue. MUST run on the UI thread after
// the display exists; this is the only place web input touches LVGL.
void web_screen_attach_indev(void);
