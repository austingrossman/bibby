// LVGL configuration for bibby. Only deviations from the LVGL defaults are
// listed; lv_conf_internal.h supplies everything else.
#ifndef LV_CONF_H
#define LV_CONF_H

#define LV_COLOR_DEPTH 32

// Linux backends: fbdev display (the DSI panel via /dev/fb0; its flush path
// software-rotates for the sideways mount) + evdev touch (Goodix).
#define LV_USE_LINUX_FBDEV 1
#define LV_USE_EVDEV       1

// Fonts. Sizes used by the bibby UI; Montserrat is built into LVGL.
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_16 1
#define LV_FONT_MONTSERRAT_20 1
#define LV_FONT_MONTSERRAT_24 1
#define LV_FONT_MONTSERRAT_32 1
#define LV_FONT_MONTSERRAT_48 1
#define LV_FONT_DEFAULT &lv_font_montserrat_16

// Memory. LVGL's built-in allocator is a fixed pool sized for MCU targets
// (64 KB by default); this screen — two multi-series charts, their legend
// chips, the kettle — sits at ~36 KB of it steady state and peaks near 42 KB,
// so one large redraw could exhaust the rest and assert inside
// lv_draw_add_task. On Linux there is no reason for a ceiling: take the
// allocator from libc and let the heap grow.
#define LV_USE_STDLIB_MALLOC LV_STDLIB_CLIB

// An LVGL assertion must never spin with the elements live: the default
// handler is `while(1);`, which wedges the UI thread at 100% CPU, leaves the
// panel unresponsive, and swallows SIGTERM. Abort instead — the kernel drops
// the GPIO requests, the SoC pull-downs hold the SSR inputs low, and
// bibby.service restarts the process.
#define LV_ASSERT_HANDLER_INCLUDE <stdlib.h>
#define LV_ASSERT_HANDLER abort();

#define LV_USE_LOG 1
#define LV_LOG_LEVEL LV_LOG_LEVEL_WARN
#define LV_LOG_PRINTF 1

#endif // LV_CONF_H
