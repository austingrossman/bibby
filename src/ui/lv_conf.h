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

#define LV_USE_LOG 1
#define LV_LOG_LEVEL LV_LOG_LEVEL_WARN
#define LV_LOG_PRINTF 1

#endif // LV_CONF_H
