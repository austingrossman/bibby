#include <cstdio>
#include <cstdlib>
#include <cfloat>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>
#include <gpiod.h>

#include <SDL3/SDL.h>
#include <GLES3/gl3.h>
#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_opengl3.h>

#include "../shared/shm_types.h"
#include "../shared/single_instance.h"
#include "../shared/config.h"
#include "sensors/max31865.h"
#include "temp_filter.h"
#include "pid.h"
#include "csv_logger.h"
#include "display.h"
#include "gui/kettle.h"
#include "gui/temp_history.h"

#ifndef FONT_DIR
  #define FONT_DIR "third_party/imgui/misc/fonts"
#endif

#ifndef BIBBY_PATH
  #define BIBBY_PATH "heater-controller"
#endif

// MAX31865 DRDY input, GPIO 16 (header pin 36), active-low.
static constexpr unsigned int DRDY_GPIO = 16;

// Samples per stage in the temperature smoothing filter.
static constexpr int TEMP_FILTER_WINDOW = 40;

// One scalable font (ImGui 1.92 bakes any size on demand); the body size is the
// default for widgets, med/large are passed explicitly to AddText. Sized from
// the panel height in init_imgui so the layout scales across displays.
static ImFont *g_font     = nullptr;
static float   g_sz_body  = 24.0f;
static float   g_sz_med   = 36.0f;
static float   g_sz_large = 88.0f;

// Live status snapshot handed to the UI each frame.
struct UiStatus {
  float   temp_c;
  bool    out1, out2;
  float   bright1, bright2; // element glow, IIR-smoothed from out1/out2, [0,1]
  bool    watchdog;
  bool    drdy_high;
  uint8_t rtd_fault; // MAX31865 Fault Status register (07h); 0 = no fault
};

// Everything the UI may edit; the caller publishes these after draw_ui returns.
struct UiControls {
  float *duty1, *duty2;
  bool  *test_pressed;
  float *setpoint_c;
  bool  *manual_control;
  bool  *grain_in;
};

static HeaterShm          *shm_connect();
static void                ensure_heater_controller();
static gpiod_line_request *init_drdy_line(gpiod_chip *chip);
static SDL_Window         *create_window(SDL_GLContext *out_ctx, int *out_w, int *out_h);
static void                init_imgui(SDL_Window *window, SDL_GLContext ctx, int portrait_h);
static void                draw_ui(int portrait_w, int portrait_h, const UiStatus &s,
                                   const UiControls &c, const TempHistory &history, double now);

// ----------------------------------------------------------------------------
int main(void) {
  // Refuse to start if another frontend is already running.
  // Lock fd is intentionally left open for the life of the process.
  if (single_instance_lock("/tmp/biab_frontend.lock") < 0) {
    fprintf(stderr, "frontend: another instance is already running\n");
    return 1;
  }

  // Load user configuration (mains, element specs, PID gains) before anything
  // else; the heater-controller reads the same file when we launch it below.
  BibbyConfig cfg;
  if (!config_load(&cfg, nullptr))
    fprintf(stderr, "frontend: config not found, using defaults\n");

  // The heater-controller must be up before we attach to its shared memory.
  ensure_heater_controller();

  SDL_GLContext gl_ctx;
  int display_w, display_h;
  SDL_Window *window = create_window(&gl_ctx, &display_w, &display_h);

  RotatedDisplay display(display_w, display_h);
  init_imgui(window, gl_ctx, display.portrait_h());

  HeaterShm *shm = shm_connect();

  gpiod_chip *drdy_chip = gpiod_chip_open("/dev/gpiochip4");
  gpiod_line_request *drdy_req = init_drdy_line(drdy_chip);

  // Per-unit RTD calibration (Rref ice-point trim + two-point span gain/offset)
  // comes from bibby.ini so recalibrating never needs a rebuild. See README §9.
  Max31865 sensor("/dev/spidev0.0", cfg.sensor_ref_resistor_ohms, drdy_req,
                  cfg.mains_hz, cfg.sensor_temp_cal_gain, cfg.sensor_temp_cal_offset);
  SecondOrderAverage temp_filter(TEMP_FILTER_WINDOW);
  TempHistory temp_history;
  PidController pid(cfg.pid_kp, cfg.pid_ki, cfg.pid_kd);
  CsvLogger logger;
  if (logger.ok()) fprintf(stderr, "frontend: logging to %s\n", logger.path());

  ImGuiIO &io = ImGui::GetIO();
  float duty1 = 0.0f, duty2 = 0.0f;
  float bright1 = 0.0f, bright2 = 0.0f;  // element glow, IIR-smoothed from outputs
  float pid_duty = 0.0f;  // latest PID power command; held between fresh samples
  float temp_c = 0.0f;
  bool  test_pressed   = false;
  float setpoint_c     = 65.0f;
  bool  manual_control = true;  // matches prior behavior: sliders drive the SSRs
  bool  grain_in       = false;
  bool  running        = true;

  while (running) {
    double now = SDL_GetTicks() / 1000.0;

    // --- Input: route touches through the portrait→landscape rotation ---
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
      if (e.type == SDL_EVENT_FINGER_DOWN ||
          e.type == SDL_EVENT_FINGER_UP   ||
          e.type == SDL_EVENT_FINGER_MOTION) {
        io.AddMousePosEvent(display.portrait_w() * (1.0f - e.tfinger.y),
                            display.portrait_h() * e.tfinger.x);
        if (e.type != SDL_EVENT_FINGER_MOTION) {
          io.AddMouseButtonEvent(0, e.type == SDL_EVENT_FINGER_DOWN);
        }
      } else {
        ImGui_ImplSDL3_ProcessEvent(&e);
        if (e.type == SDL_EVENT_QUIT) running = false;
      }
    }

    // --- Sensor: smooth fresh readings, hold the last value otherwise ---
    bool  temp_fresh = false;
    float temp_raw   = sensor.read_temperature(&temp_fresh);
    if (temp_fresh) {
      temp_c   = temp_filter.push(temp_raw);
      temp_history.push(temp_c, now);
      pid_duty = pid.update(temp_c, setpoint_c);  // run the controller on each new sample
    }

    // --- Control mode ---
    uint8_t rtd_fault     = sensor.fault_status();
    bool    temp_reliable = (rtd_fault == 0);
    // Safety (CLAUDE.md): with unreliable temperature only manual control may
    // drive the SSRs. Force manual so the PID path cannot command power.
    if (!temp_reliable) manual_control = true;
    // In auto, the PID drives both SSRs (currently zero). In manual, the sliders
    // do, set inside draw_ui below.
    if (!manual_control) {
      duty1 = pid_duty;
      duty2 = pid_duty;
    }

    // --- Controller status from shared memory ---
    UiStatus status;
    status.temp_c    = temp_c;
    status.out1      = shm->output1;
    status.out2      = shm->output2;
    // Glow ramps toward each output with a one-pole IIR:
    //   bright[n] = damp*output[n] + (1-damp)*bright[n-1].
    constexpr float damp_factor = 0.2f;
    bright1 += damp_factor * ((status.out1 ? 1.0f : 0.0f) - bright1);
    bright2 += damp_factor * ((status.out2 ? 1.0f : 0.0f) - bright2);
    status.bright1   = bright1;
    status.bright2   = bright2;
    status.watchdog  = shm->watchdog_alarm;
    status.drdy_high = gpiod_line_request_get_value(drdy_req, DRDY_GPIO) == GPIOD_LINE_VALUE_ACTIVE;
    status.rtd_fault = rtd_fault;

    // --- Log one row per fresh sample (PID has just run on this sample) ---
    if (temp_fresh && logger.ok()) {
      LogSample row;
      row.t_monotonic_s = now;
      row.temp_raw_c    = temp_raw;
      row.temp_filt_c   = temp_c;
      row.setpoint_c    = setpoint_c;
      row.duty1         = duty1;
      row.duty2         = duty2;
      row.pid           = pid.terms();
      row.manual        = manual_control;
      row.grain_in      = grain_in;
      row.rtd_fault     = rtd_fault;
      row.watchdog      = status.watchdog;
      logger.log(row);
    }

    // --- Render the UI into the portrait FBO ---
    display.begin_frame();
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    io.DisplaySize             = ImVec2((float)display.portrait_w(), (float)display.portrait_h());
    io.DisplayFramebufferScale = ImVec2(1.0f, 1.0f);

    ImGui::NewFrame();
    UiControls ctl{ &duty1, &duty2, &test_pressed, &setpoint_c, &manual_control, &grain_in };
    draw_ui(display.portrait_w(), display.portrait_h(), status, ctl, temp_history, now);
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    display.present();
    SDL_GL_SwapWindow(window);

    // --- Publish control state back to the heater-controller ---
    shm->duty1            = duty1;
    shm->duty2            = duty2;
    shm->simulate_zc      = test_pressed;
    shm->frontend_iteration++;
  }

  gpiod_line_request_release(drdy_req);
  gpiod_chip_close(drdy_chip);

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplSDL3_Shutdown();
  ImGui::DestroyContext();
  SDL_GL_DestroyContext(gl_ctx);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}

// ----------------------------------------------------------------------------
// Shared memory — link to the heater-controller process.
// ----------------------------------------------------------------------------
static HeaterShm *shm_connect() {
  int fd = shm_open(SHM_NAME, O_RDWR, 0666);
  if (fd < 0) {
    perror("shm_open");
    exit(1);
  }
  void *p = mmap(NULL, sizeof(HeaterShm), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  close(fd);
  if (p == MAP_FAILED) {
    perror("mmap");
    exit(1);
  }
  return (HeaterShm *)p;
}

static void sleep_ms(long ms) {
  struct timespec ts;
  ts.tv_sec  = ms / 1000;
  ts.tv_nsec = (ms % 1000) * 1000000L;
  nanosleep(&ts, nullptr);
}

// Launch the heater-controller detached, so it outlives the frontend (the
// controller must stay up at all times). Double-fork + setsid reparents it to
// init and avoids leaving a zombie here. A launch when one is already running
// is harmless: it fails the single-instance lock and exits immediately.
static void spawn_heater_controller(const char *exe_path) {
  pid_t pid = fork();
  if (pid < 0) { perror("fork"); return; }
  if (pid == 0) {
    setsid();
    pid_t pid2 = fork();
    if (pid2 < 0) _exit(127);
    if (pid2 > 0) _exit(0);
    execl(exe_path, exe_path, (char *)nullptr);
    perror("execl heater-controller");
    _exit(127);
  }
  waitpid(pid, nullptr, 0);  // reap the short-lived intermediate child
}

// ----------------------------------------------------------------------------
// Make sure the heater-controller is running before we attach to its shm.
// Detection uses the shm `iteration` counter: if the segment is missing, or the
// counter does not advance over a short window, the controller is not servicing
// it and we (re)launch it, then wait for the segment to appear.
// ----------------------------------------------------------------------------
static void ensure_heater_controller() {
  bool need_spawn = true;
  int  fd = shm_open(SHM_NAME, O_RDWR, 0666);
  if (fd >= 0) {
    HeaterShm *probe = (HeaterShm *)mmap(NULL, sizeof(HeaterShm),
                                         PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (probe != MAP_FAILED) {
      uint32_t before = probe->iteration;
      sleep_ms(150);  // ~9 mains cycles at 60 Hz; long enough to see ZC advance
      need_spawn = (probe->iteration == before);
      munmap(probe, sizeof(HeaterShm));
    }
  }
  if (!need_spawn) return;

  spawn_heater_controller(BIBBY_PATH);
  // Wait for the controller to create the shm segment (up to ~1 s).
  for (int i = 0; i < 100; i++) {
    int f = shm_open(SHM_NAME, O_RDWR, 0666);
    if (f >= 0) { close(f); return; }
    sleep_ms(10);
  }
  fprintf(stderr, "frontend: heater-controller shm did not appear after spawn\n");
}

// ----------------------------------------------------------------------------
// GPIO — request the DRDY line once at startup; it is then level-polled.
// ----------------------------------------------------------------------------
static gpiod_line_request *init_drdy_line(gpiod_chip *chip) {
  gpiod_line_settings *ls = gpiod_line_settings_new();
  gpiod_line_settings_set_direction(ls, GPIOD_LINE_DIRECTION_INPUT);

  gpiod_line_config *lc = gpiod_line_config_new();
  unsigned int offset = DRDY_GPIO;
  gpiod_line_config_add_line_settings(lc, &offset, 1, ls);

  gpiod_request_config *rc = gpiod_request_config_new();
  gpiod_request_config_set_consumer(rc, "frontend-drdy");

  gpiod_line_request *req = gpiod_chip_request_lines(chip, rc, lc);
  gpiod_line_settings_free(ls);
  gpiod_line_config_free(lc);
  gpiod_request_config_free(rc);
  return req;
}

// ----------------------------------------------------------------------------
// Window + GL + ImGui setup.
// ----------------------------------------------------------------------------
static SDL_Window *create_window(SDL_GLContext *out_ctx, int *out_w, int *out_h) {
  SDL_Init(SDL_INIT_VIDEO);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);

  const SDL_DisplayMode *mode = SDL_GetCurrentDisplayMode(SDL_GetPrimaryDisplay());
  *out_w = mode->w;
  *out_h = mode->h;

  SDL_Window *window = SDL_CreateWindow(
    "BIAB Ramp Controller", *out_w, *out_h,
    SDL_WINDOW_OPENGL | SDL_WINDOW_FULLSCREEN);
  *out_ctx = SDL_GL_CreateContext(window);
  SDL_GL_SetSwapInterval(1);
  return window;
}

static void init_imgui(SDL_Window *window, SDL_GLContext ctx, int portrait_h) {
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  ImGui::StyleColorsDark();

  g_sz_body  = portrait_h * 0.030f;
  g_sz_med   = portrait_h * 0.045f;
  g_sz_large = portrait_h * 0.110f;

  ImFontAtlas *fonts = ImGui::GetIO().Fonts;
  g_font = fonts->AddFontFromFileTTF(FONT_DIR "/Roboto-Medium.ttf", g_sz_body);
  if (!g_font) g_font = fonts->AddFontDefault(); // fallback if the TTF is missing
  ImGui::GetStyle().FontSizeBase = g_sz_body;    // default widget text size

  ImGui_ImplSDL3_InitForOpenGL(window, ctx);
  ImGui_ImplOpenGL3_Init("#version 300 es");
}

// ----------------------------------------------------------------------------
// UI helpers
// ----------------------------------------------------------------------------
static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// Draw text horizontally centered on cx, in a specific font/size.
static void draw_centered(ImDrawList *dl, ImFont *font, float fs, float cx, float y,
                          ImU32 col, const char *txt) {
  ImVec2 sz = font->CalcTextSizeA(fs, FLT_MAX, 0.0f, txt);
  dl->AddText(font, fs, ImVec2(cx - sz.x * 0.5f, y), col, txt);
}

// Largest size <= max_size at which `txt` fits within target_w (px).
static float fit_size(ImFont *font, float max_size, float target_w, const char *txt) {
  ImVec2 sz = font->CalcTextSizeA(max_size, FLT_MAX, 0.0f, txt);
  return (sz.x <= target_w || sz.x <= 0.0f) ? max_size : max_size * target_w / sz.x;
}

// Left-aligned body text at an absolute position, via the draw list. Used for
// all labels/status so SetCursorPos is reserved for interactive widgets only
// (a SetCursorPos not followed by an item trips an ImGui boundary assert).
static void draw_text(ImDrawList *dl, float x, float y, ImU32 col, const char *txt) {
  dl->AddText(g_font, g_sz_body, ImVec2(x, y), col, txt);
}

// A big touch toggle (not necessarily square): green when on, grey when off,
// with a centered label that wraps onto a second line at a space to fit the box.
static void toggle_button(ImDrawList *dl, const char *label, bool *v, ImVec2 pos, ImVec2 size) {
  ImGui::SetCursorPos(pos);
  if (ImGui::InvisibleButton(label, size)) *v = !*v;
  const bool  hov = ImGui::IsItemHovered();
  const ImU32 bg  = *v ? (hov ? IM_COL32(70, 175, 86, 255) : IM_COL32(50, 140, 66, 255))
                       : (hov ? IM_COL32(80, 80, 92, 255)  : IM_COL32(58, 58, 68, 255));
  const ImVec2 br    = ImVec2(pos.x + size.x, pos.y + size.y);
  const float  round = (size.x < size.y ? size.x : size.y) * 0.12f;
  dl->AddRectFilled(pos, br, bg, round);
  dl->AddRect(pos, br, IM_COL32(20, 20, 24, 255), round, 0, 1.5f);

  // Split the label onto two lines at the last space, sized to fit the box.
  char l1[24], l2[24];
  l1[0] = l2[0] = '\0';
  const char *sp = nullptr;
  for (const char *p = label; *p; p++) if (*p == ' ') sp = p;
  if (sp) {
    int n1 = (int)(sp - label);
    if (n1 > (int)sizeof(l1) - 1) n1 = (int)sizeof(l1) - 1;
    for (int i = 0; i < n1; i++) l1[i] = label[i];
    l1[n1] = '\0';
    snprintf(l2, sizeof(l2), "%s", sp + 1);
  } else {
    snprintf(l1, sizeof(l1), "%s", label);
  }

  const float cx      = pos.x + size.x * 0.5f;
  const float cy      = pos.y + size.y * 0.5f;
  const float target  = size.x * 0.86f;
  const ImU32 col_txt = IM_COL32(245, 247, 250, 255);
  float fs = fit_size(g_font, g_sz_body, target, l1);
  if (l2[0]) {
    float f2 = fit_size(g_font, g_sz_body, target, l2);
    if (f2 < fs) fs = f2;
    if (fs > size.y * 0.40f) fs = size.y * 0.40f;   // keep two lines inside the box
    draw_centered(dl, g_font, fs, cx, cy - fs - 1.0f, col_txt, l1);
    draw_centered(dl, g_font, fs, cx, cy + 1.0f,      col_txt, l2);
  } else {
    if (fs > size.y * 0.70f) fs = size.y * 0.70f;
    draw_centered(dl, g_font, fs, cx, cy - fs * 0.5f, col_txt, l1);
  }
}

// Six set-point step buttons (+10 +1 +0.1 / -10 -1 -0.1) as a 2x3 grid of squares.
static void draw_adjust_grid(ImVec2 pos, float cell, float gap, float *sp) {
  const char *lab[2][3] = { { "+10", "+1", "+0.1" }, { "-10", "-1", "-0.1" } };
  const float stp[2][3] = { { 10.0f, 1.0f, 0.1f }, { -10.0f, -1.0f, -0.1f } };
  for (int r = 0; r < 2; r++) {
    for (int col = 0; col < 3; col++) {
      ImGui::SetCursorPos(ImVec2(pos.x + col * (cell + gap), pos.y + r * (cell + gap)));
      if (ImGui::Button(lab[r][col], ImVec2(cell, cell))) {
        *sp = clampf(*sp + stp[r][col], 0.0f, 105.0f);
      }
    }
  }
}

// ----------------------------------------------------------------------------
// UI — one frame. Fixed grid: every element is placed at an absolute position,
// so conditional content (alarms, faults) never reflows the layout. The right
// edge holds a short kettle (set point + temperatures over it) with a single
// full-height duty slider to its left; the left side stacks the toggle/step
// buttons over the temperature graph and a fault-status band.
// ----------------------------------------------------------------------------
static void draw_ui(int portrait_w, int portrait_h, const UiStatus &s,
                    const UiControls &c, const TempHistory &history, double now) {
  const ImU32 col_label = IM_COL32(180, 186, 196, 255);
  const ImU32 col_red   = IM_COL32(255, 60, 60, 255);
  const ImU32 col_pink  = IM_COL32(255, 140, 140, 255);
  const ImU32 col_green = IM_COL32(80, 230, 90, 255);
  const ImU32 col_grey  = IM_COL32(150, 150, 150, 255);

  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  ImGui::SetNextWindowPos({ 0, 0 });
  ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize);
  ImGui::Begin("##main", nullptr,
               ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
               ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse |
               ImGuiWindowFlags_NoBringToFrontOnFocus);
  ImDrawList *dl = ImGui::GetWindowDrawList();

  const float W     = (float)portrait_w;
  const float H     = (float)portrait_h;
  const float lineh = ImGui::GetTextLineHeight();
  const float mx    = W * 0.02f;
  const float gap   = W * 0.012f;

  const float right_w = W * 0.34f;            // kettle column width
  const float right_x = W - right_w;          // kettle column left edge
  const float sl_w    = W * 0.07f;            // single duty slider width
  const float sl_gap  = W * 0.018f;
  const float sl_x    = right_x - sl_gap - sl_w;

  // --- Single duty slider: full height, just left of the kettle, drives both
  // SSRs. Editable only in manual mode; in auto the PID owns duty1/duty2 and the
  // disabled bar just displays the commanded value. ---
  {
    const float sl_top = H * 0.06f;
    const float sl_h   = H * 0.86f;
    ImGui::BeginDisabled(!*c.manual_control);
    ImGui::SetCursorPos(ImVec2(sl_x, sl_top));
    ImGui::VSliderFloat("##duty", ImVec2(sl_w, sl_h), c.duty1, 0.0f, 1.0f, "");
    ImGui::EndDisabled();
    if (*c.manual_control) *c.duty2 = *c.duty1;   // one bar commands both SSRs

    const float scx = sl_x + sl_w * 0.5f;
    char pct[16];
    snprintf(pct, sizeof(pct), "%.0f%%", *c.duty1 * 100.0f);
    draw_centered(dl, g_font, g_sz_body, scx, sl_top - lineh - 2.0f, col_label, "SSR");
    draw_centered(dl, g_font, g_sz_med,  scx, sl_top + sl_h + 4.0f,
                  (s.out1 || s.out2) ? col_green : col_grey, pct);
  }

  // --- Kettle column: set point reading + temperatures over a short kettle ---
  {
    char spbuf[24], cbuf[24], fbuf[24];
    snprintf(spbuf, sizeof(spbuf), "Set Pt: %.1f°C", *c.setpoint_c);
    snprintf(cbuf,  sizeof(cbuf),  "%.2f °C", s.temp_c);
    snprintf(fbuf,  sizeof(fbuf),  "%.2f °F", s.temp_c * 9.0f / 5.0f + 32.0f);
    // One size for the set point and both temperatures, fitted to the widest
    // line so nothing overflows the column.
    const float tw  = right_w * 0.96f;
    const float fs  = fit_size(g_font, g_sz_large, tw, "Set Pt: 105.0°C");
    const float cxr = right_x + right_w * 0.5f;

    // Tall kettle: rim near the top (the set point sits above it), body bottom
    // near the very bottom. draw_kettle puts the body bottom at 0.92 of region.
    const float kettle_top = H * 0.03f;
    const float kettle_h   = (H * 0.985f - kettle_top) / 0.92f;
    const float body_top   = kettle_top + 0.10f * kettle_h;
    const float lip_top    = body_top - kettle_h * 0.022f;

    const float sp_y = lip_top - fs - H * 0.008f; // set point above the rim
    const float c_y  = body_top + fs * 0.20f;     // temps overlaid inside the pot
    const float f_y  = c_y + fs;
    const float content_top = f_y + fs + H * 0.012f;

    draw_kettle(dl, ImVec2(right_x, kettle_top), ImVec2(right_w, kettle_h),
                s.bright1, s.bright2, *c.grain_in, content_top);
    draw_centered(dl, g_font, fs, cxr, sp_y, IM_COL32(130, 220, 140, 255), spbuf);
    draw_centered(dl, g_font, fs, cxr, c_y, IM_COL32(255, 255, 255, 255), cbuf);
    draw_centered(dl, g_font, fs, cxr, f_y, IM_COL32(210, 215, 225, 255), fbuf);
  }

  // --- Left zone: small button cluster on top, large graph, alarms at bottom ---
  const float lz_w = sl_x - sl_gap - mx;        // left zone width
  const float y0   = H * 0.03f;

  // Six set-point steppers as a 2x3 grid of squares (side a). To their left, the
  // three toggles in two columns: Test Touch | Grain In on top, Manual Control
  // across the bottom. Each toggle row is one stepper-square tall, so the cluster
  // is two rows high overall.
  const float a      = W * 0.105f;
  const float band_h = 2.0f * a + gap;
  const float tog_w  = lz_w - 3.0f * a - 3.0f * gap; // toggle area, left of steppers
  const float tcw    = (tog_w - gap) * 0.5f;          // two toggle columns
  toggle_button(dl, "Test Touch",     c.test_pressed,   ImVec2(mx, y0),             ImVec2(tcw, a));
  toggle_button(dl, "Grain In",       c.grain_in,       ImVec2(mx + tcw + gap, y0), ImVec2(tcw, a));
  toggle_button(dl, "Manual Control", c.manual_control, ImVec2(mx, y0 + a + gap),   ImVec2(tog_w, a));
  draw_adjust_grid(ImVec2(mx + tog_w + gap, y0), a, gap, c.setpoint_c);

  // Temperature graph fills everything between the buttons and the status line.
  draw_text(dl, mx, y0 + band_h + gap, col_label, "TEMPERATURE  (last 3 min)");
  const float fault_y = H - lineh - H * 0.012f; // lowest status line
  const float g_y0    = y0 + band_h + gap + lineh + 2.0f;
  history.draw_graph(dl, ImVec2(mx, g_y0), ImVec2(lz_w, fault_y - gap - g_y0), now);

  // Fault status hugs the bottom: RTD fault (header + decode) on the lowest
  // lines, watchdog above it. Drawn after the graph so it sits on top.
  {
    if (s.rtd_fault) {
      char rf[24];
      snprintf(rf, sizeof(rf), "RTD FAULT (0x%02X)", s.rtd_fault);
      draw_text(dl, mx, fault_y - lineh * 1.15f, col_red, rf);
      draw_text(dl, mx, fault_y, col_pink, Max31865::decode_fault_status(s.rtd_fault).c_str());
    }
    if (s.watchdog) {
      draw_text(dl, mx, s.rtd_fault ? fault_y - lineh * 2.30f : fault_y, col_red,
                "WATCHDOG ALARM");
    }
  }

  ImGui::End();
  ImGui::PopStyleVar();
}
