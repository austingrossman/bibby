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
#include "sensors/max31865.h"
#include "temp_filter.h"
#include "pid.h"
#include "display.h"
#include "gui/kettle.h"
#include "gui/temp_history.h"

#ifndef FONT_DIR
  #define FONT_DIR "third_party/imgui/misc/fonts"
#endif

#ifndef HEATER_CONTROLLER_PATH
  #define HEATER_CONTROLLER_PATH "heater-controller"
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

  Max31865 sensor("/dev/spidev0.0", 400, drdy_req);
  SecondOrderAverage temp_filter(TEMP_FILTER_WINDOW);
  TempHistory temp_history;
  PidController pid;

  ImGuiIO &io = ImGui::GetIO();
  float duty1 = 0.0f, duty2 = 0.0f;
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
    status.watchdog  = shm->watchdog_alarm;
    status.drdy_high = gpiod_line_request_get_value(drdy_req, DRDY_GPIO) == GPIOD_LINE_VALUE_ACTIVE;
    status.rtd_fault = rtd_fault;

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

  spawn_heater_controller(HEATER_CONTROLLER_PATH);
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

// A large touch toggle that flips *v when tapped; green when on, grey when off.
static void toggle_button(const char *label, bool *v, ImVec2 pos, ImVec2 size) {
  ImGui::SetCursorPos(pos);
  ImGui::PushStyleColor(ImGuiCol_Button,
                        *v ? ImVec4(0.20f, 0.55f, 0.25f, 1) : ImVec4(0.24f, 0.24f, 0.28f, 1));
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
                        *v ? ImVec4(0.25f, 0.65f, 0.30f, 1) : ImVec4(0.32f, 0.32f, 0.36f, 1));
  ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.30f, 0.70f, 0.35f, 1));
  if (ImGui::Button(label, size)) *v = !*v;
  ImGui::PopStyleColor(3);
}

// Set point stepper: a centered value over two rows of coarse/fine +/- buttons.
static void draw_setpoint(ImDrawList *dl, ImVec2 pos, ImVec2 size, float *sp) {
  char v[32];
  snprintf(v, sizeof(v), "%.1f °C", *sp);
  float fs = g_sz_med;
  draw_centered(dl, g_font, fs, pos.x + size.x * 0.5f, pos.y, IM_COL32(130, 220, 140, 255), v);

  const char *lab[2][3] = { { "-10", "-1", "-0.1" }, { "+0.1", "+1", "+10" } };
  const float stp[2][3] = { { -10.0f, -1.0f, -0.1f }, { 0.1f, 1.0f, 10.0f } };

  const float gap   = 6.0f;
  const float val_h = fs * 1.2f;
  const float bw    = (size.x - 2.0f * gap) / 3.0f;
  const float bh    = (size.y - val_h - gap) / 2.0f;
  const float by0   = pos.y + val_h;
  for (int r = 0; r < 2; r++) {
    for (int c = 0; c < 3; c++) {
      ImGui::SetCursorPos(ImVec2(pos.x + c * (bw + gap), by0 + r * (bh + gap)));
      if (ImGui::Button(lab[r][c], ImVec2(bw, bh))) {
        *sp = clampf(*sp + stp[r][c], 0.0f, 105.0f);
      }
    }
  }
}

// ----------------------------------------------------------------------------
// UI — one frame. Fixed grid: every element is placed at an absolute position,
// so conditional content (alarms, faults) never reflows the layout. Right third
// is the kettle; the left two-thirds stack set point, graph, toggles, sliders,
// and a status band.
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

  const float W       = (float)portrait_w;
  const float H       = (float)portrait_h;
  const float lineh   = ImGui::GetTextLineHeight();
  const float right_w = W * 0.34f;
  const float right_x = W - right_w;

  // --- Right third: kettle with the temperature readout overlaid ---
  draw_kettle(dl, ImVec2(right_x, 0), ImVec2(right_w, H), s.out1, s.out2, *c.grain_in);
  {
    char cbuf[24], fbuf[24];
    snprintf(cbuf, sizeof(cbuf), "%.1f °C", s.temp_c);
    snprintf(fbuf, sizeof(fbuf), "%.1f °F", s.temp_c * 9.0f / 5.0f + 32.0f);
    // Fit to the kettle width using a fixed-width template, so the readout stays
    // as large as possible without overflowing and does not jump as digits change.
    const float tw   = right_w * 0.92f;
    const float fs_c = fit_size(g_font, g_sz_large, tw, "188.8 °C");
    const float fs_f = fit_size(g_font, g_sz_med, tw, "188.8 °F");
    const float cxr  = right_x + right_w * 0.5f;
    draw_centered(dl, g_font, fs_c, cxr, H * 0.12f, IM_COL32(255, 255, 255, 255), cbuf);
    draw_centered(dl, g_font, fs_f, cxr, H * 0.12f + fs_c * 0.98f, IM_COL32(210, 215, 225, 255), fbuf);
  }

  // --- Left two-thirds: stacked bands at fixed positions ---
  const float mx   = W * 0.025f;
  const float colx = mx;
  const float colw = right_x - mx * 2.0f;
  const float gap  = H * 0.012f;
  float       y    = H * 0.015f;

  // Set point
  draw_text(dl, colx, y, col_label, "SET POINT");
  y += lineh + 2.0f;
  const float h_set = H * 0.20f;
  draw_setpoint(dl, ImVec2(colx, y), ImVec2(colw, h_set), c.setpoint_c);
  y += h_set + gap;

  // Temperature graph
  draw_text(dl, colx, y, col_label, "TEMPERATURE  (last 3 min)");
  y += lineh + 2.0f;
  const float h_graph = H * 0.20f;
  history.draw_graph(dl, ImVec2(colx, y), ImVec2(colw, h_graph), now);
  y += h_graph + gap;

  // Toggles
  const float h_tog = H * 0.075f;
  toggle_button("Grain In", c.grain_in, ImVec2(colx, y), ImVec2(colw * 0.48f, h_tog));
  toggle_button("Manual Control", c.manual_control,
                ImVec2(colx + colw * 0.52f, y), ImVec2(colw * 0.48f, h_tog));
  y += h_tog + gap;

  // Duty sliders — editable only in manual mode. In auto the PID owns duty1/
  // duty2 (set by the caller before this frame); the disabled sliders just
  // display the commanded value.
  const float h_slid = H * 0.22f;
  const float sw     = colw * 0.30f;
  const float sh     = h_slid - lineh * 2.0f - 6.0f;
  const float s1x    = colx + colw * 0.10f;
  const float s2x    = colx + colw * 0.58f;
  ImGui::BeginDisabled(!*c.manual_control);
  ImGui::SetCursorPos(ImVec2(s1x, y)); ImGui::VSliderFloat("##d1", ImVec2(sw, sh), c.duty1, 0.0f, 1.0f, "");
  ImGui::SetCursorPos(ImVec2(s2x, y)); ImGui::VSliderFloat("##d2", ImVec2(sw, sh), c.duty2, 0.0f, 1.0f, "");
  ImGui::EndDisabled();
  {
    char p1[16], p2[16];
    snprintf(p1, sizeof(p1), "%.0f%%", *c.duty1 * 100.0f);
    snprintf(p2, sizeof(p2), "%.0f%%", *c.duty2 * 100.0f);
    draw_text(dl, s1x, y + sh + 4.0f, col_label, "SSR1");
    draw_text(dl, s2x, y + sh + 4.0f, col_label, "SSR2");
    draw_text(dl, s1x, y + sh + 4.0f + lineh, s.out1 ? col_green : col_grey, p1);
    draw_text(dl, s2x, y + sh + 4.0f + lineh, s.out2 ? col_green : col_grey, p2);
  }
  y += h_slid + gap;

  // Status band — fixed slots; conditional text only changes color/contents.
  const float sy = y;
  {
    char drdy[24];
    snprintf(drdy, sizeof(drdy), "DRDYn: %s", s.drdy_high ? "HIGH" : "LOW");
    draw_text(dl, colx, sy, col_label, drdy);
    if (s.watchdog) draw_text(dl, colx, sy + lineh * 1.2f, col_red, "WATCHDOG ALARM");
    if (s.rtd_fault) {
      char rf[24];
      snprintf(rf, sizeof(rf), "RTD FAULT (0x%02X)", s.rtd_fault);
      draw_text(dl, colx, sy + lineh * 2.4f, col_red, rf);
      draw_text(dl, colx, sy + lineh * 3.6f, col_pink,
                Max31865::decode_fault_status(s.rtd_fault).c_str());
    }
  }

  // Test Touch lives in the right half of the band. It is the last (and always
  // submitted) widget, so the window never ends on a dangling SetCursorPos.
  const float bw = colw * 0.46f;
  const float bh = H * 0.06f;
  const float bx = colx + colw - bw;
  if (*c.test_pressed) draw_text(dl, bx, sy + bh + 6.0f, col_green, "WORKING!");
  ImGui::SetCursorPos(ImVec2(bx, sy));
  if (ImGui::Button("Test Touch", ImVec2(bw, bh))) *c.test_pressed = !*c.test_pressed;

  ImGui::End();
  ImGui::PopStyleVar();
}
