#include <cstdlib>
#include <fcntl.h>
#include <sys/mman.h>
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
#include "display.h"

// MAX31865 DRDY input, GPIO 16 (header pin 36), active-low.
static constexpr unsigned int DRDY_GPIO = 16;

// Samples per stage in the temperature smoothing filter.
static constexpr int TEMP_FILTER_WINDOW = 40;

// Live status snapshot handed to the UI each frame.
struct UiStatus {
  float   temp_c;
  bool    out1, out2;
  bool    watchdog;
  bool    drdy_high;
  uint8_t rtd_fault; // MAX31865 Fault Status register (07h); 0 = no fault
};

static HeaterShm          *shm_connect();
static gpiod_line_request *init_drdy_line(gpiod_chip *chip);
static SDL_Window         *create_window(SDL_GLContext *out_ctx, int *out_w, int *out_h);
static void                init_imgui(SDL_Window *window, SDL_GLContext ctx);
static void                draw_ui(int portrait_w, int portrait_h, const UiStatus &s,
                                   float *duty1, float *duty2, bool *test_pressed);

// ----------------------------------------------------------------------------
int main(void) {
  // Refuse to start if another frontend is already running.
  // Lock fd is intentionally left open for the life of the process.
  if (single_instance_lock("/tmp/biab_frontend.lock") < 0) {
    fprintf(stderr, "frontend: another instance is already running\n");
    return 1;
  }

  SDL_GLContext gl_ctx;
  int display_w, display_h;
  SDL_Window *window = create_window(&gl_ctx, &display_w, &display_h);

  RotatedDisplay display(display_w, display_h);
  init_imgui(window, gl_ctx);

  HeaterShm *shm = shm_connect();

  gpiod_chip *drdy_chip = gpiod_chip_open("/dev/gpiochip4");
  gpiod_line_request *drdy_req = init_drdy_line(drdy_chip);

  Max31865 sensor("/dev/spidev0.0", 400, drdy_req);
  SecondOrderAverage temp_filter(TEMP_FILTER_WINDOW);

  ImGuiIO &io = ImGui::GetIO();
  float duty1 = 0.0f, duty2 = 0.0f;
  float temp_c = 0.0f;
  bool  test_pressed = false;
  bool  running = true;

  while (running) {
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
    if (temp_fresh) temp_c = temp_filter.push(temp_raw);

    // --- Controller status from shared memory ---
    UiStatus status;
    status.temp_c    = temp_c;
    status.out1      = shm->output1;
    status.out2      = shm->output2;
    status.watchdog  = shm->watchdog_alarm;
    status.drdy_high = gpiod_line_request_get_value(drdy_req, DRDY_GPIO) == GPIOD_LINE_VALUE_ACTIVE;
    status.rtd_fault = sensor.fault_status();

    // --- Render the UI into the portrait FBO ---
    display.begin_frame();
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    io.DisplaySize             = ImVec2((float)display.portrait_w(), (float)display.portrait_h());
    io.DisplayFramebufferScale = ImVec2(1.0f, 1.0f);

    ImGui::NewFrame();
    draw_ui(display.portrait_w(), display.portrait_h(), status, &duty1, &duty2, &test_pressed);
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

static void init_imgui(SDL_Window *window, SDL_GLContext ctx) {
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  ImGui::StyleColorsDark();

  ImFontConfig fc;
  fc.SizePixels = 26.0f;
  ImGui::GetIO().Fonts->AddFontDefault(&fc);

  ImGui_ImplSDL3_InitForOpenGL(window, ctx);
  ImGui_ImplOpenGL3_Init("#version 300 es");
}

// ----------------------------------------------------------------------------
// UI — one frame of the control panel. Reads live status, edits the duty/test
// values in place; the caller publishes those to shared memory.
// ----------------------------------------------------------------------------
static void draw_ui(int portrait_w, int portrait_h, const UiStatus &s,
                    float *duty1, float *duty2, bool *test_pressed) {
  const float slider_area_w = (float)portrait_w * 0.4f;
  const float slider_w      = slider_area_w * 0.47f;
  const float content_w     = (float)portrait_w - slider_area_w;
  const float inner_h       = (float)portrait_h - 20.0f;

  ImGui::SetNextWindowPos({0, 0});
  ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize);
  ImGui::Begin("##main", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove);

  // --- Left column: status readout and the test toggle ---
  ImGui::BeginChild("##left", ImVec2(content_w, inner_h));

  ImGui::Text("Temperature: %.2f °C", s.temp_c);
  ImGui::Separator();
  ImGui::Text("SSR 1: %s  (%.0f%%)", s.out1 ? "ON" : "off", *duty1 * 100.0f);
  ImGui::Text("SSR 2: %s  (%.0f%%)", s.out2 ? "ON" : "off", *duty2 * 100.0f);
  ImGui::Text("DRDYn: %s", s.drdy_high ? "HIGH" : "LOW");

  if (s.watchdog) {
    ImGui::TextColored({1.0f, 0.2f, 0.2f, 1.0f}, "WATCHDOG ALARM");
  }

  if (s.rtd_fault) {
    ImGui::TextColored({1.0f, 0.2f, 0.2f, 1.0f}, "RTD FAULT (0x%02X)", s.rtd_fault);
    ImGui::TextColored({1.0f, 0.5f, 0.5f, 1.0f}, "%s",
                       Max31865::decode_fault_status(s.rtd_fault).c_str());
  }

  ImGui::Separator();
  if (ImGui::Button("Test Touch", ImVec2(200, 60))) {
    *test_pressed = !*test_pressed;
  }
  if (*test_pressed) {
    ImGui::SameLine();
    ImGui::TextColored({0.2f, 1.0f, 0.2f, 1.0f}, "WORKING!");
  }

  ImGui::EndChild();

  // --- Right column: vertical duty-cycle sliders ---
  ImGui::SameLine();
  ImGui::BeginChild("##sliders", ImVec2(slider_area_w, inner_h));

  const float slider_h = inner_h - ImGui::GetTextLineHeightWithSpacing() - 8.0f;
  ImGui::VSliderFloat("##ssr1", ImVec2(slider_w, slider_h), duty1, 0.0f, 1.0f, "");
  ImGui::SameLine(0.0f, slider_area_w - slider_w * 2.0f);
  ImGui::VSliderFloat("##ssr2", ImVec2(slider_w, slider_h), duty2, 0.0f, 1.0f, "");

  // Labels centred under each slider.
  ImGui::SetCursorPosX(ImGui::GetCursorPosX() - slider_area_w + slider_w * 0.3f);
  ImGui::Text("SSR 1");
  ImGui::SameLine(slider_w + (slider_area_w - slider_w * 2.0f) + slider_w * 0.3f);
  ImGui::Text("SSR 2");

  ImGui::EndChild();
  ImGui::End();
}
