#include <cstdlib>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <SDL3/SDL.h>
#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_opengl3.h>
#include <SDL3/SDL_opengl.h>

#include "../shared/shm_types.h"
#include "sensors/max31865.h"

static HeaterShm *shm_connect(void) {
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

int main(void) {
  SDL_Init(SDL_INIT_VIDEO);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);

  SDL_Window *window = SDL_CreateWindow(
    "BIAB Ramp Controller",
    800, 480,
    SDL_WINDOW_OPENGL | SDL_WINDOW_FULLSCREEN
  );
  SDL_GLContext gl_ctx = SDL_GL_CreateContext(window);
  SDL_GL_SetSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
  ImGui::StyleColorsDark();

  ImGui_ImplSDL3_InitForOpenGL(window, gl_ctx);
  ImGui_ImplOpenGL3_Init("#version 300 es");

  HeaterShm *shm = shm_connect();
  // Max31865 sensor("/dev/spidev0.0");

  float duty1 = 0.0f, duty2 = 0.0f;

  bool running = true;
  while (running) {
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
      ImGui_ImplSDL3_ProcessEvent(&e);
      if (e.type == SDL_EVENT_QUIT) {
        running = false;
      }
    }

    // float temp_c = sensor.read_temperature();
    float temp_c = 21.3f;

    shm->duty1.store(duty1);
    shm->duty2.store(duty2);

    bool out1     = shm->output1.load();
    bool out2     = shm->output2.load();
    bool watchdog = shm->watchdog_alarm.load();

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowPos({0, 0});
    ImGui::SetNextWindowSize(io.DisplaySize);
    ImGui::Begin("##main", nullptr,
      ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove);

    ImGui::Text("Temperature: %.2f °C", temp_c);
    ImGui::Separator();

    ImGui::SliderFloat("SSR 1 Duty", &duty1, 0.0f, 1.0f);
    ImGui::SliderFloat("SSR 2 Duty", &duty2, 0.0f, 1.0f);
    ImGui::Separator();

    ImGui::Text("SSR 1 output: %s", out1 ? "ON" : "off");
    ImGui::Text("SSR 2 output: %s", out2 ? "ON" : "off");

    if (watchdog) {
      ImGui::TextColored({1, 0.2f, 0.2f, 1}, "WATCHDOG ALARM");
    }

    ImGui::End();
    ImGui::Render();

    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    SDL_GL_SwapWindow(window);
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplSDL3_Shutdown();
  ImGui::DestroyContext();
  SDL_GL_DestroyContext(gl_ctx);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}
