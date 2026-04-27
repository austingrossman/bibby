#include <cstdlib>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <SDL3/SDL.h>
#include <GLES3/gl3.h>
#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_opengl3.h>

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

// Fullscreen quad that blits the portrait FBO onto the landscape screen, rotated 90° CW.
// portrait_u = (ndc_y + 1) / 2
// portrait_v = (1 - ndc_x) / 2
static const char *BLIT_VERT = R"(#version 300 es
const vec2 pos[4] = vec2[4](
  vec2(-1.0,  1.0),
  vec2( 1.0,  1.0),
  vec2(-1.0, -1.0),
  vec2( 1.0, -1.0)
);
out vec2 vUV;
void main() {
  vec2 p  = pos[gl_VertexID];
  vUV     = vec2((p.y + 1.0) * 0.5, (1.0 - p.x) * 0.5);
  gl_Position = vec4(p, 0.0, 1.0);
}
)";

static const char *BLIT_FRAG = R"(#version 300 es
precision mediump float;
in vec2 vUV;
uniform sampler2D uTex;
out vec4 fragColor;
void main() {
  fragColor = texture(uTex, vUV);
}
)";

static GLuint build_blit_program() {
  auto compile = [](GLenum type, const char *src) -> GLuint {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    return s;
  };
  GLuint vert = compile(GL_VERTEX_SHADER,   BLIT_VERT);
  GLuint frag = compile(GL_FRAGMENT_SHADER, BLIT_FRAG);
  GLuint prog = glCreateProgram();
  glAttachShader(prog, vert);
  glAttachShader(prog, frag);
  glLinkProgram(prog);
  glDeleteShader(vert);
  glDeleteShader(frag);
  return prog;
}


int main(void) {
  SDL_Init(SDL_INIT_VIDEO);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);

  // Query actual display resolution at runtime
  const SDL_DisplayMode *mode = SDL_GetCurrentDisplayMode(SDL_GetPrimaryDisplay());
  const int display_w  = mode->w;
  const int display_h  = mode->h;
  const int portrait_w = display_h;
  const int portrait_h = display_w;

  SDL_Window *window = SDL_CreateWindow(
    "BIAB Ramp Controller",
    display_w, display_h,
    SDL_WINDOW_OPENGL | SDL_WINDOW_FULLSCREEN
  );
  SDL_GLContext gl_ctx = SDL_GL_CreateContext(window);
  SDL_GL_SetSwapInterval(1);

  // FBO for portrait rendering
  GLuint fbo, fbo_tex;
  glGenFramebuffers(1, &fbo);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo);
  glGenTextures(1, &fbo_tex);
  glBindTexture(GL_TEXTURE_2D, fbo_tex);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, portrait_w, portrait_h, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fbo_tex, 0);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  GLuint blit_prog = build_blit_program();
  GLuint dummy_vao;
  glGenVertexArrays(1, &dummy_vao);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  ImGui::StyleColorsDark();

  ImFontConfig fc;
  fc.SizePixels = 26.0f;
  io.Fonts->AddFontDefault(&fc);

  ImGui_ImplSDL3_InitForOpenGL(window, gl_ctx);
  ImGui_ImplOpenGL3_Init("#version 300 es");

  HeaterShm *shm = shm_connect();
  Max31865 sensor("/dev/spidev0.0");

  float duty1 = 0.0f, duty2 = 0.0f;
  bool test_pressed = false;

  bool running = true;
  while (running) {
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
      if (e.type == SDL_EVENT_FINGER_DOWN ||
          e.type == SDL_EVENT_FINGER_UP   ||
          e.type == SDL_EVENT_FINGER_MOTION) {
        io.AddMousePosEvent(portrait_w * (1.0f - e.tfinger.y),
                            portrait_h * e.tfinger.x);
        if (e.type != SDL_EVENT_FINGER_MOTION) {
          io.AddMouseButtonEvent(0, e.type == SDL_EVENT_FINGER_DOWN);
        }
      } else {
        ImGui_ImplSDL3_ProcessEvent(&e);
        if (e.type == SDL_EVENT_QUIT) {
          running = false;
        }
      }
    }

    float temp_c = 21.3f; // sensor.read_temperature();

    bool out1     = shm->output1.load();
    bool out2     = shm->output2.load();
    bool watchdog = shm->watchdog_alarm.load();

    // Render ImGui into portrait FBO
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glViewport(0, 0, portrait_w, portrait_h);

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    // Override display size to portrait after SDL backend sets it to landscape
    io.DisplaySize             = ImVec2((float)portrait_w, (float)portrait_h);
    io.DisplayFramebufferScale = ImVec2(1.0f, 1.0f);

    ImGui::NewFrame();

    ImGui::SetNextWindowPos({0, 0});
    ImGui::SetNextWindowSize(io.DisplaySize);
    ImGui::Begin("##main", nullptr,
      ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove);

    ImGui::Text("Temperature: %.2f °C", temp_c);
    ImGui::Separator();

    ImGui::SliderFloat("SSR 1 Duty", &duty1, 0.0f, 1.0f);
    ImGui::SliderFloat("SSR 2 Duty", &duty2, 0.0f, 1.0f);

    shm->duty1.store(duty1);
    shm->duty2.store(duty2);

    ImGui::Separator();

    ImGui::Text("SSR 1 output: %s", out1 ? "ON" : "off");
    ImGui::Text("SSR 2 output: %s", out2 ? "ON" : "off");

    if (watchdog) {
      ImGui::TextColored({1.0f, 0.2f, 0.2f, 1.0f}, "WATCHDOG ALARM");
    }

    ImGui::Separator();
    if (ImGui::Button("Test Touch", ImVec2(200, 60))) {
      test_pressed = !test_pressed;
    }
    if (test_pressed) {
      ImGui::SameLine();
      ImGui::TextColored({0.2f, 1.0f, 0.2f, 1.0f}, "WORKING!");
      shm->simulate_zc.store(true);
    } else {
      shm->simulate_zc.store(false);
    }

    ImGui::End();
    ImGui::Render();

    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    // Blit portrait FBO to landscape screen, rotated 90° CW
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    glUseProgram(blit_prog);
    glBindTexture(GL_TEXTURE_2D, fbo_tex);
    glBindVertexArray(dummy_vao);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glBindVertexArray(0);
    glUseProgram(0);

    SDL_GL_SwapWindow(window);
  }

  glDeleteFramebuffers(1, &fbo);
  glDeleteTextures(1, &fbo_tex);
  glDeleteProgram(blit_prog);
  glDeleteVertexArrays(1, &dummy_vao);
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplSDL3_Shutdown();
  ImGui::DestroyContext();
  SDL_GL_DestroyContext(gl_ctx);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}
