#include "display.h"

// ----------------------------------------------------------------------------
// Blit shaders — map the landscape screen back to the portrait FBO, rotated CW.
//   portrait_u = (ndc_y + 1) / 2
//   portrait_v = (1 - ndc_x) / 2
// ----------------------------------------------------------------------------
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

// ----------------------------------------------------------------------------
// RotatedDisplay
// ----------------------------------------------------------------------------
RotatedDisplay::RotatedDisplay(int display_w, int display_h)
  : display_w_(display_w), display_h_(display_h),
    portrait_w_(display_h), portrait_h_(display_w) {

  // Portrait-sized FBO the UI renders into.
  glGenFramebuffers(1, &fbo_);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
  glGenTextures(1, &fbo_tex_);
  glBindTexture(GL_TEXTURE_2D, fbo_tex_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, portrait_w_, portrait_h_, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fbo_tex_, 0);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  blit_prog_ = build_blit_program();
  glGenVertexArrays(1, &vao_);
}

RotatedDisplay::~RotatedDisplay() {
  glDeleteFramebuffers(1, &fbo_);
  glDeleteTextures(1, &fbo_tex_);
  glDeleteProgram(blit_prog_);
  glDeleteVertexArrays(1, &vao_);
}

void RotatedDisplay::begin_frame() {
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
  glViewport(0, 0, portrait_w_, portrait_h_);
  glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);
}

void RotatedDisplay::present() {
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glViewport(0, 0, display_w_, display_h_);
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  glUseProgram(blit_prog_);
  glBindTexture(GL_TEXTURE_2D, fbo_tex_);
  glBindVertexArray(vao_);
  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
  glBindVertexArray(0);
  glUseProgram(0);
}
