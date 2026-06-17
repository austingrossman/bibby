#pragma once
#include <GLES3/gl3.h>

// The physical panel is mounted sideways. The UI is rendered into a portrait
// framebuffer, then blitted to the landscape screen rotated 90° CW by a
// fullscreen quad. Construct once a GL context is current.
class RotatedDisplay {
public:
  RotatedDisplay(int display_w, int display_h);
  ~RotatedDisplay();

  int portrait_w() const { return portrait_w_; }
  int portrait_h() const { return portrait_h_; }

  void begin_frame(); // bind + clear the portrait FBO; draw the UI after this
  void present();     // blit the portrait FBO to the screen, rotated

private:
  int    display_w_, display_h_;
  int    portrait_w_, portrait_h_;
  GLuint fbo_       = 0;
  GLuint fbo_tex_   = 0;
  GLuint blit_prog_ = 0;
  GLuint vao_       = 0;
};
