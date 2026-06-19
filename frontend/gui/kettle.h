#pragma once
#include <imgui.h>

// Draw a stylised, shadowed brew kettle as a half-cutaway side view into the
// rectangle [top_left, top_left+size]. The upper two-thirds of the contents show
// a grain bed when `grain_in`, otherwise just wort. Two heating elements sit in
// the lower third: SSR1 is a folded sling-blade immersion element, SSR2 a ripple
// element. Each glows bright when its output is asserted.
//
// Pure ImDrawList drawing — no widgets, no state. The temperature readout is
// drawn separately by the caller on top of this.
void draw_kettle(ImDrawList *dl, ImVec2 top_left, ImVec2 size,
                 bool out1, bool out2, bool grain_in);
