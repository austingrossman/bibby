#pragma once
#include <imgui.h>

// Draw a stylised, shadowed brew kettle as a half-cutaway side view into the
// rectangle [top_left, top_left+size]. The contents surface (liquid, and the
// grain bed when `grain_in`) starts at absolute y `content_top`, leaving the
// space above it for the caller's temperature readout. Two heating elements sit
// in the lower third of the contents: SSR1 is a folded sling-blade immersion
// element, SSR2 a ripple element. `bright1`/`bright2` in [0,1] set each
// element's glow.
//
// Pure ImDrawList drawing — no widgets, no state. The temperature readout is
// drawn separately by the caller on top of this.
void draw_kettle(ImDrawList *dl, ImVec2 top_left, ImVec2 size,
                 float bright1, float bright2, bool grain_in, float content_top);
