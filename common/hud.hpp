// hud.hpp
//
// Small 2D renderer for the HUD (see hud.cpp).

#ifndef HUD_HPP
#define HUD_HPP

#include <string>
#include <glm/glm.hpp>

// It draws coloured rectangles, circles and TrueType text (stb_truetype) in pixel
// coordinates, with the origin at the top left corner of the window. Everything drawn
// between hudBegin() and hudEnd() is sent to the GPU in a single batch

// the 4 font sizes and how a text is aligned to x
enum HudFont { FONT_SMALL = 0, FONT_NORMAL, FONT_BOLD, FONT_BIG, FONT_COUNT };
enum HudAlign { ALIGN_LEFT = 0, ALIGN_CENTER, ALIGN_RIGHT };

bool  hudInit(const char* regularFont, const char* boldFont);
void  hudCleanup(void);

// scale multiplies every size, so the HUD grows with the window
void  hudBegin(int width, int height, float scale);
void  hudEnd(void);

float hudScale(void);
void  hudRect(float x, float y, float w, float h, const glm::vec4& color);
void  hudRoundRect(float x, float y, float w, float h, float r, const glm::vec4& color);
void  hudCircle(float x, float y, float r, const glm::vec4& color);
float hudText(float x, float y, const std::string& text, HudFont font, const glm::vec4& color,
              HudAlign align = ALIGN_LEFT);   // y is the baseline, returns the width of the text
float hudTextWidth(const std::string& text, HudFont font);

#endif
