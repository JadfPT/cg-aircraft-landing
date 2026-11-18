// hud.cpp
//
// A small 2D renderer for the HUD, the information drawn on top of the 3D scene. It draws
// rectangles, circles and text in pixel coordinates. The letters of the fonts are drawn
// once into a texture (the font atlas) with stb_truetype, and each letter is then a small
// textured square. Everything drawn in a frame is kept in one list of vertices and sent
// to the GPU in a single draw call, with its own shaders and VAO.

#include <stdio.h>
#include <math.h>
#include <vector>
#include <string>

#include <GL/glew.h>
#include <glm/glm.hpp>

#define STB_TRUETYPE_IMPLEMENTATION
#include <common/stb_truetype.h>   // TrueType fonts, public domain, https://github.com/nothings/stb

#include <common/shader.hpp>
#include <common/hud.hpp>

// one vertex of the HUD, with its position in pixels, texture coordinates and colour
struct HudVertex { float x, y, u, v, r, g, b, a; };

static const int   ATLAS = 1024;                                  // the font atlas is ATLAS x ATLAS pixels, one byte each
static const float FONT_PIXELS[FONT_COUNT] = { 15.0f, 19.0f, 19.0f, 34.0f };
static stbtt_packedchar glyphs[FONT_COUNT][96];                   // ASCII characters 32 to 127
static GLuint atlasTexture = 0, hudProgram = 0, hudVAO = 0, hudVBO = 0;
static GLuint screenSizeID = 0, atlasID = 0;
static std::vector<HudVertex> batch;
static float  W = 1, H = 1, S = 1;
static glm::vec2 whiteUV;                                         // a white pixel of the atlas, used for the shapes without text

// reads a whole file into memory (the font files)
static bool readFile(const char* path, std::vector<unsigned char>& data)
{
    FILE* f = fopen(path, "rb");
    if (!f) return false;
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    data.resize(n);
    bool ok = fread(data.data(), 1, n, f) == (size_t)n;
    fclose(f);
    return ok;
}

// Creates the font atlas, the HUD shaders and its VAO and buffer
bool hudInit(const char* regularFont, const char* boldFont)
{
    std::vector<unsigned char> regular, bold;
    if (!readFile(regularFont, regular) || !readFile(boldFont, bold)) {
        fprintf(stderr, "HUD: could not read the fonts (%s, %s)\n", regularFont, boldFont);
        return false;
    }

    // draw the letters of the 4 font sizes into one texture, the atlas
    std::vector<unsigned char> pixels(ATLAS * ATLAS, 0);
    stbtt_pack_context pc;
    stbtt_PackBegin(&pc, pixels.data(), ATLAS, ATLAS, 0, 1, NULL);
    stbtt_PackSetOversampling(&pc, 2, 2);
    for (int f = 0; f < FONT_COUNT; f++) {
        const unsigned char* ttf = (f == FONT_BOLD || f == FONT_BIG) ? bold.data() : regular.data();
        stbtt_PackFontRange(&pc, ttf, 0, FONT_PIXELS[f] * 1.6f, 32, 96, glyphs[f]);   // drawn bigger here and scaled down when used, so they look sharper
    }
    stbtt_PackEnd(&pc);
    // a small white square in the last corner of the atlas, used by rectangles and circles
    for (int y = ATLAS - 4; y < ATLAS; y++)
        for (int x = ATLAS - 4; x < ATLAS; x++) pixels[y * ATLAS + x] = 255;
    whiteUV = glm::vec2((ATLAS - 2.0f) / ATLAS, (ATLAS - 2.0f) / ATLAS);

    glGenTextures(1, &atlasTexture);
    glBindTexture(GL_TEXTURE_2D, atlasTexture);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, ATLAS, ATLAS, 0, GL_RED, GL_UNSIGNED_BYTE, pixels.data());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    hudProgram = LoadShaders("HUDVertexShader.vertexshader", "HUDFragmentShader.fragmentshader");
    if (hudProgram == 0) return false;
    screenSizeID = glGetUniformLocation(hudProgram, "ScreenSize");
    atlasID      = glGetUniformLocation(hudProgram, "Atlas");

    // the HUD has its own VAO, with the layout of HudVertex
    GLint previousVAO;
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &previousVAO);
    glGenVertexArrays(1, &hudVAO);
    glBindVertexArray(hudVAO);
    glGenBuffers(1, &hudVBO);
    glBindBuffer(GL_ARRAY_BUFFER, hudVBO);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(HudVertex), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(HudVertex), (void*)(2 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(HudVertex), (void*)(4 * sizeof(float)));
    glBindVertexArray(previousVAO);
    return true;
}

// frees what hudInit() created
void hudCleanup(void)
{
    glDeleteTextures(1, &atlasTexture);
    glDeleteBuffers(1, &hudVBO);
    glDeleteVertexArrays(1, &hudVAO);
    glDeleteProgram(hudProgram);
}

// starts a new frame of the HUD, scale makes everything bigger with the window size
void hudBegin(int width, int height, float scale)
{
    W = (float)width; H = (float)height; S = scale;
    batch.clear();
}

float hudScale(void) { return S; }

// adds a textured square to the list, as two triangles
static void quad(float x0, float y0, float x1, float y1, float u0, float v0, float u1, float v1, const glm::vec4& c)
{
    HudVertex a = { x0, y0, u0, v0, c.r, c.g, c.b, c.a }, b = { x1, y0, u1, v0, c.r, c.g, c.b, c.a };
    HudVertex d = { x0, y1, u0, v1, c.r, c.g, c.b, c.a }, e = { x1, y1, u1, v1, c.r, c.g, c.b, c.a };
    batch.push_back(a); batch.push_back(b); batch.push_back(e);
    batch.push_back(a); batch.push_back(e); batch.push_back(d);
}

// adds a triangle with a solid colour
static void tri(glm::vec2 p, glm::vec2 q, glm::vec2 r, const glm::vec4& c)
{
    HudVertex a = { p.x, p.y, whiteUV.x, whiteUV.y, c.r, c.g, c.b, c.a };
    HudVertex b = { q.x, q.y, whiteUV.x, whiteUV.y, c.r, c.g, c.b, c.a };
    HudVertex d = { r.x, r.y, whiteUV.x, whiteUV.y, c.r, c.g, c.b, c.a };
    batch.push_back(a); batch.push_back(b); batch.push_back(d);
}

// solid rectangle
void hudRect(float x, float y, float w, float h, const glm::vec4& color)
{
    quad(x, y, x + w, y + h, whiteUV.x, whiteUV.y, whiteUV.x, whiteUV.y, color);
}

// circle made of 24 triangles around the centre
void hudCircle(float x, float y, float r, const glm::vec4& color)
{
    const int N = 24;
    for (int i = 0; i < N; i++) {
        float a0 = 6.2831853f * i / N, a1 = 6.2831853f * (i + 1) / N;
        tri(glm::vec2(x, y), glm::vec2(x + r * cosf(a0), y + r * sinf(a0)), glm::vec2(x + r * cosf(a1), y + r * sinf(a1)), color);
    }
}

// rectangle with round corners, three rectangles plus a quarter of a circle in each corner
void hudRoundRect(float x, float y, float w, float h, float r, const glm::vec4& color)
{
    r = glm::min(r, glm::min(w, h) * 0.5f);
    hudRect(x + r, y, w - 2 * r, h, color);
    hudRect(x, y + r, r, h - 2 * r, color);
    hudRect(x + w - r, y + r, r, h - 2 * r, color);
    const int N = 6;
    glm::vec2 centers[4] = { glm::vec2(x + r, y + r), glm::vec2(x + w - r, y + r),
                             glm::vec2(x + w - r, y + h - r), glm::vec2(x + r, y + h - r) };
    for (int k = 0; k < 4; k++)
        for (int i = 0; i < N; i++) {
            float a0 = 1.5707963f * (k + 2) + 1.5707963f * i / N, a1 = 1.5707963f * (k + 2) + 1.5707963f * (i + 1) / N;
            tri(centers[k], centers[k] + r * glm::vec2(cosf(a0), sinf(a0)), centers[k] + r * glm::vec2(cosf(a1), sinf(a1)), color);
        }
}

// width in pixels of a text, adding the advance of each letter
float hudTextWidth(const std::string& text, HudFont font)
{
    float k = FONT_PIXELS[font] * S / (FONT_PIXELS[font] * 1.6f), w = 0.0f;
    for (size_t i = 0; i < text.size(); i++) {
        unsigned char c = (unsigned char)text[i];
        if (c < 32 || c > 127) continue;
        w += glyphs[font][c - 32].xadvance * k;
    }
    return w;
}

// text with its baseline at y, each letter is a square with its part of the atlas
float hudText(float x, float y, const std::string& text, HudFont font, const glm::vec4& color, HudAlign align)
{
    float w = hudTextWidth(text, font);
    if (align == ALIGN_CENTER) x -= 0.5f * w;
    if (align == ALIGN_RIGHT)  x -= w;
    float k = FONT_PIXELS[font] * S / (FONT_PIXELS[font] * 1.6f);   // the letters were drawn 1.6 times larger in the atlas
    float pen = 0.0f;
    for (size_t i = 0; i < text.size(); i++) {
        unsigned char c = (unsigned char)text[i];
        if (c < 32 || c > 127) continue;
        const stbtt_packedchar& g = glyphs[font][c - 32];
        float x0 = x + pen + g.xoff * k, y0 = y + g.yoff * k;
        float x1 = x + pen + g.xoff2 * k, y1 = y + g.yoff2 * k;
        quad(x0, y0, x1, y1, g.x0 / (float)ATLAS, g.y0 / (float)ATLAS, g.x1 / (float)ATLAS, g.y1 / (float)ATLAS, color);
        pen += g.xadvance * k;
    }
    return w;
}

// Sends the whole list to the GPU and draws it. No depth test, so it stays on top,
// and with blending for the soft edges of the letters
void hudEnd(void)
{
    if (batch.empty()) return;
    GLint previousVAO;
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &previousVAO);

    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glUseProgram(hudProgram);
    glUniform2f(screenSizeID, W, H);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, atlasTexture);
    glUniform1i(atlasID, 0);

    glBindVertexArray(hudVAO);
    glBindBuffer(GL_ARRAY_BUFFER, hudVBO);
    glBufferData(GL_ARRAY_BUFFER, batch.size() * sizeof(HudVertex), batch.data(), GL_STREAM_DRAW);
    glDrawArrays(GL_TRIANGLES, 0, (GLsizei)batch.size());

    glBindVertexArray(previousVAO);
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
}
