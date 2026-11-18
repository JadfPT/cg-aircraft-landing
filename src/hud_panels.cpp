// hud_panels.cpp
//
// The 2D information on top of the 3D scene (HUD). The flight data on the left (speed,
// height, vertical speed, flaps, engine, throttle), the approach guidance on the right
// (ILS style needles and the 4 PAPI lights), the keys at the bottom, the help card and
// the cards shown after a landing or a crash. The shapes and text themselves are drawn
// by hud.cpp.

#include <common/landing.hpp>
#include <common/hud.hpp>

// colours of the HUD
const glm::vec4 HUD_PANEL(0.06f, 0.08f, 0.11f, 0.72f);
const glm::vec4 HUD_WHITE(1.0f, 1.0f, 1.0f, 1.0f);
const glm::vec4 HUD_GREY(0.70f, 0.74f, 0.80f, 1.0f);
const glm::vec4 HUD_GREEN(0.40f, 0.95f, 0.55f, 1.0f);
const glm::vec4 HUD_AMBER(1.00f, 0.75f, 0.25f, 1.0f);
const glm::vec4 HUD_RED(1.00f, 0.35f, 0.30f, 1.0f);
const glm::vec4 HUD_BLUE(0.35f, 0.65f, 1.00f, 1.0f);
const glm::vec4 HUD_MAGENTA(1.00f, 0.35f, 0.95f, 1.0f);

// a key drawn like a button followed by what it does, returns the width it used
float keyChip(float x, float y, const std::string& key, const std::string& label, const glm::vec4& labelColor = HUD_WHITE)
{
    float s = hudScale(), h = 24 * s, pad = 7 * s;
    float kw = hudTextWidth(key, FONT_BOLD) + 2 * pad;
    hudRoundRect(x, y, kw, h, 5 * s, glm::vec4(0.92f, 0.94f, 0.97f, 0.95f));
    hudText(x + pad, y + 17.5f * s, key, FONT_BOLD, glm::vec4(0.08f, 0.10f, 0.14f, 1.0f));
    float lw = label.empty() ? 0.0f : hudText(x + kw + 6 * s, y + 17.5f * s, label, FONT_NORMAL, labelColor) + 6 * s;
    return kw + lw;
}

// width that keyChip() will use, to centre a row of keys
float keyChipWidth(const std::string& key, const std::string& label)
{
    float s = hudScale();
    return hudTextWidth(key, FONT_BOLD) + 14 * s + (label.empty() ? 0.0f : hudTextWidth(label, FONT_NORMAL) + 12 * s);
}

// one line of the flight data panel, with a label, a big value and the unit
void dataRow(float x, float y, const std::string& label, const std::string& value, const std::string& unit,
             const glm::vec4& color = HUD_WHITE)
{
    float s = hudScale();
    hudText(x, y + 15 * s, label, FONT_SMALL, HUD_GREY);
    float vw = hudText(x, y + 50 * s, value, FONT_BIG, color);
    hudText(x + vw + 6 * s, y + 50 * s, unit, FONT_NORMAL, HUD_GREY);
}

// left panel with the speed, height, vertical speed, distance, flaps, engine and throttle
void drawFlightData(float x, float y)
{
    float s = hudScale(), w = 235 * s, row = 62 * s, h = 7 * row + 20 * s;
    hudRoundRect(x, y, w, h, 10 * s, HUD_PANEL);
    x += 16 * s; y += 8 * s;

    glm::vec4 iasColor = (phase == APPROACH && ac.speed < 58.0f * KT) ? HUD_AMBER : HUD_WHITE;
    dataRow(x, y, "AIRSPEED", fmt("%.0f", ac.speed / KT), "kt", iasColor);                       y += row;
    float agl = ac.pos.y - groundHeight(ac.pos.x, ac.pos.z);
    dataRow(x, y, "HEIGHT ABOVE GROUND", fmt("%.0f", glm::max(agl, 0.0f) / FT), "ft", agl < 30.0f * FT && phase <= APPROACH ? HUD_AMBER : HUD_WHITE);                 y += row;
    float fpm = ac.vs / FT * 60.0f;
    glm::vec4 vsColor = (fpm < -700.0f) ? HUD_RED : (fpm < -450.0f ? HUD_AMBER : HUD_WHITE);
    dataRow(x, y, "VERTICAL SPEED", fabsf(fpm) < 0.5f ? std::string("0") : fmt("%+.0f", fpm), "ft/min", vsColor);                      y += row;
    float dist = ac.pos.z;   // the threshold is at z = 0
    if (dist > 0.0f) dataRow(x, y, "DISTANCE TO RUNWAY", dist > 1000 ? fmt("%.2f", dist / 1000) : fmt("%.0f", dist),
                             dist > 1000 ? "km" : "m");
    else             dataRow(x, y, "PAST THE THRESHOLD", fmt("%.0f", -dist), "m");
    y += row;

    // flaps, the value and the 4 positions (0, 10, 25, 40)
    hudText(x, y + 15 * s, "FLAPS", FONT_SMALL, HUD_GREY);
    float vw = hudText(x, y + 50 * s, fmt("%.0f", ac.flaps), FONT_BIG, HUD_WHITE);
    hudText(x + vw + 6 * s, y + 50 * s, "deg", FONT_NORMAL, HUD_GREY);
    const float steps[4] = { 0, 10, 25, 40 };
    for (int i = 0; i < 4; i++) {
        bool on = ac.flaps >= steps[i] - 0.5f;
        hudRoundRect(x + 110 * s + i * 22 * s, y + 30 * s, 17 * s, 20 * s, 3 * s, on ? HUD_BLUE : glm::vec4(1, 1, 1, 0.15f));
    }
    y += row;
    if (rpm < 50.0f) dataRow(x, y, "ENGINE", "OFF", "", HUD_GREY);
    else             dataRow(x, y, "ENGINE", fmt("%.0f", rpm), "rpm");
    y += row;

    // throttle with a bar, the autopilot moves it as well
    float lever = (rpm < 50.0f) ? 0.0f : throttle;
    hudText(x, y + 15 * s, "THROTTLE", FONT_SMALL, HUD_GREY);
    if (manualMode && brakes)
        hudText(x + w - 32 * s, y + 15 * s, "BRAKES ON", FONT_SMALL, HUD_AMBER, ALIGN_RIGHT);
    vw = hudText(x, y + 50 * s, fmt("%.0f", lever * 100.0f), FONT_BIG, HUD_WHITE);
    hudText(x + vw + 6 * s, y + 50 * s, "%", FONT_NORMAL, HUD_GREY);
    float bx = x + 110 * s, bw = 83 * s;
    hudRoundRect(bx, y + 33 * s, bw, 14 * s, 4 * s, glm::vec4(1, 1, 1, 0.15f));
    if (lever > 0.01f) hudRoundRect(bx, y + 33 * s, glm::max(bw * lever, 8 * s), 14 * s, 4 * s, HUD_BLUE);
}

// approach guidance, glide slope and centre line needles (like an ILS) and the PAPI lights
void drawApproachGuide(float x, float y)
{
    float s = hudScale(), w = 235 * s, h = 300 * s;
    hudRoundRect(x, y, w, h, 10 * s, HUD_PANEL);
    hudText(x + w / 2, y + 26 * s, "APPROACH GUIDANCE", FONT_BOLD, HUD_WHITE, ALIGN_CENTER);

    bool active = (phase <= APPROACH || phase == FLARE) && ac.pos.z > AIM_POINT_Z;
    float bx = x + 40 * s, by = y + 45 * s, bs = 150 * s, cx = bx + bs / 2, cy = by + bs / 2;
    hudRect(bx, by, bs, bs, glm::vec4(0, 0, 0, 0.35f));
    hudRect(bx, cy - 1 * s, bs, 2 * s, glm::vec4(1, 1, 1, 0.5f));
    hudRect(cx - 1 * s, by, 2 * s, bs, glm::vec4(1, 1, 1, 0.5f));
    for (int i = -2; i <= 2; i++) if (i != 0) {
        hudCircle(cx + i * bs / 5.5f, cy, 3 * s, glm::vec4(1, 1, 1, 0.6f));
        hudCircle(cx, cy + i * bs / 5.5f, 3 * s, glm::vec4(1, 1, 1, 0.6f));
    }
    // aircraft symbol at the centre
    hudRect(cx - 18 * s, cy - 2 * s, 36 * s, 4 * s, HUD_AMBER);
    hudRect(cx - 2 * s, cy - 8 * s, 4 * s, 10 * s, HUD_AMBER);

    if (active) {
        // Each dot is 0.35 deg. The needles show where the path is, so a needle above the
        // centre means the aircraft is too low
        float gsDots = glm::clamp((elevationAngleDeg(glm::vec3(0.0f, 0.0f, AIM_POINT_Z)) - GLIDE_SLOPE_DEG) / 0.35f, -2.5f, 2.5f);
        float locDeg = glm::degrees(atan2f(ac.pos.x, ac.pos.z + RUNWAY_LENGTH + 300.0f));
        float locDots = glm::clamp(locDeg / 0.5f, -2.5f, 2.5f);
        hudRect(bx, cy + gsDots * bs / 5.5f - 2 * s, bs, 4 * s, HUD_MAGENTA);   // horizontal needle, glide slope
        hudRect(cx - locDots * bs / 5.5f - 2 * s, by, 4 * s, bs, HUD_GREEN);   // vertical needle, runway centre line
    }
    hudText(bx + bs + 6 * s, by + 12 * s, "G/S", FONT_SMALL, HUD_MAGENTA);
    hudText(cx, by + bs + 16 * s, "runway centre line", FONT_SMALL, HUD_GREEN, ALIGN_CENTER);

    // the 4 PAPI lights, white when the aircraft is above the angle of that light and red when below
    float py = by + bs + 48 * s;
    hudText(x + 16 * s, py + 5 * s, "PAPI", FONT_BOLD, HUD_WHITE);
    int whites = 0;
    for (int i = 3; i >= 0; i--) {          // from the outer to the inner light, left to right as the pilot sees them
        bool white = papiIsWhite(i);
        whites += white ? 1 : 0;
        hudCircle(x + 85 * s + (3 - i) * 28 * s, py, 10 * s, white ? glm::vec4(1, 1, 1, 1) : glm::vec4(1.0f, 0.15f, 0.1f, 1));
    }
    std::string msg = "on the glide path"; glm::vec4 col = HUD_GREEN;
    if (!active)          { msg = (phase >= ROLLOUT) ? "on the runway" : "PAPI behind the aircraft"; col = HUD_GREY; }
    else if (whites == 4) { msg = "too high"; col = HUD_AMBER; }
    else if (whites == 3) { msg = "slightly high"; col = HUD_WHITE; }
    else if (whites == 1) { msg = "slightly low"; col = HUD_WHITE; }
    else if (whites == 0) { msg = "too low!"; col = HUD_RED; }
    hudText(x + w / 2, py + 34 * s, msg, FONT_BOLD, col, ALIGN_CENTER);
}

// top left, the flight phase in colour, then the camera, simulation speed and night
void drawStatusBar(float x, float y)
{
    float s = hudScale();
    std::string label = PHASE_NAME[phase]; glm::vec4 col = HUD_BLUE;
    if (phase == FLARE) col = HUD_AMBER;
    if (phase == ROLLOUT) col = HUD_GREEN;
    if (phase == STOPPED) { label = "LANDED"; col = HUD_GREEN; }
    if (phase == READY) { label = "READY ON FINAL"; col = HUD_GREY; }
    if (!simRunning && phase != READY && phase != STOPPED) { label = "PAUSED"; col = HUD_GREY; }
    if (manualMode) { label = simRunning ? "MANUAL CONTROL" : "MANUAL - PAUSED"; col = simRunning ? HUD_AMBER : HUD_GREY; }
    if (crashed) { label = "CRASHED"; col = HUD_RED; }

    float tw = hudTextWidth(label, FONT_BOLD) + 28 * s;
    hudRoundRect(x, y, tw, 32 * s, 16 * s, glm::vec4(col.r, col.g, col.b, 0.9f));
    hudText(x + tw / 2, y + 22 * s, label, FONT_BOLD, glm::vec4(0.05f, 0.07f, 0.1f, 1.0f), ALIGN_CENTER);
    x += tw + 10 * s;

    std::string info = std::string("Camera: ") + CAMERA_NAME[cameraMode];
    if (timeScale != 1.0f) info += "    Speed: x" + fmt("%g", timeScale);
    if (nightMode) info += "    Night";
    float iw = hudTextWidth(info, FONT_NORMAL) + 24 * s;
    hudRoundRect(x, y, iw, 32 * s, 16 * s, HUD_PANEL);
    hudText(x + 12 * s, y + 22 * s, info, FONT_NORMAL, HUD_WHITE);
}

// bar at the bottom with the most useful keys, which change with the mode
void drawKeyBar(float W, float H)
{
    float s = hudScale();
    std::vector<std::pair<std::string, std::string> > keys;
    if (cameraMode == CAM_FREE) {
        keys.push_back(std::make_pair("W A S D", "move"));
        keys.push_back(std::make_pair("Q E", "down / up"));
        keys.push_back(std::make_pair("Left drag", "rotate"));
        keys.push_back(std::make_pair("Wheel", "zoom"));
        keys.push_back(std::make_pair("C", "camera"));
        keys.push_back(std::make_pair("H", "help"));
    } else if (manualMode) {
        keys.push_back(std::make_pair("W S", "nose down/up"));
        keys.push_back(std::make_pair("A D", "turn"));
        keys.push_back(std::make_pair("Q E", "throttle"));
        keys.push_back(std::make_pair("F V", "flaps"));
        keys.push_back(std::make_pair("B", brakes ? "brakes off" : "brakes"));
        keys.push_back(std::make_pair("SPACE", simRunning ? "pause" : "continue"));
        keys.push_back(std::make_pair("M", "exit manual"));
    } else {
        keys.push_back(std::make_pair("SPACE", crashed ? "try again" : (phase == STOPPED ? "fly again" : (simRunning ? "pause" : "start"))));
        keys.push_back(std::make_pair("R", "restart"));
        keys.push_back(std::make_pair("C", "camera"));
        keys.push_back(std::make_pair("+ -", "speed"));
        keys.push_back(std::make_pair("M", "fly it yourself"));
        keys.push_back(std::make_pair("N", "day/night"));
        keys.push_back(std::make_pair("H", "help"));
    }
    float total = 0;
    for (size_t i = 0; i < keys.size(); i++) total += keyChipWidth(keys[i].first, keys[i].second) + 14 * s;
    float x = (W - total) / 2, y = H - 44 * s;
    hudRoundRect(x - 12 * s, y - 8 * s, total + 10 * s, 40 * s, 12 * s, HUD_PANEL);
    for (size_t i = 0; i < keys.size(); i++) x += keyChip(x, y, keys[i].first, keys[i].second) + 14 * s;
}

// the message of notify(), fading out
void drawToast(float W)
{
    double age = glfwGetTime() - toastTime;
    if (age > 3.5 || toastText.empty()) return;
    float s = hudScale(), a = (float)glm::clamp(3.5 - age, 0.0, 1.0);
    float tw = hudTextWidth(toastText, FONT_BOLD) + 40 * s;
    hudRoundRect((W - tw) / 2, 64 * s, tw, 40 * s, 12 * s, glm::vec4(0.06f, 0.08f, 0.11f, 0.8f * a));
    hudText(W / 2, 90 * s, toastText, FONT_BOLD, glm::vec4(toastColor.r, toastColor.g, toastColor.b, a), ALIGN_CENTER);
}

// card shown when the aircraft stops, with the touchdown data compared with the POH
void drawLandingSummary(float W, float H)
{
    float s = hudScale(), w = 440 * s, h = 270 * s, x = (W - w) / 2, y = H - h - 80 * s;
    hudRoundRect(x, y, w, h, 14 * s, glm::vec4(0.06f, 0.08f, 0.11f, 0.85f));
    std::string verdict = "Smooth landing"; glm::vec4 col = HUD_GREEN;
    if (touchdownSinkFpm > 200) { verdict = "Firm landing"; col = HUD_AMBER; }
    if (touchdownSinkFpm > 400) { verdict = "Hard landing"; col = HUD_RED; }
    if (!touchdownOnRunway) { verdict += " - but off the runway!"; col = HUD_AMBER; }
    hudText(W / 2, y + 42 * s, "LANDED", FONT_BIG, HUD_WHITE, ALIGN_CENTER);
    hudText(W / 2, y + 70 * s, verdict, FONT_BOLD, col, ALIGN_CENTER);
    const char* labels[5] = { "Touchdown speed", "Sink rate at touchdown", "Touchdown after the threshold", "Ground roll",
                              "From 50 ft to full stop" };
    std::string values[5] = { fmt("%.0f kt", touchdownSpeedKt), fmt("%.0f ft/min", touchdownSinkFpm),
                              fmt("%.0f m", -touchdownZ),
                              fmt("%.0f m", groundRollDist) + fmt("  (POH %.0f m)", POH_GROUND_ROLL),
                              fmt("%.0f m", distSince50ft) };
    for (int i = 0; i < 5; i++) {
        float ry = y + 108 * s + i * 28 * s;
        hudText(x + 24 * s, ry, labels[i], FONT_NORMAL, HUD_GREY);
        hudText(x + w - 24 * s, ry, values[i], FONT_BOLD, HUD_WHITE, ALIGN_RIGHT);
    }
    keyChip(x + w / 2 - keyChipWidth("SPACE", "fly again") / 2, y + h - 38 * s, "SPACE", "fly again");
}

// card shown after a crash, with the reason
void drawCrashCard(float W, float H)
{
    float s = hudScale(), w = 460 * s, h = 150 * s, x = (W - w) / 2, y = H - h - 80 * s;
    hudRoundRect(x, y, w, h, 14 * s, glm::vec4(0.25f, 0.04f, 0.03f, 0.88f));
    hudText(W / 2, y + 44 * s, "CRASHED", FONT_BIG, HUD_WHITE, ALIGN_CENTER);
    hudText(W / 2, y + 76 * s, crashReason, FONT_BOLD, glm::vec4(1.0f, 0.75f, 0.65f, 1.0f), ALIGN_CENTER);
    keyChip(x + w / 2 - keyChipWidth("SPACE", "try again") / 2, y + h - 42 * s, "SPACE", "try again");
}

// one column of the help card, a title and then rows with a key and what it does
void helpSection(float x, float& y, const char* title, const char* const rows[][2], int n)
{
    float s = hudScale();
    hudText(x, y, title, FONT_BOLD, HUD_BLUE);
    y += 14 * s;
    for (int i = 0; i < n; i++) {
        keyChip(x, y, rows[i][0], "");
        hudText(x + 150 * s, y + 17.5f * s, rows[i][1], FONT_NORMAL, HUD_WHITE);
        y += 30 * s;
    }
    y += 22 * s;
}

// help card with all the keys, shown at the start and with H
void drawHelp(float W, float H)
{
    float s = hudScale(), w = 900 * s, h = 600 * s, x = (W - w) / 2, y = (H - h) / 2;
    hudRect(0, 0, W, H, glm::vec4(0, 0, 0, 0.45f));
    hudRoundRect(x, y, w, h, 16 * s, glm::vec4(0.06f, 0.08f, 0.11f, 0.94f));
    hudText(W / 2, y + 48 * s, "AIRCRAFT LANDING SIMULATOR", FONT_BIG, HUD_WHITE, ALIGN_CENTER);
    hudText(W / 2, y + 76 * s, "A Piper PA-28 landing, following the real numbers of its flight manual",
            FONT_NORMAL, HUD_GREY, ALIGN_CENTER);

    static const char* const sim[][2] = { { "SPACE", "start or pause" }, { "R", "restart the approach" },
                                          { "+  -", "simulation speed (x0.25 to x8)" }, { "N", "switch day and night" },
                                          { "H", "open or close this help" }, { "ESC", "quit" } };
    static const char* const cam[][2] = { { "C", "switch camera" }, { "1  2  3  4", "chase, tower, cockpit or free" },
                                          { "Left drag", "look around" }, { "Wheel", "zoom in and out" },
                                          { "W A S D  Q E", "move the free camera" } };
    static const char* const man[][2] = { { "M", "turn manual control on / off" }, { "W  S", "nose down / nose up" },
                                          { "A  D", "turn left / right" }, { "Q  E", "less / more power" },
                                          { "F  V", "flaps down / up" }, { "B", "brakes on / off" },
                                          { "Right drag", "fly with the mouse" } };
    float y1 = y + 125 * s, y2 = y1;
    helpSection(x + 40 * s, y1, "SIMULATION", sim, 6);
    helpSection(x + 470 * s, y2, "CAMERAS", cam, 5);
    helpSection(x + 470 * s, y2, "FLY IT YOURSELF", man, 7);

    float bw = keyChipWidth("SPACE", "start the landing") + keyChipWidth("H", "close this help") + 30 * s;
    float bx = (W - bw) / 2;
    bx += keyChip(bx, y + h - 46 * s, "SPACE", "start the landing", HUD_GREEN) + 30 * s;
    keyChip(bx, y + h - 46 * s, "H", "close this help");
}

// the whole HUD, drawn after the 3D scene in a single batch
void drawHUD(void)
{
    int w, h;
    glfwGetFramebufferSize(window, &w, &h);
    if (w <= 0 || h <= 0) return;
    float s = glm::clamp(h / 950.0f, 0.75f, 2.2f);
    hudBegin(w, h, s);

    drawStatusBar(16 * s, 16 * s);
    if (!showHelp) {
        drawFlightData(16 * s, 64 * s);
        drawApproachGuide(w - 251 * s, 64 * s);
        if (crashed) drawCrashCard((float)w, (float)h);
        else if (touchdownSpeedKt > 0.0f &&
                 ((phase == STOPPED && !manualMode) ||
                  (manualMode && ac.speed < 0.5f && aircraftOnGround())))
            drawLandingSummary((float)w, (float)h);
        drawKeyBar((float)w, (float)h);
        drawToast((float)w);
    } else {
        drawHelp((float)w, (float)h);
    }
    hudEnd();
}
