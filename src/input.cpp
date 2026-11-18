// input.cpp
//
// Keyboard and mouse with GLFW, read once per frame in handleInput(). The simulation
// keys (start, pause, speed, day or night), the 4 cameras and the controls of the
// manual flight (W S nose, A D bank, Q E throttle, F V flaps, B brakes, right drag as a
// yoke). The mouse also turns and zooms the cameras.

#include <common/landing.hpp>

// state of the cameras
int   cameraMode = CAM_CHASE;
float orbitYaw = 0.0f, orbitPitch = glm::radians(12.0f), orbitDistance = 14.0f;
float freeYaw = glm::radians(-40.0f), freePitch = glm::radians(25.0f), freeDistance = 400.0f;
glm::vec3 freeTarget = glm::vec3(0.0f, 0.0f, AIM_POINT_Z);
float lookYaw = 0.0f, lookPitch = glm::radians(-10.0f);
float towerZoom = 1.0f;          // the tower camera zooms by itself, the wheel only scales it

double scrollOffset = 0.0;
double lastMouseX = 0.0, lastMouseY = 0.0;

// mouse wheel, only added up here and used in handleInput()
void scrollCallback(GLFWwindow* w, double xoffset, double yoffset)
{
    scrollOffset += yoffset;
}

// true only in the frame where the key goes down, for keys that switch something on and off
bool keyHit(int key)
{
    static bool previous[GLFW_KEY_LAST + 1] = { false };
    bool down = glfwGetKey(window, key) == GLFW_PRESS;
    bool hit = down && !previous[key];
    previous[key] = down;
    return hit;
}

bool keyDown(int key) { return glfwGetKey(window, key) == GLFW_PRESS; }

// all the keys and the mouse, once per frame
void handleInput(float dt)
{
    // help and quit
    if (keyHit(GLFW_KEY_ESCAPE)) {
        if (showHelp) showHelp = false;          // ESC first closes the help card
        else quitRequested = true;
    }
    if (keyHit(GLFW_KEY_H)) showHelp = !showHelp;

    // simulation
    if (keyHit(GLFW_KEY_SPACE)) {
        showHelp = false;
        // in manual mode SPACE only pauses and continues the flight
        if ((phase == STOPPED && !manualMode) || crashed) resetSimulation();  // after a landing or a crash SPACE starts again
        simRunning = !simRunning;
        notify(simRunning ? "Running" : "Paused - press SPACE to continue");
    }
    if (keyHit(GLFW_KEY_R)) { resetSimulation(); notify("Back to the start of the final approach"); }
    bool faster = keyHit(GLFW_KEY_EQUAL) || keyHit(GLFW_KEY_KP_ADD);
    bool slower = keyHit(GLFW_KEY_MINUS) || keyHit(GLFW_KEY_KP_SUBTRACT);
    if (faster || slower) {
        timeScale = glm::clamp(timeScale * (faster ? 2.0f : 0.5f), 0.25f, 8.0f);
        notify(fmt("Simulation speed x%g", timeScale));
    }
    if (keyHit(GLFW_KEY_N)) { nightMode = !nightMode; notify(nightMode ? "Night" : "Day"); }
    if (keyHit(GLFW_KEY_M) && !crashed) {
        manualMode = !manualMode;
        if (!manualMode) resumeAutopilot();      // the autopilot takes over at once
        if (manualMode) {
            simRunning = true;
            inPitch = inRoll = 0.0f;
            brakes = false;
        }
        notify(manualMode ? "You have control - W S nose, A D bank, Q E throttle" : (simRunning ? "Autopilot on - continuing the landing" : "Autopilot on - SPACE continues the landing"),
               glm::vec4(1.0f, 0.8f, 0.3f, 1.0f));
    }

    // cameras
    int previousCamera = cameraMode;
    if (keyHit(GLFW_KEY_C)) cameraMode = cameraMode % 4 + 1;
    if (keyHit(GLFW_KEY_1)) cameraMode = CAM_CHASE;
    if (keyHit(GLFW_KEY_2)) cameraMode = CAM_TOWER;
    if (keyHit(GLFW_KEY_3)) cameraMode = CAM_COCKPIT;
    if (keyHit(GLFW_KEY_4)) cameraMode = CAM_FREE;
    if (cameraMode != previousCamera) {
        if (cameraMode == CAM_COCKPIT) { lookYaw = 0.0f; lookPitch = glm::radians(-10.0f); }
        notify(std::string(CAMERA_NAME[cameraMode]) + " camera");
    }

    // The mouse. Dragging with the left button turns the camera around, the wheel zooms,
    // and dragging with the right button is the yoke in manual mode
    double mx, my;
    glfwGetCursorPos(window, &mx, &my);
    float dx = (float)(mx - lastMouseX), dy = (float)(my - lastMouseY);
    lastMouseX = mx; lastMouseY = my;
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        float s = 0.005f;
        if (cameraMode == CAM_CHASE)   { orbitYaw -= dx * s; orbitPitch = glm::clamp(orbitPitch + dy * s, -0.2f, 1.5f); }
        if (cameraMode == CAM_FREE)    { freeYaw  -= dx * s; freePitch  = glm::clamp(freePitch + dy * s, 0.02f, 1.5f); }
        if (cameraMode == CAM_COCKPIT) { lookYaw  -= dx * s; lookPitch  = glm::clamp(lookPitch - dy * s, -1.2f, 1.2f); }
    }
    if (scrollOffset != 0.0) {
        float f = powf(0.9f, (float)scrollOffset);
        if (cameraMode == CAM_CHASE) orbitDistance = glm::clamp(orbitDistance * f, 6.0f, 400.0f);
        if (cameraMode == CAM_FREE)  freeDistance  = glm::clamp(freeDistance * f, 5.0f, 6000.0f);
        if (cameraMode == CAM_TOWER) towerZoom     = glm::clamp(towerZoom * f, 0.1f, 10.0f);
        scrollOffset = 0.0;
    }

    // W A S D move the free camera on the ground plane and Q E move it down and up
    if (cameraMode == CAM_FREE) {
        float v = freeDistance * 0.8f * dt;
        glm::vec3 fwd(-sinf(freeYaw), 0.0f, -cosf(freeYaw)), right(cosf(freeYaw), 0.0f, -sinf(freeYaw));
        if (keyDown(GLFW_KEY_W)) freeTarget += fwd * v;
        if (keyDown(GLFW_KEY_S)) freeTarget -= fwd * v;
        if (keyDown(GLFW_KEY_D)) freeTarget += right * v;
        if (keyDown(GLFW_KEY_A)) freeTarget -= right * v;
        if (keyDown(GLFW_KEY_E)) freeTarget.y += v;
        if (keyDown(GLFW_KEY_Q)) freeTarget.y = glm::max(freeTarget.y - v, 0.0f);
    }

    // Manual flight. W S push and pull the yoke, A D bank (and steer on the ground),
    // Q E throttle, F V flaps and B brakes on or off.
    // In the free camera W A S D Q E move the camera, and the aircraft keeps its controls centred
    if (manualMode) {
        bool fly = (cameraMode != CAM_FREE);
        float p = 0.0f, r = 0.0f;
        if (fly) {
            p = (keyDown(GLFW_KEY_S) ? 1.0f : 0.0f) - (keyDown(GLFW_KEY_W) ? 1.0f : 0.0f);   // pull to raise the nose
            r = (keyDown(GLFW_KEY_A) ? 1.0f : 0.0f) - (keyDown(GLFW_KEY_D) ? 1.0f : 0.0f);   // left wing down, turn left
        }
        // with the right button the mouse is the yoke, up and down for pitch, left and right for roll
        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
            p += glm::clamp(dy * 0.08f, -1.0f, 1.0f);
            r += glm::clamp(-dx * 0.08f, -1.0f, 1.0f);
        }
        // the controls move smoothly, like a real yoke
        approachValue(inPitch, glm::clamp(p, -1.0f, 1.0f), 6.0f, dt);
        approachValue(inRoll,  glm::clamp(r, -1.0f, 1.0f), 6.0f, dt);
        if (fly && keyDown(GLFW_KEY_E)) throttle = glm::min(throttle + 0.5f * dt, 1.0f);
        if (fly && keyDown(GLFW_KEY_Q)) throttle = glm::max(throttle - 0.5f * dt, 0.0f);
        if (keyHit(GLFW_KEY_B)) {
            brakes = !brakes;
            notify(brakes ? "Brakes ON" : "Brakes off");
        }
        // flaps one step down with F or up with V, through 0, 10, 25 and 40 degrees
        const float steps[4] = { 0.0f, 10.0f, 25.0f, 40.0f };
        int current = 0;
        for (int i = 0; i < 4; i++) if (flapsTarget >= steps[i] - 0.1f) current = i;
        int next = current + (keyHit(GLFW_KEY_F) ? 1 : 0) - (keyHit(GLFW_KEY_V) ? 1 : 0);
        next = glm::clamp(next, 0, 3);
        if (next != current) {
            flapsTarget = steps[next];
            notify("Flaps " + std::to_string((int)flapsTarget));
        }
    }
}
