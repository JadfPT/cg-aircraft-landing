// scene.cpp
//
// What is drawn in 3D, apart from the terrain and the sky. The airport with the runway,
// apron, hangars, tower, people and lights, the aircraft with its moving parts (the
// propeller, flaps and ailerons rotate about their hinge lines), its lights and, after a
// crash, the wreck and the debris. Lights get a halo and particles are drawn as
// billboards, squares that always face the camera.

#include <common/landing.hpp>

// Model matrix of the aircraft, M = T(position) Ry(heading) Rx(pitch) Rz(roll).
// Read from right to left, the aircraft is first rolled, then pitched, then turned to its
// heading and at last moved to its position
glm::mat4 aircraftModelMatrix(void)
{
    glm::mat4 M = glm::translate(glm::mat4(1.0f), ac.pos);
    M = glm::rotate(M, ac.heading, glm::vec3(0, 1, 0));
    M = glm::rotate(M, ac.pitch,   glm::vec3(1, 0, 0));
    M = glm::rotate(M, ac.roll,    glm::vec3(0, 0, 1));
    return M;
}

// Box made from the cube of the lab codes (side 2, centred at the origin). It is scaled
// to the size wanted and moved so that its base sits on the point base
glm::mat4 boxModel(const glm::vec3& base, const glm::vec3& size)
{
    glm::mat4 M = glm::translate(glm::mat4(1.0f), base + glm::vec3(0.0f, size.y * 0.5f, 0.0f));
    return glm::scale(M, size * 0.5f);
}

// Halos of the lights. They are drawn at the end with additive blending, so the lights
// are seen from far away and at night, like the bright points of real lights
struct Glow { glm::vec3 pos, color; float size, intensity; };
std::vector<Glow> glows;

void clearGlows(void) { glows.clear(); }

// stores a halo, they are all drawn at the end of the frame by drawGlows()
void addGlow(const glm::vec3& p, const glm::vec3& color, float size, float intensity = 1.0f)
{
    if (renderPass != PASS_OPAQUE) return;      // only collected once per frame
    Glow g = { p, color, size, intensity };
    glows.push_back(g);
}

// a light fixture on the ground, a dark stand with a small lit lamp and its halo
void drawLight(const glm::vec3& p, const glm::vec3& color, float size = 0.6f)
{
    float head = size * 0.4f, stand = 0.3f;
    if (p.y < 0.01f)
        drawModel(cubeModel, boxModel(p, glm::vec3(0.1f, stand, 0.1f)), glm::vec3(0.15f), 0.1f, 8.0f);
    glm::vec3 lamp = p + glm::vec3(0.0f, p.y < 0.01f ? stand : 0.0f, 0.0f);
    drawModel(cubeModel, boxModel(lamp, glm::vec3(head)), color, 0.0f, 1.0f, 1.0f);
    addGlow(lamp + glm::vec3(0.0f, head * 0.5f, 0.0f), color, size * 2.5f, 0.8f);
}

// a lamp post, a pole with a lamp and its halo
void drawLampPost(const glm::vec3& base, float height, const glm::vec3& color)
{
    drawModel(cubeModel, boxModel(base, glm::vec3(0.35f, height, 0.35f)), glm::vec3(0.35f), 0.2f, 8.0f);
    drawModel(cubeModel, boxModel(base + glm::vec3(0, height, 0), glm::vec3(1.2f, 0.4f, 1.2f)), color, 0.0f, 1.0f, nightMode ? 1.0f : 0.0f);
    if (nightMode) addGlow(base + glm::vec3(0, height - 0.3f, 0), color, 6.0f, 0.9f);
}

// matrix of a person or building at p, turned yawDeg degrees (0 looks towards -Z, 90 towards -X)
glm::mat4 personMatrix(const glm::vec3& p, float yawDeg)
{
    return glm::rotate(glm::translate(glm::mat4(1.0f), p), glm::radians(yawDeg), glm::vec3(0, 1, 0));
}

// runway, apron, hangars, tower, people and all the airport lights
void drawAirport(void)
{
    // runway
    drawModel(runwayModel, glm::mat4(1.0f), glm::vec3(1.0f), 0.15f, 16.0f);

    // taxiway and apron, made of flattened cubes
    glm::vec3 concrete(0.55f, 0.55f, 0.53f);
    drawModel(cubeModel, boxModel(glm::vec3(30.0f, -0.1f, -380.0f), glm::vec3(30.0f, 0.13f, 14.0f)), concrete * 0.8f, 0.1f, 8.0f);
    drawModel(cubeModel, boxModel(glm::vec3(86.0f, -0.1f, -390.0f), glm::vec3(84.0f, 0.14f, 210.0f)), concrete, 0.1f, 8.0f);

    // hangars and control tower
    for (int i = 0; i < 3; i++)
        drawModel(hangarModel, personMatrix(HANGAR_POSITION[i], HANGAR_YAW), glm::vec3(1.0f), 0.3f, 16.0f);
    drawModel(towerModel, personMatrix(TOWER_POSITION, 90.0f), glm::vec3(1.0f), 0.3f, 32.0f);

    // ground crew watching the landing from the apron, and a pilot waiting next to the tower
    drawModel(crewModel[0], personMatrix(glm::vec3(48.0f, 0.0f, -372.0f), 90.0f), glm::vec3(1.0f), 0.2f, 16.0f);
    drawModel(crewModel[1], personMatrix(glm::vec3(49.5f, 0.0f, -370.5f), 70.0f), glm::vec3(1.0f), 0.2f, 16.0f);
    drawModel(crewModel[2], personMatrix(glm::vec3(52.0f, 0.0f, -395.0f), 110.0f), glm::vec3(1.0f), 0.2f, 16.0f);
    drawModel(crewModel[3], personMatrix(glm::vec3(80.0f, 0.0f, -330.0f), 60.0f), glm::vec3(1.0f), 0.2f, 16.0f);
    drawModel(pilotStandingModel, personMatrix(TOWER_POSITION + glm::vec3(-7.0f, 0.0f, 4.0f), 80.0f),
              glm::vec3(1.0f), 0.2f, 16.0f);

    if (renderPass != PASS_OPAQUE) return;     // the lights are all opaque, so they are drawn in the first pass only

    // apron floodlights (the point light at night is the first one) and a lamp over each hangar door
    glm::vec3 sodium(1.0f, 0.82f, 0.55f);
    drawLampPost(glm::vec3(60.0f, 0.0f, -380.0f), 20.0f, sodium);
    drawLampPost(glm::vec3(60.0f, 0.0f, -300.0f), 14.0f, sodium);
    drawLampPost(glm::vec3(60.0f, 0.0f, -460.0f), 14.0f, sodium);
    for (int i = 0; i < 3; i++) {
        glm::vec3 door = HANGAR_POSITION[i] + glm::vec3(-14.6f, 7.0f, 0.0f);
        drawModel(cubeModel, boxModel(door - glm::vec3(0, 0.2f, 0), glm::vec3(0.4f, 0.4f, 1.2f)), sodium, 0.0f, 1.0f,
                  nightMode ? 1.0f : 0.0f);
        if (nightMode) addGlow(door, sodium, 5.0f, 0.8f);
    }
    // blue lights along the taxiway
    for (float x = 18.0f; x <= 44.0f; x += 6.5f)
        for (int s = -1; s <= 1; s += 2)
            drawLight(glm::vec3(x, 0.0f, -380.0f + s * 7.5f), glm::vec3(0.2f, 0.4f, 1.0f), 0.35f);
    // on top of the tower a red light and the airport beacon, with white and green flashes
    double t = glfwGetTime();
    glm::vec3 top = TOWER_POSITION + glm::vec3(0.0f, 30.4f, 0.0f);
    addGlow(top, glm::vec3(1.0f, 0.1f, 0.05f), 2.5f, fmod(t, 1.5) < 0.75 ? 1.0f : 0.25f);
    float beaconPhase = (float)fmod(t * 0.5, 1.0);   // one turn every 2 s, white and then green
    if (fmod(t * 4.0, 2.0) < 0.35)
        addGlow(top + glm::vec3(0, 1.2f, 0), beaconPhase < 0.5f ? glm::vec3(1.0f) : glm::vec3(0.2f, 1.0f, 0.35f), 6.0f, 1.0f);

    // white lights along the runway edges, green ones at the threshold and red ones at the end
    float glow = nightMode ? 1.0f : 0.85f;
    float edge = RUNWAY_WIDTH * 0.5f + 0.8f;
    for (float z = 0.0f; z >= -RUNWAY_LENGTH; z -= 60.0f)
        for (int s = -1; s <= 1; s += 2)
            drawLight(glm::vec3(s * edge, 0.0f, z), glm::vec3(1.0f, 0.95f, 0.75f) * glow, 0.5f);
    for (float x = -edge; x <= edge + 0.1f; x += 3.0f) {
        drawLight(glm::vec3(x, 0.0f, 1.5f), glm::vec3(0.1f, 1.0f, 0.3f) * glow, 0.5f);
        drawLight(glm::vec3(x, 0.0f, -RUNWAY_LENGTH - 1.5f), glm::vec3(1.0f, 0.1f, 0.1f) * glow, 0.5f);
    }

    // approach lights before the runway, with a flash that runs towards it
    int rabbit = (int)(glfwGetTime() * 12.0) % 14;
    for (int i = 1; i <= 10; i++) {
        float z = 30.0f * i;
        glm::vec3 c = (i == 10 - rabbit) ? glm::vec3(1.0f) : glm::vec3(1.0f, 0.9f, 0.6f) * 0.8f;
        drawLight(glm::vec3(0.0f, 0.5f, z), c, (i == 10 - rabbit) ? 1.2f : 0.7f);
        drawModel(cubeModel, boxModel(glm::vec3(0.0f, -0.1f, z), glm::vec3(0.15f, 0.6f, 0.15f)), glm::vec3(0.3f));
    }
    for (float x = -12.0f; x <= 12.1f; x += 2.0f)
        drawLight(glm::vec3(x, 0.5f, 150.0f), glm::vec3(1.0f, 0.9f, 0.6f) * 0.8f, 0.6f);

    // the PAPI, each light white above its angle and red below
    for (int i = 0; i < 4; i++) {
        glm::vec3 p(PAPI_X[i], 0.0f, AIM_POINT_Z);
        drawModel(cubeModel, boxModel(p, glm::vec3(2.2f, 0.6f, 1.2f)), glm::vec3(0.25f), 0.2f, 8.0f);
        glm::vec3 c = papiIsWhite(i) ? glm::vec3(1.0f) : glm::vec3(1.0f, 0.08f, 0.05f);
        drawLight(p + glm::vec3(0.0f, 0.6f, 0.0f), c, 1.0f);
    }
}

// Rotation of a flap or aileron about its hinge line. As in the lab, a rotation about an
// axis that does not pass through the origin is a translation to the origin, the rotation,
// and the translation back
glm::mat4 hingeMatrix(const glm::vec3 hinge[2], float angleDeg)
{
    glm::vec3 axis = glm::normalize(hinge[1] - hinge[0]);
    glm::mat4 R = glm::translate(glm::mat4(1.0f), hinge[0]);
    R = glm::rotate(R, glm::radians(angleDeg), axis);
    return glm::translate(R, -hinge[0]);
}

// moving parts of the aircraft, 0 is the propeller, 1 and 2 the flaps, 3 and 4 the ailerons
const Model& partModel(int part)
{
    return part == 0 ? propellerModel : (part <= 2 ? flapModel[part - 1] : aileronModel[part - 3]);
}

// World matrix of a moving part. The propeller spins about its hub (z axis), the flaps
// and ailerons rotate about their hinge lines. The hinge axis goes from the root to the
// tip, so the sign of the flap rotation depends on the side (x < 0 is the left wing)
glm::mat4 partMatrix(int part)
{
    glm::mat4 M = aircraftModelMatrix();
    if (part == 0) {
        glm::mat4 P = glm::translate(M, propellerHub);
        P = glm::rotate(P, glm::radians(propAngle), glm::vec3(0, 0, 1));
        return glm::translate(P, -propellerHub);
    }
    if (part <= 2) {
        int s = part - 1;
        return M * hingeMatrix(flapHinge[s], (s == 0 ? -1.0f : 1.0f) * ac.flaps);
    }
    return M * hingeMatrix(aileronHinge[part - 3], ac.aileron);
}

// after a crash, the burnt aircraft and the debris, each one with its own translation and rotation
void drawWreck(void)
{
    glm::mat4 M = aircraftModelMatrix();
    glm::vec3 burnt(0.22f, 0.20f, 0.19f);
    drawModel(aircraftModel, M, burnt, 0.1f, 8.0f);
    drawModel(pilotModel, glm::translate(M, PILOT_HIP), burnt, 0.1f, 8.0f);
    for (size_t i = 0; i < debris.size(); i++) {
        const Debris& d = debris[i];
        glm::mat4 T = glm::translate(glm::mat4(1.0f), d.center + d.offset);
        T = glm::rotate(T, d.angle, d.axis);
        T = glm::translate(T, -d.center) * d.M0;
        drawModel(partModel(d.part), T, burnt * 1.5f, 0.2f, 16.0f);
    }
}

// the landing light is on while flying and rolling with the engine running
bool landingLightOn(void)
{
    return rpm > 50.0f && !crashed && !(phase == STOPPED && !manualMode);
}

// the aircraft, its moving parts, the pilot (or his hands in the cockpit view) and its lights
void drawAircraft(void)
{
    if (crashed) { drawWreck(); return; }
    glm::mat4 M = aircraftModelMatrix();
    glm::vec3 white(1.0f);
    drawModel(aircraftModel, M, white, 0.6f, 48.0f);

    // propeller, flaps (trailing edge down) and ailerons (left one up and right one down to roll left)
    for (int part = 0; part < 5; part++)
        drawModel(partModel(part), partMatrix(part), white, 0.6f, 48.0f);

    // pilot in the left seat, not drawn when we are looking through his eyes
    if (cameraMode != CAM_COCKPIT)
        drawModel(pilotModel, glm::translate(M, PILOT_HIP), white, 0.2f, 16.0f);

    // in the cockpit view his hands on the yoke and on the throttle, already in aircraft
    // coordinates in hands.obj
    if (cameraMode == CAM_COCKPIT)
        drawModel(handsModel, M, white, 0.2f, 16.0f);

    // Lights of the aircraft, on while the engine runs. Red on the left wing tip, green on
    // the right one and white on the tail, a red beacon on the fin that flashes, white
    // strobes on the wing tips with a double flash, and the landing light in the nose
    if (rpm > 50.0f && cameraMode != CAM_COCKPIT) {
        double t = glfwGetTime();
        // they stay on in daylight too, as in a real aircraft, but show much less
        float day = nightMode ? 1.0f : 0.3f;     // steady lights
        float flash = nightMode ? 1.0f : 0.7f;   // the beacon and strobes are made to be seen by day
        // the positions are the lenses of the lights in the PA-28 model
        addGlow(glm::vec3(M * glm::vec4(LIGHT_NAV_LEFT, 1)), glm::vec3(1.0f, 0.1f, 0.08f), 0.35f, day);
        addGlow(glm::vec3(M * glm::vec4(LIGHT_NAV_RIGHT, 1)), glm::vec3(0.1f, 1.0f, 0.3f), 0.35f, day);
        addGlow(glm::vec3(M * glm::vec4(LIGHT_TAIL, 1)), glm::vec3(1.0f), 0.3f, day);
        if (fmod(t, 1.0) < 0.12)
            addGlow(glm::vec3(M * glm::vec4(LIGHT_BEACON, 1)), glm::vec3(1.0f, 0.15f, 0.1f), 0.8f, flash);
        double st = fmod(t, 1.3);
        if (st < 0.05 || (st > 0.15 && st < 0.2)) {
            addGlow(glm::vec3(M * glm::vec4(LIGHT_NAV_LEFT + glm::vec3(0, 0, 0.12f), 1)), glm::vec3(1.0f), 1.2f, flash);
            addGlow(glm::vec3(M * glm::vec4(LIGHT_NAV_RIGHT + glm::vec3(0, 0, 0.12f), 1)), glm::vec3(1.0f), 1.2f, flash);
        }
        if (landingLightOn())
            addGlow(glm::vec3(M * glm::vec4(LIGHT_LANDING, 1)), glm::vec3(1.0f, 0.97f, 0.9f), 0.5f, 1.0f);
    }
}

// Billboard, a square at p that always faces the camera. Its axes are the right and up
// vectors of the camera, which are the first two rows of the view matrix
glm::mat4 billboardMatrix(const glm::vec3& p, float size)
{
    glm::vec3 right(View[0][0], View[1][0], View[2][0]), up(View[0][1], View[1][1], View[2][1]);
    glm::mat4 B(glm::vec4(right, 0), glm::vec4(up, 0), glm::vec4(glm::cross(right, up), 0), glm::vec4(0, 0, 0, 1));
    return glm::translate(glm::mat4(1.0f), p) * B * glm::scale(glm::mat4(1.0f), glm::vec3(size));
}

// halos with additive blending (light adds up), never smaller than about 0.25 deg on
// screen, so far lights are still seen
void drawGlows(void)
{
    if (glows.empty() || !particleModel.loaded) return;
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);            // additive blending
    for (size_t i = 0; i < glows.size(); i++) {
        const Glow& g = glows[i];
        glm::vec3 toCam = cameraPosition - g.pos;
        float d = glm::length(toCam);
        // at least about 0.25 deg on the screen, so far lights are still visible points
        float size = glm::max(g.size * (nightMode ? 1.0f : 0.6f), d * 0.0045f);
        // a few cm towards the camera, so the halo is not hidden inside its own lamp
        glm::vec3 p = g.pos + toCam / glm::max(d, 0.001f) * glm::min(0.15f, d * 0.5f);
        particleModel.meshes[0].alpha = glm::clamp(g.intensity * (nightMode ? 0.95f : 0.55f), 0.0f, 0.99f);
        drawModel(particleModel, billboardMatrix(p, size), g.color, 0.0f, 1.0f, 1.0f);
    }
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

// particles, billboards drawn from the farthest to the closest since they are transparent
bool fartherFirst(const Particle& a, const Particle& b)
{
    return glm::length(a.pos - cameraPosition) > glm::length(b.pos - cameraPosition);
}

void drawParticles(void)
{
    if (particles.empty() || !particleModel.loaded) return;
    std::sort(particles.begin(), particles.end(), fartherFirst);
    for (size_t i = 0; i < particles.size(); i++) {
        const Particle& q = particles[i];
        float t = q.age / q.life;
        float size = glm::mix(q.size0, q.size1, sqrtf(t));
        float alpha = q.alpha * (1.0f - t) * glm::smoothstep(0.0f, 0.08f, t);
        particleModel.meshes[0].alpha = glm::clamp(alpha, 0.0f, 0.99f);
        drawModel(particleModel, billboardMatrix(q.pos, size), glm::mix(q.color0, q.color1, t), 0.0f, 1.0f, q.emissive);
    }
}

// everything except the sky and the terrain, called once for each pass
void drawScene(void)
{
    drawAirport();
    drawAircraft();
}
