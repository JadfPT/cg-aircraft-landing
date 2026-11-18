// simulation.cpp
//
// What moves in the simulation. The automatic landing (simulationStep) goes through the
// phases of the procedure, approach, flare, touchdown and rollout, with the TOLD data of
// landing.hpp. The manual flight (manualFlightStep) turns the keys into rotations about
// the aircraft's own axes. Both are integrated in small fixed steps of 5 ms, so they
// behave the same at any frame rate and at any simulation speed.
// The collisions, the crash, the particles and the PAPI are also here.

#include <common/landing.hpp>

Aircraft ac;
Phase phase = READY;
bool  simRunning = false;
bool  manualMode = false;
float timeScale = 1.0f;
float simTime = 0.0f;
float rolloutTime = 0.0f;
float propAngle = 0.0f;
float rpm = RPM_IDLE;
float flapsTarget = FLAPS_START_DEG;
bool  nightMode = false;

// messages on the screen
std::string toastText;
glm::vec4   toastColor(1.0f);
double      toastTime = -100.0;
bool        showHelp = true;          // the help card is open when the program starts
bool        quitRequested = false;

// formats one number like printf and returns it as a string
std::string fmt(const char* f, double v) { char b[96]; snprintf(b, sizeof(b), f, v); return b; }

// shows a message at the top of the screen, drawToast() fades it out after a few seconds
void notify(const std::string& text, const glm::vec4& color)
{
    toastText = text;
    toastColor = color;
    toastTime = glfwGetTime();
}

// what is recorded during the landing, for the summary at the end
bool  passed50ft = false;
float touchdownZ = 0.0f, touchdownSinkFpm = 0.0f, touchdownSpeedKt = 0.0f;
bool  touchdownOnRunway = true;
float groundRollDist = 0.0f, distSince50ft = 0.0f;   // metres travelled
float brakingDecel = 0.0f;         // m/s^2, worked out at touchdown
bool  tyreSmokeBurst = false;      // true in the moment the wheels touch the ground

// crash
bool        crashed = false;
std::string crashReason;
double      crashTime = 0.0;
glm::vec3   wreckPosition(0.0f);

// particles of the tyre smoke, the fire and the smoke of the wreck
std::vector<Particle> particles;

float frand(float a, float b) { return a + (b - a) * (float)rand() / (float)RAND_MAX; }
// random direction (unit vector) for particles and debris
glm::vec3 randomDir(void)
{
    glm::vec3 d(frand(-1, 1), frand(-1, 1), frand(-1, 1));
    return glm::length(d) > 1e-3f ? glm::normalize(d) : glm::vec3(0, 1, 0);
}

void addParticle(const glm::vec3& pos, const glm::vec3& vel, float life, float size0, float size1,
                 const glm::vec3& color0, const glm::vec3& color1, float alpha, float emissive, float buoyancy)
{
    if (particles.size() >= 3000) return;
    Particle q = { pos, vel, 0.0f, life, size0, size1, color0, color1, alpha, emissive, buoyancy };
    particles.push_back(q);
}

// light grey smoke of the tyres. It is thrown a bit forward, spreads and fades
void spawnTyreSmoke(const glm::vec3& p, const glm::vec3& vel, int count, float strength)
{
    for (int i = 0; i < count; i++)
        addParticle(p + glm::vec3(frand(-0.2f, 0.2f), frand(0.0f, 0.2f), frand(-0.2f, 0.2f)),
                    vel * 0.25f + glm::vec3(frand(-1, 1), frand(0.3f, 1.5f), frand(-1, 1)) * strength,
                    frand(2.5f, 4.5f), 0.6f, frand(4.0f, 7.0f) * strength, glm::vec3(0.88f), glm::vec3(0.88f),
                    0.6f * glm::min(strength, 1.0f), 0.35f, 0.4f);
}

// fire, short lived, from yellow to red. It is not lit by the lights (emissive) and rises fast
void spawnFire(const glm::vec3& p, int count, float speed, float size)
{
    for (int i = 0; i < count; i++)
        addParticle(p + randomDir() * frand(0.0f, 1.5f),
                    randomDir() * frand(0.3f, 1.0f) * speed + glm::vec3(0, speed * 0.4f, 0),
                    frand(0.6f, 1.4f), size * 0.6f, size * frand(1.2f, 2.0f),
                    glm::vec3(1.0f, 0.85f, 0.35f), glm::vec3(0.9f, 0.25f, 0.05f), 0.9f, 1.0f, 3.0f);
}

// smoke of the wreck, from dark to grey. It is lit and grows a lot
void spawnSmoke(const glm::vec3& p, int count, float size)
{
    for (int i = 0; i < count; i++)
        addParticle(p + glm::vec3(frand(-1, 1), frand(0, 1), frand(-1, 1)),
                    glm::vec3(frand(-0.5f, 0.5f), frand(1.5f, 3.0f), frand(-0.5f, 0.5f)),
                    frand(4.0f, 7.0f), size, size * 5.0f,
                    glm::vec3(0.18f, 0.16f, 0.15f), glm::vec3(0.45f, 0.45f, 0.47f), 0.7f, 0.0f, 0.5f);
}

// moves the particles with air drag and buoyancy, and removes the old ones
void updateParticles(float dt)
{
    for (size_t i = 0; i < particles.size(); ) {
        Particle& q = particles[i];
        q.age += dt;
        if (q.age >= q.life) { particles[i] = particles.back(); particles.pop_back(); continue; }
        q.vel *= expf(-1.2f * dt);                 // air drag
        q.vel.y += q.buoyancy * dt;
        q.pos += q.vel * dt;
        i++;
    }
}

// debris, the moving parts of the aircraft thrown away in a crash
std::vector<Debris> debris;

// Thrust of the propeller, from 0 at idle to the full value, falling with the speed
float thrustAccel(float throttle, float v)
{
    return throttle * glm::max(THRUST_STATIC - THRUST_LOSS * v, 0.0f);
}

// Drag, the parasite part (with the flaps) plus the induced part. On the wheels there is
// no lift, so there is no induced drag either
float dragAccel(float v, float flaps, bool onGround)
{
    float half = 0.5f * G / GLIDE_RATIO;                 // each part at the best glide speed
    float r = (v * v) / (V_BEST_GLIDE * V_BEST_GLIDE);
    float parasite = half * r * (1.0f + FLAPS_DRAG * flaps / 40.0f);
    return onGround ? parasite : parasite + half / glm::max(r, 0.2f);
}

// throttle the autopilot needs for an acceleration along the path, given the vertical speed
float throttleFor(float accel, float v, float vs, float flaps)
{
    float need = accel + dragAccel(v, flaps, false) + G * vs / glm::max(v, 1.0f);
    return glm::clamp(need / glm::max(thrustAccel(1.0f, v), 0.1f), 0.0f, 1.0f);
}

// puts the aircraft back at the start of the final approach, on the 3 deg glide slope,
// 2000 m from the aiming point and 45 m off the centre line
void resetSimulation(void)
{
    float gs = glm::radians(GLIDE_SLOPE_DEG);
    ac.pos     = glm::vec3(START_LATERAL, START_DISTANCE * tanf(gs), AIM_POINT_Z + START_DISTANCE);
    ac.heading = 0.0f;
    ac.speed   = V_START;
    ac.vs      = -V_START * sinf(gs);
    ac.pitch   = glm::radians(APPROACH_AOA_DEG - GLIDE_SLOPE_DEG);
    ac.roll    = 0.0f;
    ac.flaps   = FLAPS_START_DEG;
    ac.aileron = 0.0f;
    flapsTarget = FLAPS_START_DEG;
    phase = READY;
    simRunning = false;
    simTime = rolloutTime = 0.0f;
    // the throttle that holds the speed on the glide slope, so it starts trimmed
    throttle = throttleFor(0.0f, ac.speed, ac.vs, ac.flaps);
    rpm = RPM_IDLE + (RPM_FULL - RPM_IDLE) * throttle;
    passed50ft = false;
    touchdownOnRunway = true;
    touchdownSpeedKt = touchdownSinkFpm = 0.0f;
    groundRollDist = distSince50ft = 0.0f;
    crashed = false;
    particles.clear();
    debris.clear();
    printf("\n[RESET] Aircraft on final approach, %.0f m from the aiming point, %.0f ft, %.0f kt, flaps %.0f. "
           "Press SPACE.\n", START_DISTANCE, ac.pos.y / FT, ac.speed / KT, ac.flaps);
}

// keeps an angle in [-pi, pi], so a heading difference always takes the short way
float wrapAngle(float a)
{
    while (a >  3.14159265f) a -= 2.0f * 3.14159265f;
    while (a < -3.14159265f) a += 2.0f * 3.14159265f;
    return a;
}

// Moves value smoothly towards target. Each step covers a part k h of what is missing,
// so it slows down as it gets close
void approachValue(float& value, float target, float k, float h)
{
    value += (target - value) * glm::min(1.0f, k * h);
}

// moves value towards target at a constant rate, in units per second
void moveTowards(float& value, float target, float rate, float h)
{
    value += glm::clamp(target - value, -rate * h, rate * h);
}

// One step of h seconds of the automatic landing
void simulationStep(float h)
{
    float gs = glm::radians(GLIDE_SLOPE_DEG);
    float agl = ac.pos.y;
    bool onGround = (phase == ROLLOUT || phase == STOPPED);

    // Lateral guidance, like a localizer. The aircraft aims at a point on the centre line
    // some seconds ahead of it, so it joins the line smoothly
    float lookAhead = glm::max(ac.speed * 8.0f, 30.0f);
    glm::vec2 target(0.0f, ac.pos.z - lookAhead);
    glm::vec2 d = target - glm::vec2(ac.pos.x, ac.pos.z);
    float desiredHeading = atan2f(-d.x, -d.y);
    float maxRate = glm::radians(onGround ? 10.0f : 5.0f);
    float turnRate = glm::clamp(0.6f * wrapAngle(desiredHeading - ac.heading), -maxRate, maxRate);
    if (phase == STOPPED) turnRate = 0.0f;
    ac.heading = wrapAngle(ac.heading + turnRate * h);

    // in a coordinated turn the bank angle comes from tan(roll) = V w / g
    float rollTarget = onGround ? 0.0f : atanf(ac.speed * turnRate / G);
    float rollRate = (rollTarget - ac.roll) * glm::min(1.0f, 3.0f * h) / h;
    approachValue(ac.roll, rollTarget, 3.0f, h);
    // the ailerons move with the roll rate that is being asked
    approachValue(ac.aileron, glm::clamp(glm::degrees(rollRate) * 1.5f, -15.0f, 15.0f), 6.0f, h);

    // Vertical guidance and power, depending on the phase. Like a pilot, the autopilot
    // follows the glide slope with the pitch and holds the speed with the throttle
    float pitchTarget = ac.pitch;
    float throttleCmd = 0.0f;
    switch (phase) {
    case APPROACH: {
        float distToAim = ac.pos.z - AIM_POINT_Z;
        float yGlideSlope = distToAim * tanf(gs);
        float vsCmd = glm::clamp(-ac.speed * sinf(gs) + 0.5f * (yGlideSlope - agl), -6.0f, 3.0f);
        approachValue(ac.vs, vsCmd, 2.0f, h);
        pitchTarget = asinf(glm::clamp(ac.vs / ac.speed, -1.0f, 1.0f)) + glm::radians(APPROACH_AOA_DEG);
        // the throttle for a gentle acceleration towards 63 kt, and over the threshold the
        // power comes off little by little, so it is at idle when the flare starts
        float accelCmd = glm::clamp(0.25f * (V_APPROACH - ac.speed), -0.5f, 0.3f);
        float powerOff = glm::clamp((agl - FLARE_HEIGHT) / (POWER_OFF_HEIGHT - FLARE_HEIGHT), 0.0f, 1.0f);
        throttleCmd = throttleFor(accelCmd, ac.speed, ac.vs, ac.flaps) * powerOff;
        if (agl < 300.0f * FT && flapsTarget < FLAPS_LANDING_DEG) {
            flapsTarget = FLAPS_LANDING_DEG;
            printf("[FLAPS 40]  t=%5.1f s  %.0f ft, full flaps\n", simTime, agl / FT);
            notify("Flaps 40 - full flaps, slowing to 63 kt");
        }
        if (agl <= FLARE_HEIGHT) {
            phase = FLARE;
            notify("Flare - power to idle, nose up", glm::vec4(1.0f, 0.8f, 0.3f, 1.0f));
            printf("[FLARE]     t=%5.1f s  height %.0f ft, %.0f kt, %.0f fpm, %.0f m before the aiming point\n",
                   simTime, agl / FT, ac.speed / KT, ac.vs / FT * 60.0f, distToAim);
        }
        break;
    }
    case FLARE: {
        // the sink rate is proportional to the height (exponential flare), with a minimum
        float k = V_APPROACH * sinf(gs) / FLARE_HEIGHT;
        float vsCmd = -glm::max(k * agl, FLARE_MIN_SINK);
        approachValue(ac.vs, vsCmd, 4.0f, h);
        float t = glm::clamp((FLARE_HEIGHT - agl) / FLARE_HEIGHT, 0.0f, 1.0f);
        pitchTarget = glm::radians(glm::mix(APPROACH_AOA_DEG - GLIDE_SLOPE_DEG, FLARE_PITCH_DEG, t));
        throttleCmd = 0.0f;        // idle, the drag slows the aircraft down towards the stall speed
        break;
    }
    case ROLLOUT: {
        rolloutTime += h;
        ac.vs = 0.0f;
        if (rolloutTime < DEROTATION_TIME) {
            pitchTarget = glm::radians(FLARE_PITCH_DEG) * (1.0f - rolloutTime / DEROTATION_TIME);
            ac.speed -= 0.5f * h;                  // only rolling resistance and drag
        } else {
            pitchTarget = 0.0f;
            ac.speed -= brakingDecel * h;          // brakes
        }
        if (ac.speed <= 0.3f) {
            ac.speed = 0.0f;
            phase = STOPPED;
            flapsTarget = 0.0f;                    // after landing the flaps go up
            notify("Aircraft stopped - flaps up", glm::vec4(0.5f, 1.0f, 0.6f, 1.0f));
            printf("[STOPPED]   t=%5.1f s  ground roll %.0f m (POH %.0f m); landing distance from 50 ft %.0f m\n",
                   simTime, groundRollDist, POH_GROUND_ROLL, distSince50ft);
            printf("            stopped %.0f m before the end of the runway\n", ac.pos.z + RUNWAY_LENGTH);
        }
        break;
    }
    default:
        break;
    }
    approachValue(ac.pitch, pitchTarget, 2.5f, h);
    moveTowards(ac.flaps, flapsTarget, FLAPS_RATE, h);

    // In the air the speed comes from thrust, drag and the weight along the path. The pilot
    // moves the throttle smoothly and the engine follows it
    if (!onGround) {
        approachValue(throttle, throttleCmd, 1.5f, h);
        ac.speed += (thrustAccel(throttle, ac.speed) - dragAccel(ac.speed, ac.flaps, false)
                     - G * ac.vs / ac.speed) * h;
    } else
        approachValue(throttle, 0.0f, 3.0f, h);
    if (phase != STOPPED) approachValue(rpm, RPM_IDLE + (RPM_FULL - RPM_IDLE) * throttle, 3.0f, h);

    // the aircraft moves along its heading
    ac.pos.x += -ac.speed * sinf(ac.heading) * h;
    ac.pos.z += -ac.speed * cosf(ac.heading) * h;
    if (!onGround) ac.pos.y += ac.vs * h;

    if (ac.pos.y <= 50.0f * FT) passed50ft = true;
    if (passed50ft) distSince50ft += ac.speed * h;
    if (phase == ROLLOUT) groundRollDist += ac.speed * h;

    // touchdown, the moment the wheels reach the runway
    if ((phase == APPROACH || phase == FLARE) && ac.pos.y <= 0.0f) {
        ac.pos.y = 0.0f;
        touchdownZ = ac.pos.z;
        touchdownSinkFpm = -ac.vs / FT * 60.0f;
        touchdownSpeedKt = ac.speed / KT;
        groundRollDist = 0.0f;
        // The braking deceleration comes from v^2 = 2 a d. While the nose comes down the
        // aircraft rolls for t seconds slowing down only 0.5 m/s^2, and the rest of the POH
        // ground roll is left for the brakes
        {
            float t = DEROTATION_TIME, v2 = ac.speed - 0.5f * t;
            float rolled = ac.speed * t - 0.25f * t * t;
            brakingDecel = v2 * v2 / (2.0f * (POH_GROUND_ROLL - rolled));
        }
        phase = ROLLOUT;
        rolloutTime = 0.0f;
        touchdownOnRunway = true;
        tyreSmokeBurst = true;
        notify(touchdownSinkFpm < 300 ? "Touchdown - smooth landing!" : "Touchdown - hard landing",
               touchdownSinkFpm < 300 ? glm::vec4(0.5f, 1.0f, 0.6f, 1.0f) : glm::vec4(1.0f, 0.4f, 0.3f, 1.0f));
        printf("[TOUCHDOWN] t=%5.1f s  %.0f kt, sink %.0f fpm, %.0f m past the threshold (%s)\n",
               simTime, touchdownSpeedKt, touchdownSinkFpm, -touchdownZ,
               touchdownSinkFpm < 300 ? "smooth landing" : "HARD LANDING");
    }
}

// Manual flight, where the user flies the aircraft with the keyboard and the mouse.
// The controls act about the aircraft's own axes, the aircraft always moves where
// its nose points and banking makes it turn (coordinated turn)
float inPitch = 0.0f, inRoll = 0.0f;   // from -1 to 1, elevator and ailerons (nose wheel on the ground)
float throttle = 0.5f;                               // from 0 to 1
bool  brakes = false;

// buildings the aircraft can hit, each one a box given by its centre and half size
std::string hitBuilding(const glm::vec3& p)
{
    for (int i = 0; i < 3; i++) {
        glm::vec3 c = HANGAR_POSITION[i] + glm::vec3(0.0f, 4.25f, 0.0f), e(14.3f, 4.25f, 11.7f);
        if (glm::all(glm::lessThan(glm::abs(p - c), e))) return "a hangar";
    }
    glm::vec3 t = TOWER_POSITION + glm::vec3(0.0f, 15.0f, 0.0f);
    if (glm::all(glm::lessThan(glm::abs(p - t), glm::vec3(3.9f, 15.0f, 3.9f)))) return "the control tower";
    return "";
}

// The crash. There is an explosion of fire particles, the moving parts fly away as
// debris with their own spin, and the engine stops
void crash(const std::string& reason)
{
    if (crashed) return;
    crashed = true;
    crashReason = reason;
    crashTime = glfwGetTime();
    glm::mat4 M = aircraftModelMatrix();
    wreckPosition = glm::vec3(M * glm::vec4(0.0f, 1.0f, -1.0f, 1.0f));
    glm::vec3 vel = glm::vec3(M * glm::vec4(0, 0, -1, 0)) * ac.speed;

    // a ball of fire, then the wreck keeps burning (see updateSimulation)
    spawnFire(wreckPosition, 160, 9.0f, 3.0f);
    spawnSmoke(wreckPosition, 30, 2.0f);
    // the moving parts fly away
    debris.clear();
    for (int part = 0; part < 5; part++) {
        Debris d;
        d.part = part;
        d.M0 = partMatrix(part);
        const Model& m = partModel(part);
        d.center = glm::vec3(d.M0 * glm::vec4(0.5f * (m.bbmin + m.bbmax), 1.0f));
        d.offset = glm::vec3(0.0f);
        d.vel = vel * 0.4f + randomDir() * frand(6.0f, 14.0f) + glm::vec3(0, frand(6.0f, 12.0f), 0);
        d.axis = randomDir();
        d.angle = 0.0f;
        d.spin = frand(4.0f, 12.0f);
        d.resting = false;
        debris.push_back(d);
    }
    ac.speed = 0.0f;
    ac.vs = 0.0f;
    rpm = 0.0f;                       // the engine is destroyed
    manualMode = false;
    simRunning = false;
    notify("CRASH - " + reason, glm::vec4(1.0f, 0.35f, 0.3f, 1.0f));
    printf("[CRASH]     %s\n", reason.c_str());
}

// One step of the manual flight. Rotations from the controls, speed from the thrust
// minus drag, gravity and brakes, a move along the nose, and then the collisions
void manualFlightStep(float h)
{
    // The flaps lower the stall speed, about 50 kt without flaps and 44 kt with 40 deg in
    // the PA-28, and so the rotation speed too
    const float STALL = glm::mix(V_STALL_CLEAN, V_STALL_FLAPS, ac.flaps / 40.0f);
    const float VR    = STALL + 5.0f * KT;          // below the rotation speed (55 kt without flaps) the wheels stay on the ground
    float ground = groundHeight(ac.pos.x, ac.pos.z);
    bool onGround = ac.pos.y <= ground + 0.05f;

    // the orientation as a matrix, R = Ry(heading) Rx(pitch) Rz(roll), as in the model matrix
    glm::mat4 R = glm::eulerAngleYXZ(ac.heading, ac.pitch, ac.roll);

    // Rotations about the aircraft's own axes. Multiplying on the right rotates in the
    // aircraft frame. x goes along the wings (pitch), z along the fuselage (roll), y up (yaw)
    R = R * glm::rotate(glm::mat4(1.0f), glm::radians(25.0f) * inPitch * h, glm::vec3(1, 0, 0));
    if (!onGround)
        R = R * glm::rotate(glm::mat4(1.0f), glm::radians(45.0f) * inRoll * h, glm::vec3(0, 0, 1));
    else      // on the wheels A and D steer the nose wheel
        R = R * glm::rotate(glm::mat4(1.0f), glm::radians(25.0f) * inRoll * h, glm::vec3(0, 1, 0));

    // In the air the bank angle turns the aircraft, w = g tan(bank) / V. This rotation is
    // about the world vertical, so it multiplies on the left
    if (!onGround && ac.speed > 5.0f) {
        float bank = glm::clamp(ac.roll, -1.3f, 1.3f);
        float turnRate = G * tanf(bank) / ac.speed;
        R = glm::rotate(glm::mat4(1.0f), turnRate * h, glm::vec3(0, 1, 0)) * R;
    }
    // Too slow, the wing stalls and the nose drops down to about -8 deg. The speed then
    // builds up again and the pilot can recover. Not in the last metres above the ground
    if (!onGround && ac.speed < STALL && ac.pos.y - ground > 3.0f && ac.pitch > glm::radians(-8.0f))
        R = glm::rotate(glm::mat4(1.0f), -glm::radians(10.0f) * h, glm::vec3(R[0])) * R;

    glm::extractEulerAngleYXZ(R, ac.heading, ac.pitch, ac.roll);
    ac.heading = wrapAngle(ac.heading);

    if (onGround) {           // on the wheels the wings stay level and the nose can only go up a little
        approachValue(ac.roll, 0.0f, 4.0f, h);
        ac.pitch = glm::clamp(ac.pitch, 0.0f, glm::radians(ac.speed > VR * 0.9f ? 15.0f : 2.0f));
    }

    // speed from the thrust, minus drag (more with flaps), gravity along the path and brakes
    glm::vec3 fwd = glm::vec3(glm::eulerAngleYXZ(ac.heading, ac.pitch, ac.roll) * glm::vec4(0, 0, -1, 0));
    float accel = thrustAccel(throttle, ac.speed) - dragAccel(ac.speed, ac.flaps, onGround) - G * fwd.y;
    if (onGround) accel -= 0.3f + (brakes ? 3.0f : 0.0f);
    ac.speed = glm::clamp(ac.speed + accel * h, 0.0f, 75.0f);

    // the aircraft moves where the nose points, and sinks if the wing is stalled
    glm::vec3 move = fwd * ac.speed * h;
    if (!onGround && ac.speed < STALL) move.y -= (STALL - ac.speed) * 0.8f * h;
    // extending the flaps gives some extra lift for a moment, retracting them makes it sink
    float flapsRate = glm::clamp(flapsTarget - ac.flaps, -FLAPS_RATE, FLAPS_RATE);
    if (!onGround) move.y += 0.06f * flapsRate * (ac.speed / V_APPROACH) * h;
    if (onGround && (ac.speed < VR || move.y < 0.0f)) move.y = 0.0f;            // still no lift
    ac.vs = move.y / h;
    ac.pos += move;
    // landing distances, from the last time the aircraft went below 50 ft and on the ground
    float aglNow = ac.pos.y - groundHeight(ac.pos.x, ac.pos.z);
    if (aglNow > 60.0f * FT) { passed50ft = false; distSince50ft = 0.0f; }
    if (!passed50ft && aglNow <= 50.0f * FT) { passed50ft = true; distSince50ft = 0.0f; }
    if (passed50ft) distSince50ft += ac.speed * h;
    if (onGround && touchdownSpeedKt > 0.0f) groundRollDist += ac.speed * h;

    // Collisions. Some points of the aircraft (nose, wing tips, tail, belly) are taken to
    // world coordinates and tested against the buildings and the ground
    glm::mat4 M = aircraftModelMatrix();
    const glm::vec3 points[] = { glm::vec3(0, 1.0f, -3.7f), glm::vec3(-5.2f, 1.0f, -0.9f), glm::vec3(5.2f, 1.0f, -0.9f),
                                 glm::vec3(0, 1.0f, 3.4f), glm::vec3(0, 2.7f, 3.3f), glm::vec3(0, 0.6f, -1.5f),
                                 glm::vec3(0, 1.5f, -1.0f) };
    const char* names[] = { "The nose hit the ground", "The left wing hit the ground", "The right wing hit the ground",
                            "Tail strike", "", "The belly hit the ground", "" };
    for (int i = 0; i < 7; i++) {
        glm::vec3 p = glm::vec3(M * glm::vec4(points[i], 1.0f));
        std::string b = hitBuilding(p);
        if (!b.empty()) { crash("Crashed into " + b); return; }
        if (names[i][0] && p.y < groundHeight(p.x, p.z) - 0.05f) {
            if (i == 3 && -ac.vs < 2.0f) {            // a gentle tail strike is only a warning
                ac.pitch -= glm::radians(2.0f);
                notify("Tail strike! Lower the nose", glm::vec4(1.0f, 0.75f, 0.25f, 1.0f));
                continue;
            }
            crash(groundHeight(p.x, p.z) > 3.0f ? std::string("Crashed into the mountain") : std::string(names[i]));
            return;
        }
    }

    ground = groundHeight(ac.pos.x, ac.pos.z);
    // On the ground, with 5 cm of tolerance, unless the aircraft is climbing away. That is
    // what lets it take off again
    if (ac.pos.y <= ground + (move.y > 0.0f ? 0.0f : 0.05f)) {
        if (!onGround) {                              // the wheels just touched the ground, so this was a landing
            float fpm = -ac.vs / FT * 60.0f;
            if (fpm > 1000.0f) { crash(fmt("Hit the ground too hard (%.0f ft/min)", fpm)); return; }
            if (ground > 3.0f && ac.speed > 8.0f) { crash("Landed on the mountain side"); return; }
            touchdownZ = ac.pos.z;
            touchdownSinkFpm = fpm;
            touchdownSpeedKt = ac.speed / KT;
            groundRollDist = 0.0f;
            touchdownOnRunway = fabsf(ac.pos.x) < RUNWAY_WIDTH * 0.5f && ac.pos.z < 278.0f && ac.pos.z > -1552.0f;
            tyreSmokeBurst = true;
            std::string msg = fpm < 200 ? "Touchdown - smooth" : (fpm < 400 ? "Touchdown - firm" : "Touchdown - hard!");
            if (!touchdownOnRunway) msg += ", off the runway";
            notify(msg + fmt("  (%.0f ft/min)", fpm), fpm < 400 && touchdownOnRunway ? glm::vec4(0.5f, 1.0f, 0.6f, 1.0f)
                                                                                   : glm::vec4(1.0f, 0.75f, 0.25f, 1.0f));
        }
        ac.pos.y = ground;
        ac.vs = 0.0f;
    }

    // stopped on the ground with the throttle closed, the pilot shuts the engine down
    bool shutdown = aircraftOnGround() && ac.speed < 0.5f && throttle < 0.01f;
    approachValue(rpm, shutdown ? 0.0f : RPM_IDLE + (RPM_FULL - RPM_IDLE) * throttle, shutdown ? 0.8f : 3.0f, h);
    approachValue(ac.aileron, 15.0f * inRoll, 8.0f, h);
    moveTowards(ac.flaps, flapsTarget, FLAPS_RATE, h);
}

// back from manual control, the automatic landing continues from where the aircraft is
void resumeAutopilot(void)
{
    if (crashed) return;
    if (aircraftOnGround()) {
        phase = (ac.speed < 0.5f) ? STOPPED : ROLLOUT;
        rolloutTime = DEROTATION_TIME;               // brake at once, as firmly as after a landing
        brakingDecel = glm::max(brakingDecel, 2.5f);
    } else {
        phase = APPROACH;
    }
}

// Once per frame. Propeller, particles and debris, then the manual or the automatic
// flight in steps of 5 ms. dt is multiplied by the simulation speed
void updateSimulation(float dt)
{
    bool moving = simRunning && !crashed;     // SPACE also pauses the manual flight
    float hs = dt * timeScale;
    // The propeller uses the simulated time, so it stops in pause and follows the simulation
    // speed. After landing the engine is shut down and it slows to a stop
    float hp = (moving || phase == STOPPED) ? hs : 0.0f;
    if (phase == STOPPED && !manualMode) approachValue(rpm, 0.0f, 0.8f, hp);
    // it turns much slower than a real one, because at the real speed it would seem to
    // stand still or turn backwards at 60 frames per second
    propAngle = fmodf(propAngle + rpm * 1.0f * hp, 360.0f);

    // effects use the simulated time too, and keep going after a crash
    if (moving || crashed) {
        updateParticles(hs);
        glm::mat4 M = aircraftModelMatrix();
        glm::vec3 vel = glm::vec3(M * glm::vec4(0, 0, -1, 0)) * ac.speed;
        // tyre smoke, a puff at touchdown and while braking on the runway
        if (tyreSmokeBurst) {
            for (int w = 0; w < 2; w++) spawnTyreSmoke(glm::vec3(M * glm::vec4(MAIN_WHEEL[w], 1.0f)), vel, 45, 1.0f);
            tyreSmokeBurst = false;
        }
        bool braking = (phase == ROLLOUT && rolloutTime > DEROTATION_TIME && !manualMode) ||
                       (manualMode && brakes && aircraftOnGround());
        if (moving && braking && ac.speed > 3.0f) {
            static float acc = 0.0f;
            acc += hs * 18.0f;
            while (acc >= 1.0f) {
                for (int w = 0; w < 2; w++) spawnTyreSmoke(glm::vec3(M * glm::vec4(MAIN_WHEEL[w], 1.0f)), vel, 1, 0.6f);
                acc -= 1.0f;
            }
        }
        // the wreck burns, fire for a while and then only smoke
        if (crashed) {
            float t = (float)(glfwGetTime() - crashTime);
            static float accF = 0.0f, accS = 0.0f;
            accF += hs * (t < 10.0f ? 30.0f : 0.0f);
            accS += hs * 8.0f;
            while (accF >= 1.0f) { spawnFire(wreckPosition, 1, 2.0f, 1.6f); accF -= 1.0f; }
            while (accS >= 1.0f) { spawnSmoke(wreckPosition + glm::vec3(0, 1.5f, 0), 1, 1.5f); accS -= 1.0f; }
            for (size_t i = 0; i < debris.size(); i++) {
                Debris& d = debris[i];
                if (d.resting) continue;
                d.vel.y -= G * hs;
                d.offset += d.vel * hs;
                d.angle += d.spin * hs;
                glm::vec3 c = d.center + d.offset;
                if (c.y < groundHeight(c.x, c.z) + 0.15f && d.vel.y < 0.0f) {
                    d.offset.y += groundHeight(c.x, c.z) + 0.15f - c.y;
                    d.vel *= 0.35f; d.vel.y = -d.vel.y * 0.3f; d.spin *= 0.4f;
                    if (glm::length(d.vel) < 0.6f) d.resting = true;
                }
            }
        }
    }
    if (crashed) return;

    if (manualMode) {
        if (!simRunning) return;
        const float STEP = 0.005f;
        float h = hs;
        while (h > 0.0f && !crashed) {
            manualFlightStep(glm::min(h, STEP));
            h -= STEP;
        }
        return;
    }
    if (!simRunning) return;
    if (phase == READY) {
        phase = APPROACH;
        notify("Final approach - following the 3 deg glide slope");
        printf("[APPROACH]  t=%5.1f s  established on the %.1f deg glide slope\n", simTime, GLIDE_SLOPE_DEG);
    }
    float h = dt * timeScale;
    simTime += h;
    const float STEP = 0.005f;
    while (h > 0.0f) {
        simulationStep(glm::min(h, STEP));
        h -= STEP;
    }
}

// angle above the horizon at which the aircraft is seen from a point on the ground
// (used by the PAPI and the glide slope needle)
float elevationAngleDeg(const glm::vec3& from)
{
    glm::vec3 eye = ac.pos + glm::vec3(0.0f, 2.0f, 0.0f);
    float horiz = glm::length(glm::vec2(eye.x - from.x, eye.z - from.z));
    return glm::degrees(atan2f(eye.y - from.y, horiz));
}

// a PAPI light is white if the aircraft is above its angle and red if below
bool papiIsWhite(int i)
{
    glm::vec3 light(PAPI_X[i], 1.0f, AIM_POINT_Z);
    if (ac.pos.z < light.z) return false;          // the aircraft already passed the PAPI
    return elevationAngleDeg(light) > PAPI_ANGLES_DEG[i];
}
