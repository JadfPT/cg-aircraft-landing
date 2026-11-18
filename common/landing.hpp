// landing.hpp
//
// Everything the source files of the simulator share. The constants of the landing
// (the TOLD data of the PA-28 and the layout of the airport), the types (meshes, the
// aircraft, particles), the global state and the functions used by more than one file.
//
// Where each part of the program is
//   main.cpp        OpenGL setup, meshes on the GPU, setMVP(), draw() and the render loop
//   simulation.cpp  automatic landing, manual flight, crash and particles
//   scene.cpp       airport, lights, the aircraft and its moving parts
//   terrain.cpp     mountains and sky dome
//   input.cpp       keyboard, mouse and cameras
//   hud_panels.cpp  the HUD panels (flight data, PAPI, help, landing summary)

#ifndef LANDING_HPP
#define LANDING_HPP

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <string>
#include <algorithm>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

// euler_angles is an experimental part of GLM, so it has to be enabled first
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/euler_angles.hpp>
using namespace glm;

extern GLFWwindow* window;      // created in main.cpp

// The runway goes along -Z. The threshold where the aircraft lands is at z = 0
const float RUNWAY_LENGTH = 1275.0f;   // metres, from z = 0 to z = -1275
const float RUNWAY_WIDTH  = 28.5f;     // metres of asphalt

// Lights of the aircraft, in aircraft coordinates. They sit on the lenses of the model
const glm::vec3 LIGHT_NAV_LEFT (-5.27f, 1.32f, -1.06f);   // red, left wing tip
const glm::vec3 LIGHT_NAV_RIGHT( 5.27f, 1.32f, -1.06f);   // green, right wing tip
const glm::vec3 LIGHT_TAIL     ( 0.00f, 2.63f,  3.58f);   // white, top of the rudder
const glm::vec3 LIGHT_BEACON   ( 0.01f, 2.73f,  3.20f);   // red flashing beacon on top of the fin
const glm::vec3 LIGHT_LANDING  ( 0.00f, 0.90f, -3.21f);   // landing light in the nose

// The pilot sits in the left seat. His hip is where the seated model is placed and the
// cockpit camera is at his eyes
const glm::vec3 PILOT_HIP = glm::vec3(-0.25f, 1.02f, -1.10f);
const glm::vec3 PILOT_EYE = PILOT_HIP + glm::vec3(0.0f, 0.725f, -0.06f);

// Takeoff and landing data (TOLD) of the Piper PA-28-161, taken from its POH.
// Sea level, standard day, maximum weight, flaps 40 and no wind
const float KT = 0.514444f;              // one knot in m/s
const float FT = 0.3048f;                // one foot in metres
const float G  = 9.81f;

const float GLIDE_SLOPE_DEG   = 3.0f;          // usual glide path of an ILS or a PAPI
const float V_START           = 70.0f * KT;    // initial approach speed of the POH, with flaps 25
const float V_APPROACH        = 63.0f * KT;    // final approach speed with flaps 40 (63 knots in the POH)
const float V_STALL_CLEAN     = 50.0f * KT;    // stall speed without flaps
const float V_STALL_FLAPS     = 44.0f * KT;    // stall speed with flaps 40
const float FLARE_HEIGHT      = 15.0f * FT;    // height where the flare starts
const float FLARE_MIN_SINK    = 100.0f * FT / 60.0f;   // sink rate at touchdown, about 100 ft/min
const float POH_GROUND_ROLL   = 595.0f * FT;   // ground roll of the landing in the POH
const float DEROTATION_TIME   = 2.0f;          // seconds to lower the nose after touchdown, before braking
const float POWER_OFF_HEIGHT  = 30.0f * FT;    // below this the power comes off, idle at the flare
const float APPROACH_AOA_DEG  = 4.0f;          // angle of attack on final approach
const float FLARE_PITCH_DEG   = 7.0f;          // nose up attitude at touchdown
const float RPM_IDLE          = 700.0f;        // throttle closed
const float RPM_FULL          = 2400.0f;       // throttle fully open (fixed pitch propeller)

// Simple performance model, with forces divided by the mass (m/s^2) so the weight does not
// appear. The drag has a parasite part that grows with V^2 (more with the flaps) and an
// induced part that falls with 1/V^2. Both are equal at the best glide speed (73 kt), where
// the lift to drag ratio is about 10. The fixed pitch propeller pulls less as the speed
// grows. With these numbers full power gives about 120 kt in level flight and close to the
// 710 ft/min of the POH at 79 kt, and the approach at 63 kt with flaps 40 needs about half
// power (around 1500 rpm)
const float V_BEST_GLIDE      = 73.0f * KT;
const float GLIDE_RATIO       = 10.0f;
const float FLAPS_DRAG        = 1.2f;          // extra parasite drag with flaps 40 (x 2.2)
const float THRUST_STATIC     = 2.4f;          // full throttle, standing still
const float THRUST_LOSS       = 0.0146f;       // lost for each m/s of speed
const float FLAPS_START_DEG   = 25.0f;         // flaps when the simulation starts
const float FLAPS_LANDING_DEG = 40.0f;         // full flaps on short final
const float FLAPS_RATE        = 8.0f;          // degrees per second

const float AIM_POINT_Z       = -104.0f;       // the aiming point markings painted on the runway model
const float START_DISTANCE    = 2000.0f;       // metres from the aiming point, on the glide slope
const float START_LATERAL     = 45.0f;         // metres to the side of the centre line, corrected on the way

// The four PAPI lights, from the one closest to the runway to the outer one. Each light
// is white when the aircraft is seen above its angle and red when below
const float PAPI_ANGLES_DEG[4] = { 3.5f, 3.1667f, 2.8333f, 2.5f };
const float PAPI_X[4]          = { -24.0f, -30.0f, -36.0f, -42.0f };

// Layout of the airport
const glm::vec3 TOWER_POSITION = glm::vec3(75.0f, 0.0f, -480.0f);
const glm::vec3 TOWER_EYE      = glm::vec3(-6.0f, 25.0f, 0.0f);   // the tower camera, on the cab balcony
const glm::vec3 HANGAR_POSITION[3] = { glm::vec3(112.0f, 0.0f, -300.0f), glm::vec3(112.0f, 0.0f, -334.0f),
                                       glm::vec3(112.0f, 0.0f, -368.0f) };
const float HANGAR_YAW = 90.0f;   // degrees, so the doors face the runway

// Where the main wheels touch the ground, in aircraft coordinates
const glm::vec3 MAIN_WHEEL[2] = { glm::vec3(-1.6f, 0.05f, -0.86f), glm::vec3(1.6f, 0.05f, -0.86f) };

// A mesh is a set of buffers in GPU memory with the vertices, colours, normals and
// texture coordinates, plus its texture and its opacity
struct Mesh {
    GLuint vertexbuffer = 0;
    GLuint colorbuffer  = 0;
    GLuint normalbuffer = 0;
    GLuint uvbuffer     = 0;
    GLuint texture      = 0;
    GLuint texture2     = 0;      // only the terrain uses it, for the rock (the grass is in texture)
    float  alpha        = 1.0f;
    GLsizei count = 0;
};

// A model is a list of meshes, one for each material of the .obj file
struct Model {
    std::vector<Mesh> meshes;
    glm::vec3 bbmin = glm::vec3(0.0f), bbmax = glm::vec3(0.0f);   // bounding box
    bool loaded = false;
    std::vector<glm::vec3> points;   // copy of the vertices, only kept to find the hinges
};

extern Model aircraftModel, propellerModel, flapModel[2], aileronModel[2], runwayModel;
extern Model pilotModel, handsModel, pilotStandingModel, crewModel[4], hangarModel, towerModel;
extern Model cubeModel, terrainModel, skyModel, particleModel;
extern glm::vec3 propellerHub, flapHinge[2][2], aileronHinge[2][2];
extern glm::mat4 Projection, View;
extern glm::vec3 cameraPosition;

// Each frame is drawn in two passes. Opaque meshes first, then the transparent ones
// (windows, particles, halos) with blending
enum RenderPass { PASS_OPAQUE, PASS_TRANSPARENT };
extern RenderPass renderPass;

// the only uniform that is changed outside draw()
extern GLuint TwoSidedID;

// Phases of the landing
enum Phase { READY, APPROACH, FLARE, ROLLOUT, STOPPED };
const char* const PHASE_NAME[] = { "READY", "FINAL APPROACH", "FLARE", "ROLLOUT", "STOPPED" };

struct Aircraft {
    glm::vec3 pos;       // point on the ground below the aircraft, the wheels are at pos.y
    float heading;       // radians around y, 0 means flying towards -Z
    float pitch;         // radians, positive with the nose up
    float roll;          // radians, positive with the left wing down
    float speed;         // m/s
    float vs;            // vertical speed in m/s
    float flaps;         // degrees, positive with the trailing edge down
    float aileron;       // degrees, positive rolls to the left (left aileron up)
};
extern Aircraft ac;

extern Phase phase;
extern bool  simRunning, manualMode, nightMode;
extern float timeScale, simTime, rolloutTime, propAngle, rpm, flapsTarget;

// controls of the manual flight
extern float inPitch, inRoll, throttle;
extern bool  brakes;

// messages on the screen and the help card
extern std::string toastText;
extern glm::vec4   toastColor;
extern double      toastTime;
extern bool        showHelp, quitRequested;

// what is recorded during the landing, for the summary at the end
extern bool  passed50ft, touchdownOnRunway, tyreSmokeBurst;
extern float brakingDecel;
extern float touchdownZ, touchdownSinkFpm, touchdownSpeedKt, groundRollDist, distSince50ft;

// crash
extern bool        crashed;
extern std::string crashReason;
extern double      crashTime;
extern glm::vec3   wreckPosition;

// Particles for the tyre smoke, the fire and the smoke of the wreck. Each one is a small
// textured square that always faces the camera
struct Particle {
    glm::vec3 pos, vel;
    float age, life;           // seconds
    float size0, size1;        // metres, it grows with age
    glm::vec3 color0, color1;
    float alpha;               // opacity when it is born
    float emissive;            // 1 for fire (not lit), 0 for smoke (lit by the lights)
    float buoyancy;            // m/s^2 upwards, smoke rises
};
extern std::vector<Particle> particles;

// In a crash the moving parts of the aircraft are thrown away as debris
struct Debris {
    int part;                  // 0 propeller, 1 and 2 flaps, 3 and 4 ailerons
    glm::mat4 M0;              // its world matrix at the moment of the crash
    glm::vec3 center, offset, vel, axis;
    float angle, spin;
    bool resting;
};
extern std::vector<Debris> debris;

// Cameras
enum CameraMode { CAM_CHASE = 1, CAM_TOWER, CAM_COCKPIT, CAM_FREE };
const char* const CAMERA_NAME[] = { "", "Chase", "Tower", "Cockpit", "Free" };
extern int   cameraMode;
extern float orbitYaw, orbitPitch, orbitDistance, freeYaw, freePitch, freeDistance;
extern float lookYaw, lookPitch, towerZoom;
extern glm::vec3 freeTarget;
extern double scrollOffset, lastMouseX, lastMouseY;

// Functions used by more than one file, grouped by the file they are in

// main.cpp
Mesh  uploadMesh(const std::vector<glm::vec3>& vertices, const std::vector<glm::vec3>& colors,
                 const std::vector<glm::vec3>& normals, const std::vector<glm::vec2>& uvs);
Model modelFromArrays(const std::vector<glm::vec3>& v, const std::vector<glm::vec3>& c, const std::vector<glm::vec3>& n);
void  drawModel(const Model& model, const glm::mat4& ModelMatrix, const glm::vec3& tint,
                float specular = 0.3f, float shininess = 16.0f, float emissive = 0.0f);

// terrain.cpp
float valueNoise(float x, float z);
float terrainHeight(float x, float z);
float groundHeight(float x, float z);        // the terrain, or y = 0 on the runway and the apron
bool  aircraftOnGround(void);
Model buildTerrain(void);
Model buildSkyDome(void);

// simulation.cpp
std::string fmt(const char* f, double v);    // like printf, for a single number
void  notify(const std::string& text, const glm::vec4& color = glm::vec4(1.0f));
void  resetSimulation(void);
void  resumeAutopilot(void);
void  approachValue(float& value, float target, float k, float h);   // smooth approach to a target
float thrustAccel(float throttle, float v);
float dragAccel(float v, float flaps, bool onGround);
void  updateSimulation(float dt);
float elevationAngleDeg(const glm::vec3& from);
bool  papiIsWhite(int i);

// input.cpp
void  scrollCallback(GLFWwindow* w, double xoffset, double yoffset);
void  handleInput(float dt);

// scene.cpp
glm::mat4 aircraftModelMatrix(void);
glm::mat4 partMatrix(int part);
const Model& partModel(int part);
bool  landingLightOn(void);
void  drawScene(void);
void  drawParticles(void);
void  drawGlows(void);
void  clearGlows(void);

// hud_panels.cpp
void  drawHUD(void);

#endif
