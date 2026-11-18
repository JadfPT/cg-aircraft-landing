// CG Assignment 2 - student 52216 -> Exercise 2, landing an aircraft in R^3
//
// A Piper PA-28-161 lands on the runway following the takeoff and landing data (TOLD)
// of its POH, for sea level, a standard day, maximum weight, flaps 40 and no wind.
// The runway threshold is at z = 0 and the aircraft lands towards -Z. With the key M
// the user can also fly the aircraft with the keyboard and the mouse.
//
//
// THE LANDING
//
//   final approach  starts 2000 m before the aiming point, at 344 ft and 70 kt (the
//                   initial approach speed of the POH) with flaps 25, and 45 m to the
//                   side of the centre line. It follows the 3 deg glide slope by asking
//                   for the vertical speed
//                       vs = -V sin(3 deg) + 0.5 (height on the glide slope - height)
//                   Below 300 ft the flaps go to 40 and the speed slows to 63 kt (POH).
//                   As a pilot does, the speed is held with the throttle. The speed comes
//                   from thrust, drag and weight (dV/dt = T - D - g vs / V), and the
//                   autopilot sets the throttle that gives the acceleration it wants.
//                   This needs about 1500 rpm on the glide slope. From 30 ft over the
//                   threshold the power comes off little by little
//   flare           from 15 ft the sink rate is proportional to the height
//                       vs = -k h
//                   so the aircraft sinks slower and slower until about 100 ft/min.
//                   The engine is at idle, the nose comes up to 7 deg and the drag
//                   slows the aircraft down
//   touchdown       a bit above the stall speed with flaps 40 (44 kt)
//   rollout         2 s to lower the nose, then the brakes. The deceleration comes from
//                   v^2 = 2 a d so that the aircraft stops in the ground roll of the POH
//                   (595 ft), a = V^2 / (2 (595 ft - distance rolled while the nose comes down))
//
// A typical landing uses about 1500 rpm on the glide slope, touches down at 49 kt and
// 100 ft/min some 177 m after the threshold, and stops after a ground roll of 181 m, the
// 595 ft of the POH.
//
// Aircraft data (TOLD) taken from
//   approach speeds, 70 kt initial and 63 kt final with flaps 40, from the POH
//       https://manualslib.com/manual/1226976/Piper-Warrior-Iii-Pa-28-161.html?page=75
//   stall speeds (50 kt without flaps, 44 kt with flaps 40), climb and engine power
//       https://github.com/FGMEMBERS/pa28-161
//   landing ground roll of 595 ft
//       https://www.planeandpilotmag.com/?p=14636
//   engine, 160 hp at 2700 rpm
//       https://aopa.org/go-fly/aircraft-and-ownership/aircraft-fact-sheets/piper-warrior
//
//
// MOVING THE AIRCRAFT IN R^3
//
// The model matrix of the aircraft is M = T(position) Ry(heading) Rx(pitch) Rz(roll).
// To fix the 45 m offset the autopilot chases a point ahead of it on the centre line and
// turns with coordinated turns, where the bank angle comes from tan(roll) = V w / g.
// In manual flight the keys rotate the aircraft about its own axes, it always moves
// where the nose points and banking makes it turn (w = g tan(roll) / V).
//
//
// EXTRAS
//
//   - textured cockpit of the PA-28 with the pilot's hands on the yoke and throttle
//     (fingers closed with the skeleton of the model) and the cockpit camera
//   - Russian pilot seated in the left seat, posed by hand since the model came standing
//   - ground crew on the apron and a pilot next to the tower
//   - moving parts, the propeller turns and the flaps and ailerons rotate about their hinges
//   - mountains all over the map (fractal noise) with the final approach along a valley,
//     sky dome and fog
//   - textured terrain, grass and rock detail textures made with fractal noise (seamless,
//     so they repeat without visible edges), with the rock projected along three axes so
//     it does not stretch on the slopes
//   - airport buildings, three hangars and the control tower
//   - runway, approach and taxiway lights and a working PAPI
//   - landing driven by the TOLD data, with a HUD (TrueType text, ILS needles, PAPI,
//     landing summary) and the help card
//   - tyre smoke, and collisions in manual flight with the terrain and buildings. A hard
//     touchdown (over 1000 ft/min), the nose or a wing on the ground means a crash, with an
//     explosion, debris thrown away and a burning wreck
//   - night mode with the apron floodlight, moonlight, the landing light of the aircraft
//     (spot light), its navigation, beacon and strobe lights and the airport lights. Every
//     light has a halo so it can be seen from far away
//   - transparent windows, drawn in a second pass with blending
//
//
// FILES
//
// The structure is the one of the lab codes (rotatingcube, house). This file has
// transferDataToGPUMemory(), setMVP(), draw() and the render loop, and the rest is
// split by topic
//   common/landing.hpp  constants (TOLD data, airport), types and shared state
//   simulation.cpp      automatic landing, manual flight, crash and particles
//   scene.cpp           airport, lights, the aircraft and its moving parts
//   terrain.cpp         mountains and sky dome
//   input.cpp           keyboard, mouse and cameras
//   hud_panels.cpp      HUD with the flight data, PAPI, help and landing summary
//   hud.cpp             small 2D renderer used by the HUD (shapes and text)
//   objloader.cpp       .obj and .mtl loader, texture.cpp textures, shader.cpp from the labs
//
//
// BUILDING AND RUNNING
//
// With glfw, glew and glm installed (Fedora glfw-devel glew-devel glm-devel, Ubuntu
// libglfw3-dev libglew-dev libglm-dev), run make and then ./bin/landing from the project
// folder, since the shaders and models are read from there.
//
// Keys: SPACE start or pause, R restart, C or 1 to 4 cameras, M fly it yourself,
// N day or night, H help, ESC quit. The help card (H) shows all of them.
//
//
// MODELS AND OTHER CREDITS (most models were converted to .obj, put in the frame of the
// program and reduced, and the seated pilot and the fingers of the hands were posed)
//
//   Piper PA-28 "Cadet"      helijah (Emmanuel Baranger), CC-BY-4.0     https://skfb.ly/LZwT
//   Runway                   pranav27, CC-BY-4.0                        https://skfb.ly/oKQXw
//   Ground crew              waelXcm, CC-BY-4.0                         https://skfb.ly/oI7zO
//   Russian pilot (seated)   Chernov-Egor, CC-BY-4.0                    https://skfb.ly/osGUs
//   First person hands       DavidFischer, CC-BY-4.0                    https://skfb.ly/BRVC
//   Pilot in winter uniform  kerrylogun, CC-BY-NC-4.0                   https://skfb.ly/pMRZp
//   Aircraft hangar          toromanovic1989, CC-BY-NC-4.0              https://skfb.ly/oxMBC
//   Control tower            abass20, CC-BY-4.0                         https://skfb.ly/oSVZM
//   stb_image, stb_truetype  Sean Barrett, public domain                https://github.com/nothings/stb
//   DejaVu Sans fonts        DejaVu fonts, free licence (fonts/LICENSE-DejaVu.txt)

#include <common/landing.hpp>
#include <common/shader.hpp>
#include <common/objloader.hpp>
#include <common/texture.hpp>
#include <common/hud.hpp>

GLFWwindow* window;

// The .obj files, read from the project folder (so the program has to run from there).
// They were all converted to the same frame. The aircraft has the nose towards -Z, y up
// and the wheels at y = 0, in metres. The runway goes along -Z with the threshold at
// z = 0 and its surface at y = 0. People have their feet at y = 0 and look towards -Z
const char* AIRCRAFT_OBJ       = "models/aircraft.obj";
const char* PROPELLER_OBJ      = "models/propeller.obj";
const char* FLAP_LEFT_OBJ      = "models/flap_left.obj";
const char* FLAP_RIGHT_OBJ     = "models/flap_right.obj";
const char* AILERON_LEFT_OBJ   = "models/aileron_left.obj";
const char* AILERON_RIGHT_OBJ  = "models/aileron_right.obj";
const char* RUNWAY_OBJ         = "models/runway.obj";
const char* PILOT_OBJ          = "models/pilot_seated.obj";
const char* HANDS_OBJ          = "models/hands.obj";
const char* PILOT_STANDING_OBJ = "models/pilot_standing.obj";
const char* HANGAR_OBJ         = "models/hangar.obj";
const char* TOWER_OBJ          = "models/tower.obj";
const char* CREW_OBJ[4] = { "models/crew_red.obj", "models/crew_green.obj",
                            "models/crew_yellow.obj", "models/crew_blue.obj" };

// OpenGL objects, as in the lab codes. One VAO, one shader program and the ids of
// its uniforms
GLuint VertexArrayID;
GLuint programID;
GLuint MatrixID, ModelMatrixID, NormalMatrixID;
GLuint LightPosID, LightColorID, LightAttID, CameraPosID;
GLuint TintID, AlphaID, AlphaCutoffID, UseTextureID, TextureID, RockTextureID;
GLuint TwoSidedID, AmbientID, SpecularID, ShininessID, EmissiveID, SkyColorID, FogID;
GLuint SpotPosID, SpotDirID, SpotColorID, SpotCutoffID, MoonDirID, MoonColorID;

// the models of the scene, each one with a mesh per material
Model aircraftModel, propellerModel, flapModel[2], aileronModel[2], runwayModel;
Model pilotModel, handsModel, pilotStandingModel, crewModel[4], hangarModel, towerModel;
Model cubeModel, terrainModel, skyModel, particleModel;

// centre of the propeller and hinge lines of the flaps and ailerons, in aircraft coordinates
glm::vec3 propellerHub = glm::vec3(0.0f);
glm::vec3 flapHinge[2][2], aileronHinge[2][2];

GLint WindowWidth = 1280;
GLint WindowHeight = 800;

glm::mat4 Projection = glm::mat4(1.0f);
glm::mat4 View = glm::mat4(1.0f);
glm::vec3 cameraPosition = glm::vec3(0.0f);

RenderPass renderPass = PASS_OPAQUE;

// Sends the vertex data to the GPU, one buffer each for the positions, colours,
// normals and texture coordinates
Mesh uploadMesh(const std::vector<glm::vec3>& vertices,
                const std::vector<glm::vec3>& colors,
                const std::vector<glm::vec3>& normals,
                const std::vector<glm::vec2>& uvs)
{
    Mesh m;
    m.count = (GLsizei)vertices.size();

    glGenBuffers(1, &m.vertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, m.vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), vertices.data(), GL_STATIC_DRAW);

    glGenBuffers(1, &m.colorbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, m.colorbuffer);
    glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(glm::vec3), colors.data(), GL_STATIC_DRAW);

    glGenBuffers(1, &m.normalbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, m.normalbuffer);
    glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(glm::vec3), normals.data(), GL_STATIC_DRAW);

    glGenBuffers(1, &m.uvbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, m.uvbuffer);
    glBufferData(GL_ARRAY_BUFFER, uvs.size() * sizeof(glm::vec2), uvs.data(), GL_STATIC_DRAW);
    return m;
}

// frees the GPU buffers of every mesh of a model
void deleteModel(Model& model)
{
    for (size_t i = 0; i < model.meshes.size(); i++) {
        Mesh& m = model.meshes[i];
        glDeleteBuffers(1, &m.vertexbuffer);
        glDeleteBuffers(1, &m.colorbuffer);
        glDeleteBuffers(1, &m.normalbuffer);
        glDeleteBuffers(1, &m.uvbuffer);
    }
    model.meshes.clear();
}

// Model with a single mesh without texture, for the geometry made in the code
// (cube, terrain, sky). It also keeps the bounding box
Model modelFromArrays(const std::vector<glm::vec3>& v, const std::vector<glm::vec3>& c, const std::vector<glm::vec3>& n)
{
    Model model;
    model.meshes.push_back(uploadMesh(v, c, n, std::vector<glm::vec2>(v.size(), glm::vec2(0.0f))));
    model.loaded = true;
    if (!v.empty()) {
        model.bbmin = model.bbmax = v[0];
        for (size_t i = 1; i < v.size(); i++) {
            model.bbmin = glm::min(model.bbmin, v[i]);
            model.bbmax = glm::max(model.bbmax, v[i]);
        }
    }
    return model;
}

// Loads an .obj file with one mesh per material, each with its texture (map_Kd) and
// opacity (d). keepPoints keeps a copy of the vertices, used to find the hinges
bool loadModel(const char* path, Model& model, bool required, bool keepPoints = false)
{
    std::vector<ObjPart> parts;
    FILE* f = fopen(path, "r");
    if (!f) {
        if (required) fprintf(stderr, "Missing file %s\n", path);
        return false;
    }
    fclose(f);
    if (!loadOBJ(path, parts)) return false;

    bool first = true;
    for (size_t i = 0; i < parts.size(); i++) {
        ObjPart& p = parts[i];
        // with a texture the colour comes from the image, since Kd is usually just a grey
        glm::vec3 color = p.texture.empty() ? p.diffuse : glm::vec3(1.0f);
        Mesh mesh = uploadMesh(p.vertices, std::vector<glm::vec3>(p.vertices.size(), color), p.normals, p.uvs);
        mesh.alpha = p.alpha;
        if (!p.texture.empty()) mesh.texture = loadTexture(p.texture.c_str());
        model.meshes.push_back(mesh);
        for (size_t k = 0; k < p.vertices.size(); k++) {
            if (first) { model.bbmin = model.bbmax = p.vertices[k]; first = false; }
            model.bbmin = glm::min(model.bbmin, p.vertices[k]);
            model.bbmax = glm::max(model.bbmax, p.vertices[k]);
            if (keepPoints) model.points.push_back(p.vertices[k]);
        }
    }
    model.loaded = true;
    return true;
}

// The hinge of a flap or aileron is its leading edge. Its two ends are the most
// forward vertices at the inner end and at the outer end of the surface
void computeHinge(const Model& m, glm::vec3 hinge[2])
{
    float xin = 1e9f, xout = -1e9f;
    for (size_t i = 0; i < m.points.size(); i++) {
        xin  = glm::min(xin,  fabsf(m.points[i].x));
        xout = glm::max(xout, fabsf(m.points[i].x));
    }
    hinge[0] = hinge[1] = glm::vec3(0.0f, 0.0f, 1e9f);
    for (size_t i = 0; i < m.points.size(); i++) {
        const glm::vec3& p = m.points[i];
        if (fabsf(p.x) < xin + 0.05f  && p.z < hinge[0].z) hinge[0] = p;
        if (fabsf(p.x) > xout - 0.05f && p.z < hinge[1].z) hinge[1] = p;
    }
}

// The cube of the lab codes has no normals, so each triangle gets the normal of its
// plane (cross product of two edges), turned to point away from the centre of the mesh
std::vector<glm::vec3> computeFlatNormals(const std::vector<glm::vec3>& v)
{
    glm::vec3 c(0.0f);
    for (size_t i = 0; i < v.size(); i++) c += v[i];
    c /= (float)v.size();
    std::vector<glm::vec3> n(v.size());
    for (size_t i = 0; i + 2 < v.size(); i += 3) {
        glm::vec3 fn = glm::normalize(glm::cross(v[i + 1] - v[i], v[i + 2] - v[i]));
        glm::vec3 fc = (v[i] + v[i + 1] + v[i + 2]) / 3.0f;
        if (glm::dot(fn, fc - c) < 0.0f) fn = -fn;
        n[i] = n[i + 1] = n[i + 2] = fn;
    }
    return n;
}

// model from an array of x, y, z floats, like the cube of the lab codes
Model modelFromArray(const GLfloat* data, int numFloats)
{
    std::vector<glm::vec3> v;
    for (int i = 0; i < numFloats; i += 3) v.push_back(glm::vec3(data[i], data[i + 1], data[i + 2]));
    std::vector<glm::vec3> c(v.size(), glm::vec3(1.0f));     // white, the real colour is given by Tint
    return modelFromArrays(v, c, computeFlatNormals(v));
}

// Everything that goes to the GPU only once, at the start. The shaders, the .obj
// models, the meshes made in the code (cube, terrain, sky and the particle square)
// and the ids of the uniforms
void transferDataToGPUMemory(void)
{
    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);

    // the Phong shaders (vertex and fragment)
    programID = LoadShaders("PhongVertexShader.vertexshader", "PhongFragmentShader.fragmentshader");
    if (programID == 0) {
        fprintf(stderr, "Could not load the shaders. Run the program from the project folder.\n");
        exit(EXIT_FAILURE);
    }

    // the aircraft and the runway are required, the other models are optional
    if (!loadModel(AIRCRAFT_OBJ, aircraftModel, true) || !loadModel(RUNWAY_OBJ, runwayModel, true)) {
        fprintf(stderr, "Could not load the .obj files. Run the program from the project folder.\n");
        exit(EXIT_FAILURE);
    }
    // moving parts of the aircraft, already in the same frame as the aircraft
    if (loadModel(PROPELLER_OBJ, propellerModel, false))
        propellerHub = 0.5f * (propellerModel.bbmin + propellerModel.bbmax);
    const char* flapFiles[2]    = { FLAP_LEFT_OBJ, FLAP_RIGHT_OBJ };
    const char* aileronFiles[2] = { AILERON_LEFT_OBJ, AILERON_RIGHT_OBJ };
    for (int s = 0; s < 2; s++) {
        if (loadModel(flapFiles[s], flapModel[s], false, true))       computeHinge(flapModel[s], flapHinge[s]);
        if (loadModel(aileronFiles[s], aileronModel[s], false, true)) computeHinge(aileronModel[s], aileronHinge[s]);
    }
    // people and buildings
    loadModel(PILOT_OBJ, pilotModel, false);
    loadModel(HANDS_OBJ, handsModel, false);
    loadModel(PILOT_STANDING_OBJ, pilotStandingModel, false);
    loadModel(HANGAR_OBJ, hangarModel, false);
    loadModel(TOWER_OBJ, towerModel, false);
    for (int i = 0; i < 4; i++) loadModel(CREW_OBJ[i], crewModel[i], false);

    // Cube of the lab codes (rotatingcube). Scaled, it makes the apron, the light
    // fixtures, the lamp posts and the PAPI boxes
    static const GLfloat cube[] = {
        -1.0f,-1.0f,-1.0f,  -1.0f,-1.0f, 1.0f,  -1.0f, 1.0f, 1.0f,
         1.0f, 1.0f,-1.0f,  -1.0f,-1.0f,-1.0f,  -1.0f, 1.0f,-1.0f,
         1.0f,-1.0f, 1.0f,  -1.0f,-1.0f,-1.0f,   1.0f,-1.0f,-1.0f,
         1.0f, 1.0f,-1.0f,   1.0f,-1.0f,-1.0f,  -1.0f,-1.0f,-1.0f,
        -1.0f,-1.0f,-1.0f,  -1.0f, 1.0f, 1.0f,  -1.0f, 1.0f,-1.0f,
         1.0f,-1.0f, 1.0f,  -1.0f,-1.0f, 1.0f,  -1.0f,-1.0f,-1.0f,
        -1.0f, 1.0f, 1.0f,  -1.0f,-1.0f, 1.0f,   1.0f,-1.0f, 1.0f,
         1.0f, 1.0f, 1.0f,   1.0f,-1.0f,-1.0f,   1.0f, 1.0f,-1.0f,
         1.0f,-1.0f,-1.0f,   1.0f, 1.0f, 1.0f,   1.0f,-1.0f, 1.0f,
         1.0f, 1.0f, 1.0f,   1.0f, 1.0f,-1.0f,  -1.0f, 1.0f,-1.0f,
         1.0f, 1.0f, 1.0f,  -1.0f, 1.0f,-1.0f,  -1.0f, 1.0f, 1.0f,
         1.0f, 1.0f, 1.0f,  -1.0f, 1.0f, 1.0f,   1.0f,-1.0f, 1.0f
    };
    cubeModel = modelFromArray(cube, sizeof(cube) / sizeof(GLfloat));

    // The particles and halos use a square of side 1 with a soft round texture,
    // made here pixel by pixel. White, with the opacity falling from the centre
    {
        const int N = 64;
        std::vector<unsigned char> img(N * N * 4);
        for (int y = 0; y < N; y++)
            for (int x = 0; x < N; x++) {
                float dx = (x + 0.5f) / N * 2 - 1, dy = (y + 0.5f) / N * 2 - 1;
                float r = sqrtf(dx * dx + dy * dy);
                float n = 0.75f + 0.25f * valueNoise(x * 0.25f, y * 0.25f);
                float a = glm::clamp(1.0f - r, 0.0f, 1.0f);
                unsigned char* p = &img[(y * N + x) * 4];
                p[0] = p[1] = p[2] = 255;
                p[3] = (unsigned char)(255.0f * glm::clamp(a * a * n * 1.4f, 0.0f, 1.0f));
            }
        GLuint tex;
        glGenTextures(1, &tex);
        glBindTexture(GL_TEXTURE_2D, tex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, N, N, 0, GL_RGBA, GL_UNSIGNED_BYTE, img.data());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glGenerateMipmap(GL_TEXTURE_2D);
        std::vector<glm::vec3> v, c(6, glm::vec3(1.0f)), n(6, glm::vec3(0, 0, 1));
        std::vector<glm::vec2> uv;
        const float q[6][2] = { { -0.5f, -0.5f }, { 0.5f, -0.5f }, { 0.5f, 0.5f }, { -0.5f, -0.5f }, { 0.5f, 0.5f }, { -0.5f, 0.5f } };
        for (int i = 0; i < 6; i++) { v.push_back(glm::vec3(q[i][0], q[i][1], 0)); uv.push_back(glm::vec2(q[i][0] + 0.5f, q[i][1] + 0.5f)); }
        particleModel.meshes.push_back(uploadMesh(v, c, n, uv));
        particleModel.meshes[0].texture = tex;
        particleModel.meshes[0].alpha = 0.5f;
        particleModel.loaded = true;
    }

    // terrain with its two detail textures (grass and rock) and sky
    terrainModel = buildTerrain();
    if (!terrainModel.meshes.empty()) {
        terrainModel.meshes[0].texture  = loadTexture("models/textures/terrain_grass.jpg");
        terrainModel.meshes[0].texture2 = loadTexture("models/textures/terrain_rock.jpg");
        if (!terrainModel.meshes[0].texture) terrainModel.meshes[0].texture2 = 0;
    }
    skyModel     = buildSkyDome();

    // ids of the uniforms of the shaders
    MatrixID       = glGetUniformLocation(programID, "MVP");
    ModelMatrixID  = glGetUniformLocation(programID, "M");
    NormalMatrixID = glGetUniformLocation(programID, "NormalMatrix");
    LightPosID     = glGetUniformLocation(programID, "LightPosition_worldspace");
    LightColorID   = glGetUniformLocation(programID, "LightColor");
    LightAttID     = glGetUniformLocation(programID, "LightAttenuation");
    CameraPosID    = glGetUniformLocation(programID, "CameraPosition_worldspace");
    TintID         = glGetUniformLocation(programID, "Tint");
    AlphaID        = glGetUniformLocation(programID, "Alpha");
    AlphaCutoffID  = glGetUniformLocation(programID, "AlphaCutoff");
    UseTextureID   = glGetUniformLocation(programID, "UseTexture");
    TextureID      = glGetUniformLocation(programID, "TextureSampler");
    RockTextureID  = glGetUniformLocation(programID, "RockSampler");
    TwoSidedID     = glGetUniformLocation(programID, "TwoSided");
    SpotPosID      = glGetUniformLocation(programID, "SpotPosition_worldspace");
    SpotDirID      = glGetUniformLocation(programID, "SpotDirection");
    SpotColorID    = glGetUniformLocation(programID, "SpotColor");
    SpotCutoffID   = glGetUniformLocation(programID, "SpotCosCutoff");
    MoonDirID      = glGetUniformLocation(programID, "MoonDirection");
    MoonColorID    = glGetUniformLocation(programID, "MoonColor");
    AmbientID      = glGetUniformLocation(programID, "AmbientStrength");
    SpecularID     = glGetUniformLocation(programID, "SpecularStrength");
    ShininessID    = glGetUniformLocation(programID, "Shininess");
    EmissiveID     = glGetUniformLocation(programID, "Emissive");
    SkyColorID     = glGetUniformLocation(programID, "SkyColor");
    FogID          = glGetUniformLocation(programID, "FogDensity");
}

// frees what transferDataToGPUMemory() created
void cleanupDataFromGPU()
{
    Model* models[] = { &aircraftModel, &propellerModel, &flapModel[0], &flapModel[1], &aileronModel[0],
                        &aileronModel[1], &runwayModel, &pilotModel, &handsModel, &pilotStandingModel,
                        &crewModel[0], &crewModel[1], &crewModel[2], &crewModel[3], &hangarModel, &towerModel,
                        &cubeModel, &terrainModel, &skyModel, &particleModel };
    for (size_t i = 0; i < sizeof(models) / sizeof(models[0]); i++) deleteModel(*models[i]);
    deleteTextures();
    glDeleteVertexArrays(1, &VertexArrayID);
    glDeleteProgram(programID);
}

// View and Projection matrices of the current camera. The Model matrix changes for
// each object, so MVP = Projection * View * Model is done in drawModel()
void setMVP(void)
{
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    float aspect = (height > 0) ? (float)width / (float)height : 4.0f / 3.0f;

    glm::vec3 center, up(0, 1, 0);
    float fov = 50.0f, zNear = 0.5f;
    glm::vec3 acCenter = ac.pos + glm::vec3(0.0f, 1.3f, 0.0f);

    switch (cameraMode) {
    // chase camera, on a sphere around the aircraft that turns with it
    case CAM_CHASE: {
        float yaw = ac.heading + orbitYaw;
        glm::vec3 offset(sinf(yaw) * cosf(orbitPitch), sinf(orbitPitch), cosf(yaw) * cosf(orbitPitch));
        cameraPosition = acCenter + orbitDistance * offset;
        // never under the ground
        cameraPosition.y = glm::max(cameraPosition.y, terrainHeight(cameraPosition.x, cameraPosition.z) + 1.0f);
        center = acCenter;
        break;
    }
    case CAM_TOWER:
        // from the tower, always looking at the aircraft. The field of view closes as
        // the aircraft gets far, so about 70 m around it always fill the picture
        cameraPosition = TOWER_POSITION + TOWER_EYE;
        center = acCenter;
        fov = glm::clamp(glm::degrees(2.0f * atan2f(35.0f, glm::length(center - cameraPosition))) * towerZoom, 1.5f, 70.0f);
        break;
    // the eyes of the pilot. Position, viewing direction and up vector are all taken
    // from aircraft coordinates with the model matrix, so the camera rolls with the aircraft
    case CAM_COCKPIT: {
        glm::mat4 M = aircraftModelMatrix();
        glm::vec3 look(-sinf(lookYaw) * cosf(lookPitch), sinf(lookPitch), -cosf(lookYaw) * cosf(lookPitch));
        cameraPosition = glm::vec3(M * glm::vec4(PILOT_EYE, 1.0f));
        center = cameraPosition + glm::vec3(M * glm::vec4(look, 0.0f));
        up = glm::vec3(M * glm::vec4(0, 1, 0, 0));
        fov = 70.0f;
        zNear = 0.05f;
        break;
    }
    // free camera, on a sphere around a point that the user moves
    case CAM_FREE:
    default: {
        glm::vec3 offset(sinf(freeYaw) * cosf(freePitch), sinf(freePitch), cosf(freeYaw) * cosf(freePitch));
        cameraPosition = freeTarget + freeDistance * offset;
        center = freeTarget;
        break;
    }
    }

    // perspective projection, seeing from zNear up to 20 km
    Projection = glm::perspective(glm::radians(fov), aspect, zNear, 20000.0f);

    // camera matrix
    View = glm::lookAt(cameraPosition, center, up);
}

// Draws a model with the model matrix M. It sends MVP, M, the normal matrix and the
// material to the shaders and then draws each mesh with its texture. Only the meshes of
// the current pass are drawn, the opaque ones in the first pass and the transparent ones
// (windows) in the second
void drawModel(const Model& model, const glm::mat4& ModelMatrix, const glm::vec3& tint,
               float specular, float shininess, float emissive)
{
    if (!model.loaded) return;
    glm::mat4 MVP = Projection * View * ModelMatrix;
    // The normals need the inverse transpose of M, otherwise they lose the right
    // direction when the model is scaled differently along each axis
    glm::mat3 NormalMatrix = glm::inverseTranspose(glm::mat3(ModelMatrix));

    glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);
    glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &ModelMatrix[0][0]);
    glUniformMatrix3fv(NormalMatrixID, 1, GL_FALSE, &NormalMatrix[0][0]);
    glUniform3fv(TintID, 1, &tint[0]);
    glUniform1f(SpecularID, specular);
    glUniform1f(ShininessID, shininess);
    glUniform1f(EmissiveID, emissive);

    for (size_t i = 0; i < model.meshes.size(); i++) {
        const Mesh& mesh = model.meshes[i];
        bool transparent = mesh.alpha < 0.999f;
        if (transparent != (renderPass == PASS_TRANSPARENT)) continue;

        glUniform1f(AlphaID, mesh.alpha);
        glUniform1f(AlphaCutoffID, transparent ? 0.01f : 0.5f);

        // the texture goes in unit 0 (the terrain also uses unit 1, for the rock)
        glUniform1i(UseTextureID, mesh.texture2 ? 2 : (mesh.texture ? 1 : 0));
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, mesh.texture);
        glUniform1i(TextureID, 0);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, mesh.texture2);
        glUniform1i(RockTextureID, 1);
        glActiveTexture(GL_TEXTURE0);

        // the four attributes of the vertex shader (position, colour, normal and texture
        // coordinates), each one read from its own buffer as in the lab codes
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, mesh.vertexbuffer);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, mesh.colorbuffer);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

        glEnableVertexAttribArray(2);
        glBindBuffer(GL_ARRAY_BUFFER, mesh.normalbuffer);
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

        glEnableVertexAttribArray(3);
        glBindBuffer(GL_ARRAY_BUFFER, mesh.uvbuffer);
        glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, 0, (void*)0);

        glDrawArrays(GL_TRIANGLES, 0, mesh.count);

        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
        glDisableVertexAttribArray(2);
        glDisableVertexAttribArray(3);
    }
}

// Draws one frame. First the lights. By day the point light is far away and strong,
// like the sun. By night it is the floodlight of the apron, with attenuation, plus a
// weak moonlight and the landing light of the aircraft. Then the sky, the terrain, the
// scene and at the end the transparent things
void draw(void)
{
    glm::vec3 skyTint, horizon, lightPos, lightColor, lightAtt;
    float ambient, fog;
    if (!nightMode) {
        skyTint    = glm::vec3(1.0f);
        horizon    = glm::vec3(0.74f, 0.82f, 0.90f);
        lightPos   = glm::vec3(3000.0f, 4000.0f, 2000.0f);
        lightColor = glm::vec3(1.0f, 0.96f, 0.88f);
        lightAtt   = glm::vec3(1.0f, 0.0f, 0.0f);
        ambient    = 0.38f;
        fog        = 0.00016f;
    } else {
        skyTint    = glm::vec3(0.07f, 0.08f, 0.14f);
        horizon    = glm::vec3(0.78f, 0.86f, 0.95f) * skyTint;
        lightPos   = glm::vec3(60.0f, 20.0f, -380.0f);
        lightColor = glm::vec3(1.0f, 0.88f, 0.70f) * 1.6f;
        lightAtt   = glm::vec3(1.0f, 0.004f, 0.00004f);
        ambient    = 0.12f;
        fog        = 0.00018f;
    }
    clearGlows();

    // The viewport is the whole window. It is set every frame because on WSL the resize
    // callback does not always arrive
    int fbw, fbh;
    glfwGetFramebufferSize(window, &fbw, &fbh);
    glViewport(0, 0, fbw, fbh);

    // clear the screen with the colour of the horizon
    glClearColor(horizon.r, horizon.g, horizon.b, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // our shader, our VAO and the light uniforms
    glUseProgram(programID);
    glBindVertexArray(VertexArrayID);
    glUniform3fv(LightPosID, 1, &lightPos[0]);
    glUniform3fv(LightColorID, 1, &lightColor[0]);
    glUniform3fv(LightAttID, 1, &lightAtt[0]);
    glUniform3fv(CameraPosID, 1, &cameraPosition[0]);
    glUniform3fv(SkyColorID, 1, &horizon[0]);
    glUniform1f(AmbientID, ambient);

    // At night a weak bluish moonlight, a light that only has a direction, so the shapes
    // of the mountains are still seen
    glm::vec3 moonDir = glm::normalize(glm::vec3(-0.4f, 0.8f, 0.45f));
    glm::vec3 moonColor = nightMode ? glm::vec3(0.20f, 0.23f, 0.33f) : glm::vec3(0.0f);
    glUniform3fv(MoonDirID, 1, &moonDir[0]);
    glUniform3fv(MoonColorID, 1, &moonColor[0]);

    // The landing light is a spot light in the nose, pointing forward and a bit down.
    // Its position and direction go from aircraft to world coordinates with the model matrix
    glm::mat4 AM = aircraftModelMatrix();
    glm::vec3 spotPos = glm::vec3(AM * glm::vec4(LIGHT_LANDING, 1.0f));
    glm::vec3 spotDir = glm::normalize(glm::vec3(AM * glm::vec4(0.0f, -0.12f, -1.0f, 0.0f)));
    glm::vec3 spotColor = (nightMode && landingLightOn()) ? glm::vec3(5.0f, 4.8f, 4.4f) : glm::vec3(0.0f);
    glUniform3fv(SpotPosID, 1, &spotPos[0]);
    glUniform3fv(SpotDirID, 1, &spotDir[0]);
    glUniform3fv(SpotColorID, 1, &spotColor[0]);
    glUniform1f(SpotCutoffID, cosf(glm::radians(14.0f)));

    // first pass, the opaque objects
    renderPass = PASS_OPAQUE;
    glDisable(GL_BLEND);

    // The sky dome moves with the camera, so it is never reached. It has no fog and does
    // not write in the depth buffer, so everything else is drawn in front of it
    glDepthMask(GL_FALSE);
    glUniform1f(FogID, 0.0f);
    glm::mat4 sky = glm::scale(glm::translate(glm::mat4(1.0f), cameraPosition), glm::vec3(15000.0f));
    drawModel(skyModel, sky, skyTint, 0.0f, 1.0f, 1.0f);
    glDepthMask(GL_TRUE);

    glUniform1f(FogID, fog);
    glUniform1i(TwoSidedID, 0);     // the terrain is only seen from above
    drawModel(terrainModel, glm::mat4(1.0f), glm::vec3(1.0f), 0.04f, 4.0f);
    glUniform1i(TwoSidedID, 1);     // the faces of the .obj models may point either way
    drawScene();

    // Second pass, the transparent objects blended over the opaque ones. They do not
    // write in the depth buffer, so they do not hide each other
    renderPass = PASS_TRANSPARENT;
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    drawScene();
    drawParticles();
    drawGlows();
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    renderPass = PASS_OPAQUE;
}

// Opens the window, creates the OpenGL context and runs the render loop
int main( void )
{
    bool wsl = false;
#ifdef __linux__
    // On WSL GLFW picks Wayland by default and then glewInit() fails, so we make it use X11
    unsetenv("WAYLAND_DISPLAY");

    // On WSL the graphics card is reached through the D3D12 driver of Mesa. Without this
    // Mesa may pick another driver that crashes. Only done if the user did not choose one
    wsl = getenv("WSL_DISTRO_NAME") != NULL || getenv("WSL_INTEROP") != NULL;
    bool userChoice = getenv("GALLIUM_DRIVER") != NULL || getenv("LIBGL_ALWAYS_SOFTWARE") != NULL;
    if (wsl && !userChoice) {
        setenv("GALLIUM_DRIVER", "d3d12", 0);
        setenv("MESA_D3D12_DEFAULT_ADAPTER_NAME", "NVIDIA", 0);   // prefer the dedicated GPU
    }
#endif

    for (int attempt = 0; attempt < 2 && window == NULL; attempt++) {
        if (!glfwInit()) {
            fprintf(stderr, "Failed to initialize GLFW\n");
            return -1;
        }

        glfwWindowHint(GLFW_SAMPLES, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

        // the window takes 85% of the screen and is centred
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();
        const GLFWvidmode* mode = monitor ? glfwGetVideoMode(monitor) : NULL;
        if (mode) {
            WindowWidth  = (int)(mode->width * 0.85f);
            WindowHeight = (int)(mode->height * 0.85f);
        }
        window = glfwCreateWindow( WindowWidth, WindowHeight, "Aircraft Landing Simulator - Piper PA-28", NULL, NULL);
        if (window && mode)
            glfwSetWindowPos(window, (mode->width - WindowWidth) / 2, (mode->height - WindowHeight) / 2);
        if (window == NULL) {
            glfwTerminate();
#ifdef __linux__
            if (wsl && attempt == 0 && getenv("LIBGL_ALWAYS_SOFTWARE") == NULL) {
                // if the graphics card driver fails, try again with software rendering
                fprintf(stderr, "GPU (D3D12) OpenGL not available -> using software rendering\n");
                unsetenv("GALLIUM_DRIVER");
                setenv("LIBGL_ALWAYS_SOFTWARE", "1", 1);
                continue;
            }
#endif
            fprintf(stderr, "Failed to open GLFW window (OpenGL 3.3 needed)\n");
            return -1;
        }
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // On WSL glewInit() may report an error even though the functions were loaded, so
    // like in the lab codes we carry on, and only stop if they are really missing
    glewExperimental = true; // Needed for core profile
    GLenum glewStatus = glewInit();
    if (glewStatus != GLEW_OK)
        fprintf(stderr, "Warning: glewInit: %s (continuing)\n", glewGetErrorString(glewStatus));
    if (glGenVertexArrays == NULL) {
        fprintf(stderr, "OpenGL 3.3 functions not available. On WSL try: export LIBGL_ALWAYS_SOFTWARE=1\n");
        return -1;
    }

    // keys and mouse wheel
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
    glfwSetScrollCallback(window, scrollCallback);
    glfwGetCursorPos(window, &lastMouseX, &lastMouseY);

	// information about the OpenGL driver, as in the lab codes
	const GLubyte *renderer = glGetString( GL_RENDERER );
	const GLubyte *vendor = glGetString( GL_VENDOR );
	const GLubyte *version = glGetString( GL_VERSION );
	const GLubyte *glslVersion = glGetString( GL_SHADING_LANGUAGE_VERSION );

	GLint major, minor;
	glGetIntegerv(GL_MAJOR_VERSION, &major);
	glGetIntegerv(GL_MINOR_VERSION, &minor);

	printf("-------------------------------------------------------------\n");
	printf("GL Vendor    : %s\n", vendor);
	printf("GL Renderer  : %s\n", renderer);
	printf("GL Version   : %s\n", version);
	printf("GL Version   : %d.%d\n", major, minor);
	printf("GLSL Version : %s\n", glslVersion);
	printf("-------------------------------------------------------------\n");

    // depth test, a fragment is kept if it is closer to the camera than the one there
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glEnable(GL_MULTISAMPLE);

    // send the data (vertices, colours, normals, textures and shaders) to the GPU
    transferDataToGPUMemory();

    resetSimulation();
    if (!hudInit("fonts/DejaVuSans.ttf", "fonts/DejaVuSans-Bold.ttf"))
        fprintf(stderr, "Warning: the HUD could not be created (missing fonts/ or HUD shaders)\n");
    // the main TOLD numbers, written in the terminal
    printf("TOLD (PA-28-161): final approach %.0f kt, stall %.0f kt (flaps 40), glide slope %.1f deg, "
           "POH ground roll %.0f ft\n",
           V_APPROACH / KT, V_STALL_FLAPS / KT, GLIDE_SLOPE_DEG, POH_GROUND_ROLL / FT);

    double lastTime = glfwGetTime();

    // Render loop. dt is the real time of the last frame, limited to 50 ms so a slow
    // frame does not make the aircraft jump
    do{
        double now = glfwGetTime();
        float dt = (float)glm::min(now - lastTime, 0.05);
        lastTime = now;

        handleInput(dt);        // keyboard and mouse
        updateSimulation(dt);   // the landing, or the manual flight
        setMVP();               // camera
        draw();                 // 3D scene
        drawHUD();              // 2D information on top

        glfwSwapBuffers(window);
        glfwPollEvents();

    } // until ESC or the window is closed
    while( !quitRequested && glfwWindowShouldClose(window) == 0 );

    // free the VAO, buffers, textures and shaders on the GPU
    hudCleanup();
    cleanupDataFromGPU();

    glfwTerminate();

    return 0;
}
