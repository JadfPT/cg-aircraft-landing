// terrain.cpp
//
// The ground around the airport and the sky. The height of the terrain is a fractal sum
// of value noise (fBm). The big octaves give the mountain ranges and the small ones the
// ridges. Around the airport it is flat, and under the approach path it is kept low, so
// the final approach is flown along a valley. The terrain and the sky dome are built
// once, as triangle meshes, at the start of the program.

#include <common/landing.hpp>

// pseudo random value between 0 and 1 for each integer point (x, z), always the same for the same point
float hash2(int x, int z)
{
    unsigned int h = (unsigned int)x * 374761393u + (unsigned int)z * 668265263u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return (float)((h ^ (h >> 16)) & 0xffffu) / 65535.0f;
}

// value noise, the random values of the integer points smoothly interpolated in between
float valueNoise(float x, float z)
{
    int ix = (int)floorf(x), iz = (int)floorf(z);
    float fx = x - ix, fz = z - iz;
    fx = fx * fx * (3.0f - 2.0f * fx);
    fz = fz * fz * (3.0f - 2.0f * fz);
    float a = hash2(ix, iz), b = hash2(ix + 1, iz), c = hash2(ix, iz + 1), d = hash2(ix + 1, iz + 1);
    return glm::mix(glm::mix(a, b, fx), glm::mix(c, d, fx), fz);
}

// Fractal sum of value noise (fBm). Each octave has double the frequency and about half
// the amplitude of the one before, which gives a natural looking relief
float fbm(float x, float z, int octaves)
{
    float sum = 0.0f, amp = 0.5f, freq = 1.0f, norm = 0.0f;
    for (int o = 0; o < octaves; o++) {
        sum  += amp * valueNoise(x * freq + 31.7f * o, z * freq - 17.1f * o);
        norm += amp;
        freq *= 2.02f;
        amp  *= 0.48f;
    }
    return sum / norm;
}

// Height of the terrain. Rolling mountains everywhere, except a flat area around the
// airport. Under the approach path the terrain is kept well below the glide slope, so
// the final approach is flown along a wide valley
const glm::vec2 VALLEY_CENTER(0.0f, -500.0f);

float terrainHeight(float x, float z)
{
    // distance to the airport area, the runway and the apron
    float ax = glm::max(glm::max(-60.0f - x, x - 140.0f), 0.0f);
    float az = glm::max(glm::max(-1650.0f - z, z - 350.0f), 0.0f);
    float dAirport = sqrtf(ax * ax + az * az);

    float big    = fbm(x / 4200.0f + 3.1f, z / 4200.0f + 7.7f, 6);      // mountain ranges
    float detail = fbm(x / 900.0f, z / 900.0f, 4);                       // ridges and gullies
    float range  = glm::smoothstep(0.35f, 0.75f, big);
    float h = glm::smoothstep(200.0f, 2200.0f, dAirport) * (80.0f + 750.0f * range * range + 140.0f * detail * (0.3f + range))
            + 15.0f * fbm(x / 400.0f, z / 400.0f, 3) * glm::smoothstep(80.0f, 400.0f, dAirport);

    // under the final approach (x near 0, z > 0) the valley floor is flat and only rises
    // gently far away, always well below the 3 deg glide slope
    if (z > 0.0f) {
        float glide = glm::max((z - 2600.0f) * 0.035f, 0.0f);
        float corridor = 1.0f - glm::smoothstep(300.0f, 1400.0f, fabsf(x - START_LATERAL * 0.5f));
        h = glm::mix(h, glm::min(h, glm::max(glide, 0.0f)), corridor);
    }
    return -0.15f + h;
}

// height of the ground where the wheels roll, the runway and the apron are at y = 0
float groundHeight(float x, float z)
{
    return glm::max(terrainHeight(x, z), 0.0f);
}

// the wheels are on the ground, with 5 cm of tolerance
bool aircraftOnGround(void)
{
    return ac.pos.y <= groundHeight(ac.pos.x, ac.pos.z) + 0.05f;
}

// Terrain mesh with 320 x 320 cells over 24 km, two triangles per cell. Height and
// normal come from terrainHeight(), the colour from the height and the slope
Model buildTerrain(void)
{
    const int   N = 320;
    const float SIZE = 24000.0f;
    const glm::vec2 C = VALLEY_CENTER;
    const float cell = SIZE / N;

    std::vector<glm::vec3> v, c, n;
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) {
            float x0 = C.x - SIZE / 2 + i * cell, z0 = C.y - SIZE / 2 + j * cell;
            glm::vec2 q[6] = { glm::vec2(x0, z0), glm::vec2(x0, z0 + cell), glm::vec2(x0 + cell, z0 + cell),
                               glm::vec2(x0, z0), glm::vec2(x0 + cell, z0 + cell), glm::vec2(x0 + cell, z0) };
            for (int k = 0; k < 6; k++) {
                float x = q[k].x, z = q[k].y, y = terrainHeight(x, z);
                const float e = 5.0f;   // the normal from central differences of the height
                glm::vec3 nn = glm::normalize(glm::vec3(terrainHeight(x - e, z) - terrainHeight(x + e, z), 2 * e,
                                                        terrainHeight(x, z - e) - terrainHeight(x, z + e)));
                // colour, grass in the valley, then forest, then rock high up or where it is steep
                float patch = fbm(x / 260.0f, z / 260.0f, 3);
                float var = 0.10f * (patch - 0.5f) + 0.05f * (valueNoise(x / 37.0f, z / 37.0f) - 0.5f);
                glm::vec3 grass(0.36f + var, 0.52f + var, 0.22f);
                glm::vec3 forest(0.13f + 0.6f * var, 0.25f + var, 0.11f);
                glm::vec3 rock(0.47f + var, 0.44f + var, 0.39f + var);
                float forestAmount = glm::smoothstep(8.0f, 60.0f, y) * glm::smoothstep(0.35f, 0.55f, patch + 0.25f);
                glm::vec3 col = glm::mix(grass, forest, forestAmount);
                float rockAmount = glm::smoothstep(520.0f, 760.0f, y + 120.0f * (patch - 0.5f))
                                 + glm::smoothstep(0.80f, 0.62f, nn.y);
                col = glm::mix(col, rock, glm::clamp(rockAmount, 0.0f, 1.0f));
                v.push_back(glm::vec3(x, y, z));
                n.push_back(nn);
                c.push_back(glm::clamp(col, 0.0f, 1.0f));
            }
        }
    return modelFromArrays(v, c, n);
}

// Sky dome, a hemisphere with a colour going from the horizon to the zenith.
// It is drawn around the camera
Model buildSkyDome(void)
{
    const int SLICES = 36, STACKS = 14;
    const glm::vec3 horizon(0.78f, 0.86f, 0.95f), zenith(0.22f, 0.42f, 0.82f);
    std::vector<glm::vec3> v, c, n;
    for (int i = 0; i < SLICES; i++)
        for (int j = 0; j < STACKS; j++) {
            float a0 = 2.0f * 3.14159265f * i / SLICES, a1 = 2.0f * 3.14159265f * (i + 1) / SLICES;
            float e0 = glm::radians(-15.0f + 105.0f * j / STACKS), e1 = glm::radians(-15.0f + 105.0f * (j + 1) / STACKS);
            glm::vec3 p[4] = { glm::vec3(cosf(e0) * cosf(a0), sinf(e0), cosf(e0) * sinf(a0)),
                               glm::vec3(cosf(e0) * cosf(a1), sinf(e0), cosf(e0) * sinf(a1)),
                               glm::vec3(cosf(e1) * cosf(a1), sinf(e1), cosf(e1) * sinf(a1)),
                               glm::vec3(cosf(e1) * cosf(a0), sinf(e1), cosf(e1) * sinf(a0)) };
            int idx[6] = { 0, 1, 2, 0, 2, 3 };
            for (int k = 0; k < 6; k++) {
                glm::vec3 q = p[idx[k]];
                float t = glm::clamp(q.y, 0.0f, 1.0f);
                v.push_back(q);
                n.push_back(-q);
                c.push_back(glm::mix(horizon, zenith, sqrtf(t)));
            }
        }
    return modelFromArrays(v, c, n);
}
