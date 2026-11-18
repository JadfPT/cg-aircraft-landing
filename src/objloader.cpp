// objloader.cpp
//
// Reads the .obj files of the models (aircraft, runway, people, buildings).

#include <stdio.h>
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>
#include <cstdlib>

#include <glm/glm.hpp>

#include <common/objloader.hpp>

// Very simple OBJ loader in the spirit of the one of opengl-tutorial.org (tutorial 7),
// extended to read polygons with any number of vertices, the four face formats
// and the materials of the .mtl file (colour Kd, opacity d and texture map_Kd)

struct Material {
    glm::vec3 kd = glm::vec3(0.8f);
    float alpha = 1.0f;
    std::string texture;
};

// removes the spaces at the start and end of a string
static std::string trim(const std::string & s)
{
    size_t a = s.find_first_not_of(" \t\r\n");
    size_t b = s.find_last_not_of(" \t\r\n");
    return (a == std::string::npos) ? std::string("") : s.substr(a, b - a + 1);
}

// reads the materials of a .mtl file (newmtl, Kd, d and map_Kd)
static void loadMTL(const std::string & dir, const std::string & file, std::map<std::string, Material> & materials)
{
    std::ifstream in((dir + file).c_str());
    if (!in.is_open()) {
        printf("Warning: could not open material file %s (using default colour)\n", (dir + file).c_str());
        return;
    }
    std::string line, current;
    while (std::getline(in, line)) {
        std::istringstream ss(line);
        std::string tag;
        ss >> tag;
        if (tag == "newmtl") {
            ss >> current;
            materials[current] = Material();
        } else if (current.empty()) {
            continue;
        } else if (tag == "Kd") {
            ss >> materials[current].kd.x >> materials[current].kd.y >> materials[current].kd.z;
        } else if (tag == "d") {
            ss >> materials[current].alpha;
        } else if (tag == "map_Kd") {
            std::string name;
            std::getline(ss, name);
            materials[current].texture = dir + trim(name);
        }
    }
}

// OBJ indices start at 1, or are negative to count from the end. This turns them into
// normal indices that start at 0
static int fixIndex(int idx, int count)
{
    if (idx > 0) return idx - 1;
    if (idx < 0) return count + idx;
    return -1;
}

// Reads the .obj file. The triangles are grouped by material, one ObjPart for each.
// If a face has no normals, the normal of its plane is used
bool loadOBJ(const char * path, std::vector<ObjPart> & out_parts)
{
    printf("Loading OBJ file %s...\n", path);

    std::ifstream file(path);
    if (!file.is_open()) {
        printf("Impossible to open %s ! Are you in the right path ?\n", path);
        return false;
    }

    // folder of the .obj file, the .mtl and the textures are relative to it
    std::string dir(path);
    size_t slash = dir.find_last_of("/\\");
    dir = (slash == std::string::npos) ? std::string("") : dir.substr(0, slash + 1);

    std::vector<glm::vec3> temp_vertices;
    std::vector<glm::vec2> temp_uvs;
    std::vector<glm::vec3> temp_normals;
    std::map<std::string, Material> materials;
    std::map<std::string, int> partIndex;    // index of each material in out_parts
    size_t firstPart = out_parts.size();
    int current = -1;
    int faceCount = 0;

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string tag;
        ss >> tag;

        if (tag == "v") {
            glm::vec3 vertex;
            ss >> vertex.x >> vertex.y >> vertex.z;
            temp_vertices.push_back(vertex);
        } else if (tag == "vt") {
            glm::vec2 uv;
            ss >> uv.x >> uv.y;
            temp_uvs.push_back(uv);
        } else if (tag == "vn") {
            glm::vec3 normal;
            ss >> normal.x >> normal.y >> normal.z;
            temp_normals.push_back(normal);
        } else if (tag == "mtllib") {
            std::string name;
            std::getline(ss, name);
            loadMTL(dir, trim(name), materials);
        } else if (tag == "usemtl" || (tag == "f" && current < 0)) {
            std::string name = "default";
            if (tag == "usemtl") ss >> name;
            std::map<std::string, int>::iterator it = partIndex.find(name);
            if (it == partIndex.end()) {
                ObjPart part;
                Material m = materials.count(name) ? materials[name] : Material();
                part.material = name;
                part.diffuse  = m.kd;
                part.alpha    = m.alpha;
                part.texture  = m.texture;
                out_parts.push_back(part);
                current = (int)out_parts.size() - 1;
                partIndex[name] = current;
            } else {
                current = it->second;
            }
            if (tag == "usemtl") continue;
        }

        if (tag == "f") {
            ObjPart & part = out_parts[current];
            // read all corners of the polygon, in any of the formats v, v/vt, v/vt/vn or v//vn
            std::vector<int> vIdx, tIdx, nIdx;
            std::string corner;
            while (ss >> corner) {
                int v = 0, t = 0, n = 0;
                size_t s1 = corner.find('/');
                v = atoi(corner.substr(0, s1).c_str());
                if (s1 != std::string::npos) {
                    size_t s2 = corner.find('/', s1 + 1);
                    if (s2 == std::string::npos) {
                        t = atoi(corner.substr(s1 + 1).c_str());
                    } else {
                        if (s2 > s1 + 1) t = atoi(corner.substr(s1 + 1, s2 - s1 - 1).c_str());
                        if (s2 + 1 < corner.size()) n = atoi(corner.substr(s2 + 1).c_str());
                    }
                }
                vIdx.push_back(fixIndex(v, (int)temp_vertices.size()));
                tIdx.push_back(fixIndex(t, (int)temp_uvs.size()));
                nIdx.push_back(fixIndex(n, (int)temp_normals.size()));
            }
            // split the polygon into triangles as a fan (0, i, i+1)
            for (size_t i = 1; i + 1 < vIdx.size(); i++) {
                int tri[3] = { 0, (int)i, (int)i + 1 };
                bool ok = true;
                for (int k = 0; k < 3; k++)
                    if (vIdx[tri[k]] < 0 || vIdx[tri[k]] >= (int)temp_vertices.size()) ok = false;
                if (!ok) continue;

                glm::vec3 p0 = temp_vertices[vIdx[tri[0]]];
                glm::vec3 p1 = temp_vertices[vIdx[tri[1]]];
                glm::vec3 p2 = temp_vertices[vIdx[tri[2]]];
                glm::vec3 faceNormal = glm::cross(p1 - p0, p2 - p0);
                float len = glm::length(faceNormal);
                faceNormal = (len > 1e-12f) ? faceNormal / len : glm::vec3(0, 1, 0);

                for (int k = 0; k < 3; k++) {
                    part.vertices.push_back(temp_vertices[vIdx[tri[k]]]);
                    int ti = tIdx[tri[k]];
                    part.uvs.push_back((ti >= 0 && ti < (int)temp_uvs.size()) ? temp_uvs[ti] : glm::vec2(0.0f));
                    int ni = nIdx[tri[k]];
                    if (ni >= 0 && ni < (int)temp_normals.size() && glm::length(temp_normals[ni]) > 1e-6f)
                        part.normals.push_back(glm::normalize(temp_normals[ni]));
                    else
                        part.normals.push_back(faceNormal);
                }
                faceCount++;
            }
        }
        // other tags (o, g, s, comments) are ignored
    }

    // remove the materials that ended up without triangles
    for (size_t i = out_parts.size(); i-- > firstPart; )
        if (out_parts[i].vertices.empty()) out_parts.erase(out_parts.begin() + i);

    printf("   %d vertices, %d uvs, %d normals, %d triangles, %d materials\n",
           (int)temp_vertices.size(), (int)temp_uvs.size(), (int)temp_normals.size(), faceCount, (int)materials.size());
    return faceCount > 0;
}
