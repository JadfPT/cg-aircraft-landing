// objloader.hpp
//
// Loader of .obj files, the format of the models used in the program.

#ifndef OBJLOADER_HPP
#define OBJLOADER_HPP

#include <string>
#include <vector>
#include <glm/glm.hpp>

// one part of an .obj file, all the triangles that use the same material
struct ObjPart {
    std::string material;            // name of the material (usemtl)
    glm::vec3   diffuse;             // colour Kd
    float       alpha;               // opacity d, 1 is opaque
    std::string texture;             // texture map_Kd (path from the program folder), empty if there is none
    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> normals;
    std::vector<glm::vec2> uvs;      // (0, 0) if the file has no texture coordinates
};

// Loads an .obj mesh. Polygons are split into triangles as fans and the four face
// formats v, v/vt, v/vt/vn and v//vn are read, with negative indices too.
// If the file has no normals, each face gets the normal of its plane.
// The materials of the .mtl file (Kd, d and map_Kd) are read and the triangles are
// grouped by material, one ObjPart for each.
// Based on the loader of http://www.opengl-tutorial.org/beginners-tutorials/tutorial-7-model-loading/
bool loadOBJ(const char * path, std::vector<ObjPart> & out_parts);

#endif
