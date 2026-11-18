// texture.cpp
//
// Loads the images of the textures with stb_image and sends them to the GPU.

#include <stdio.h>
#include <string>
#include <map>

#include <GL/glew.h>

#define STB_IMAGE_IMPLEMENTATION
#include <common/stb_image.h>      // image loader, public domain, https://github.com/nothings/stb

#include <common/texture.hpp>

// textures already loaded, so the same file is only read once
static std::map<std::string, GLuint> textures;

// loads an image into an OpenGL texture with mipmaps, returns 0 if it can not be read
GLuint loadTexture(const char * imagepath)
{
    std::map<std::string, GLuint>::iterator it = textures.find(imagepath);
    if (it != textures.end()) return it->second;

    // OpenGL expects the first row of the image at the bottom (v = 0)
    stbi_set_flip_vertically_on_load(1);
    int width, height, channels;
    unsigned char * data = stbi_load(imagepath, &width, &height, &channels, 4);
    if (data == NULL) {
        printf("Warning: could not load texture %s (%s)\n", imagepath, stbi_failure_reason());
        textures[imagepath] = 0;
        return 0;
    }

    // create one OpenGL texture and give it the image
    GLuint textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
    stbi_image_free(data);

    // trilinear filtering with mipmaps, and the image repeats outside [0, 1]
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glGenerateMipmap(GL_TEXTURE_2D);
    // anisotropic filtering when the card has it, which keeps textures sharp on surfaces
    // seen at a low angle, like the terrain far away
    if (GLEW_EXT_texture_filter_anisotropic) {
        GLfloat maxAniso = 1.0f;
        glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &maxAniso);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, maxAniso < 8.0f ? maxAniso : 8.0f);
    }

    textures[imagepath] = textureID;
    return textureID;
}

// frees all the textures loaded
void deleteTextures(void)
{
    for (std::map<std::string, GLuint>::iterator it = textures.begin(); it != textures.end(); ++it)
        if (it->second) glDeleteTextures(1, &it->second);
    textures.clear();
}
