// texture.hpp
//
// Textures of the models, loaded with stb_image.

#ifndef TEXTURE_HPP
#define TEXTURE_HPP

#include <GL/glew.h>

// Loads an image (.png, .jpg, .bmp, .tga) into an OpenGL 2D texture with mipmaps.
// The same file is only loaded once. Returns 0 if the file can not be read.
GLuint loadTexture(const char * imagepath);

// frees all the textures loaded by loadTexture
void deleteTextures(void);

#endif
