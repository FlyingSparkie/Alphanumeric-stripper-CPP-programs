// ConsoleApplication2.cpp : Defines the entry point for the console
//application.
//#include "stdafx.h"
#include <iostream>
#include <string>
#include <GLuint.h>

/*
//int _tmain(int argc, _TCHAR * argv[])
int main()
{
std::string mystring;
std::cin >> mystring;
std::cout << "Hello " << mystring;
return 0;
}*/

/* Create a texture with width and height */
GLuint CreateTexture2D(int width, int height, char* TheData)
{
// Texture handle
GLuint textureId;
// Set the alignment
glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
// Generate a texture object
glGenTextures(1, &textureId);
// Bind the texture object
glBindTexture(GL_TEXTURE_2D, textureId);
// set it up
glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA,width,height,0,GL_RGBA,GL_UNSIGNED_BYTE,
TheData);
if (glGetError() != GL_NO_ERROR) printf("Oh bugger");
// Set the filtering mode
glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
if (glGetError() == GL_NO_ERROR) return textureId;
printf("Oh bugger");
return textureId;
}