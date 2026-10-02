/*
    Copyright 2016-2026 melonDS team

    This file is part of melonDS.

    melonDS is free software: you can redistribute it and/or modify it under
    the terms of the GNU General Public License as published by the Free
    Software Foundation, either version 3 of the License, or (at your option)
    any later version.

    melonDS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with melonDS. If not, see http://www.gnu.org/licenses/.
*/

#ifndef OPENGLSUPPORT_H
#define OPENGLSUPPORT_H

#include <stdio.h>
#include <string.h>

#include "Platform.h"
#include "PlatformOGL.h"

// ANDROID: desktop GL functions used by the renderers that GLES 3.2 lacks
inline void glDrawBuffer(GLenum buf) { glDrawBuffers(1, &buf); }
inline void glClearDepth(double depth) { glClearDepthf((GLfloat)depth); }
inline void glDepthRange(double n, double f) { glDepthRangef((GLfloat)n, (GLfloat)f); }
inline void* glMapBuffer(GLenum target, GLenum access)
{
    GLint size;
    glGetBufferParameteriv(target, GL_BUFFER_SIZE, &size);
    return glMapBufferRange(target, 0, size, (access == GL_READ_ONLY) ? GL_MAP_READ_BIT : GL_MAP_WRITE_BIT);
}

namespace melonDS::OpenGL
{

// ANDROID: GLES has no GL_UNSIGNED_SHORT_1_5_5_5_REV, which is the DS color layout (bit 0-4 R, 5-9 G,
// 10-14 B, 15 A). DS colors are converted to GL_UNSIGNED_SHORT_5_5_5_1 for upload, and read back as
// GL_RGBA/GL_UNSIGNED_BYTE.
inline u16 ToRGBA5551(u16 col)
{
    return ((col & 0x1F) << 11) | ((col & 0x3E0) << 1) | ((col >> 9) & 0x3E) | (col >> 15);
}

inline void ToRGBA5551(u16* data, u32 count)
{
    for (u32 i = 0; i < count; i++)
        data[i] = ToRGBA5551(data[i]);
}

inline u16 FromRGBA8(u32 col)
{
    return ((col >> 3) & 0x1F) | (((col >> 11) & 0x1F) << 5) | (((col >> 19) & 0x1F) << 10) | ((col >> 31) << 15);
}

void LoadShaderCache();
void SaveShaderCache();

struct AttributeTarget
{
    const char* Name;
    u32 Location;
};


bool CompileVertexFragmentProgram(GLuint& result,
    const std::string& vs, const std::string& fs,
    const std::string& name,
    const std::initializer_list<AttributeTarget>& vertexInAttrs,
    const std::initializer_list<AttributeTarget>& fragmentOutAttrs);

bool CompileComputeProgram(GLuint& result, const std::string& source, const std::string& name);

}

#endif // OPENGLSUPPORT_H
