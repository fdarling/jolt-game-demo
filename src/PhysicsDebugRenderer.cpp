#include <GL/glew.h> // must be before any other OpenGL headers
#include <GL/gl.h>
#include <GL/glu.h>

#include "PhysicsDebugRenderer.h"

OpenGLBatch::OpenGLBatch(const JPH::DebugRenderer::Vertex *inVertices, int inVertexCount, const JPH::uint32 *inIndices, int inIndexCount) :
    _refCount(0),
    _indexCount(inIndexCount > 0 ? inIndexCount : inVertexCount)
{
    // generate Vertex Buffer Object
    glGenBuffers(1, &_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, _vbo);
    glBufferData(GL_ARRAY_BUFFER, inVertexCount * sizeof(JPH::DebugRenderer::Vertex), inVertices, GL_STATIC_DRAW);

    // generate Element Buffer Object
    glGenBuffers(1, &_ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _ebo);
    if (inIndices)
    {
        // we were given indices
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, _indexCount * sizeof(GLuint), inIndices, GL_STATIC_DRAW);
    }
    else
    {
        // we need to generate indices
        std::vector<GLuint> seq_indices(_indexCount);
        for (int i = 0; i < _indexCount; ++i)
            seq_indices[i] = i;
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, _indexCount * sizeof(GLuint), seq_indices.data(), GL_STATIC_DRAW);
    }

    // unbind buffers
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

OpenGLBatch::OpenGLBatch(const JPH::DebugRenderer::Triangle *inTriangles, int inTriangleCount) : _refCount(0)
{
    // flatten triangles to vertices
    // TODO: eliminate duplicates
    int vertex_count = inTriangleCount * 3;
    std::vector<JPH::DebugRenderer::Vertex> verts(vertex_count);
    for (int i = 0; i < inTriangleCount; ++i)
    {
        for (int j = 0; j < 3; j++)
        {
            verts.push_back(inTriangles[i].mV[j]);
        }
    }

    // generate indices
    _indexCount = vertex_count;
    std::vector<GLuint> indices(_indexCount);
    for (int i = 0; i < _indexCount; ++i)
        indices[i] = i;

    // upload to VBO
    glGenBuffers(1, &_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, _vbo);
    glBufferData(GL_ARRAY_BUFFER, vertex_count * sizeof(JPH::DebugRenderer::Vertex), verts.data(), GL_STATIC_DRAW);

    // upload to EBO
    glGenBuffers(1, &_ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, _indexCount * sizeof(GLuint), indices.data(), GL_STATIC_DRAW);

    // unbind buffers
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

OpenGLBatch::~OpenGLBatch()
{
    glDeleteBuffers(1, &_vbo);
    glDeleteBuffers(1, &_ebo);
}

void OpenGLBatch::Draw(JPH::ColorArg inColor)
{
    // use the VBO
    glBindBuffer(GL_ARRAY_BUFFER, _vbo);

    // for vertices
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT, sizeof(JPH::DebugRenderer::Vertex), (void*)offsetof(JPH::DebugRenderer::Vertex, mPosition));

    // for normals
    glEnableClientState(GL_NORMAL_ARRAY);
    glNormalPointer(GL_FLOAT, sizeof(JPH::DebugRenderer::Vertex), (void*)offsetof(JPH::DebugRenderer::Vertex, mNormal));

    // for texture coordinates
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glTexCoordPointer(2, GL_FLOAT, sizeof(JPH::DebugRenderer::Vertex), (void*)offsetof(JPH::DebugRenderer::Vertex, mUV));

    // possibly for colors too
    constexpr bool use_per_vertex_color = false;
    if (use_per_vertex_color)
    {
        glEnableClientState(GL_COLOR_ARRAY);
        // TODO this is currently broken... probably the format is wrong
        glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(JPH::DebugRenderer::Vertex), (void*)offsetof(JPH::DebugRenderer::Vertex, mColor));
    }
    else
    {
        // use a uniform color that was passed to this function
        glColor4ub(inColor.r, inColor.g, inColor.b, inColor.a);
    }

    // use the EBO
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _ebo);
    glDrawElements(GL_TRIANGLES, _indexCount, GL_UNSIGNED_INT, 0);

    // clean up OpenGL state
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    if (use_per_vertex_color)
        glDisableClientState(GL_COLOR_ARRAY);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

OpenGLDebugRenderer::OpenGLDebugRenderer()
{
    Initialize(); // we are required to call this! Jolt will crash if you don't
}

OpenGLDebugRenderer::~OpenGLDebugRenderer() = default;

void OpenGLDebugRenderer::DrawLine(JPH::RVec3Arg inFrom, JPH::RVec3Arg inTo, JPH::ColorArg inColor)
{
    glColor4ub(inColor.r, inColor.g, inColor.b, inColor.a);
    glBegin(GL_LINES);
    glVertex3d(inFrom.GetX(), inFrom.GetY(), inFrom.GetZ());
    glVertex3d(inTo.GetX(), inTo.GetY(), inTo.GetZ());
    glEnd();
}

void OpenGLDebugRenderer::DrawTriangle(JPH::RVec3Arg inV1, JPH::RVec3Arg inV2, JPH::RVec3Arg inV3, JPH::ColorArg inColor, ECastShadow inCastShadow)
{
    glColor4ub(inColor.r, inColor.g, inColor.b, inColor.a);
    glBegin(GL_TRIANGLES);
    glVertex3d(inV1.GetX(), inV1.GetY(), inV1.GetZ());
    glVertex3d(inV2.GetX(), inV2.GetY(), inV2.GetZ());
    glVertex3d(inV3.GetX(), inV3.GetY(), inV3.GetZ());
    glEnd();
}

void OpenGLDebugRenderer::DrawText3D(JPH::RVec3Arg inPosition, const std::string_view& inString, JPH::ColorArg inColor, float inHeight)
{
}

JPH::DebugRenderer::Batch OpenGLDebugRenderer::CreateTriangleBatch(const JPH::DebugRenderer::Vertex *inVertices, int inVertexCount, const JPH::uint32 *inIndices, int inIndexCount)
{
    return new OpenGLBatch(inVertices, inVertexCount, inIndices, inIndexCount);
}

JPH::DebugRenderer::Batch OpenGLDebugRenderer::CreateTriangleBatch(const JPH::DebugRenderer::Triangle *inTriangles, int inTriangleCount)
{
    return new OpenGLBatch(inTriangles, inTriangleCount);
}

void OpenGLDebugRenderer::DrawGeometry(JPH::RMat44Arg inModelMatrix, const JPH::AABox &inWorldSpaceBounds, float inLODScaleSq, JPH::ColorArg inModelColor, const JPH::DebugRenderer::GeometryRef &inGeometry, JPH::DebugRenderer::ECullMode inCullMode, JPH::DebugRenderer::ECastShadow inCastShadow, JPH::DebugRenderer::EDrawMode inDrawMode)
{
    GLenum fillMode = GL_FILL;
    if (inDrawMode == JPH::DebugRenderer::EDrawMode::Solid)
        fillMode = GL_FILL;
    else if (inDrawMode == JPH::DebugRenderer::EDrawMode::Wireframe)
        fillMode = GL_LINE;
    glPolygonMode(GL_FRONT_AND_BACK, fillMode);
    glPushMatrix();
    GLfloat mat[16];
    inModelMatrix.StoreFloat4x4((JPH::Float4*)mat);
    glMultMatrixf(mat);

    const JPH::DebugRenderer::LOD &lod = inGeometry->GetLOD(_cameraPos, inWorldSpaceBounds, inLODScaleSq); // TODO determine camera position and

    OpenGLBatch * const batch = reinterpret_cast<OpenGLBatch*>(lod.mTriangleBatch.GetPtr());
    batch->Draw(inModelColor);

    glPopMatrix();
}

void OpenGLDebugRenderer::SetCameraPosition(float x, float y, float z)
{
    _cameraPos = JPH::Vec3(x, y, z);
}
