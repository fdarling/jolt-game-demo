#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Renderer/DebugRenderer.h>

class OpenGLBatch : public JPH::RefTargetVirtual
{
public:
    JPH_OVERRIDE_NEW_DELETE

    OpenGLBatch(const JPH::DebugRenderer::Vertex *inVertices, int inVertexCount, const JPH::uint32 *inIndices, int inIndexCount);
    OpenGLBatch(const JPH::DebugRenderer::Triangle *inTriangles, int inTriangleCount);
    ~OpenGLBatch() override;
    void AddRef()
    {
        _refCount++;
    }
    void Release()
    {
        _refCount--;
        if (_refCount == 0)
            delete this;
    }
    void Draw(JPH::ColorArg inColor);
protected:
    int _refCount = 0;
    GLuint _vbo = 0;
    GLuint _ebo = 0;
    int _indexCount = 0;
};

class OpenGLDebugRenderer final : public JPH::DebugRenderer
{
public:
    JPH_OVERRIDE_NEW_DELETE
    OpenGLDebugRenderer();
    ~OpenGLDebugRenderer() override;

    void DrawLine(JPH::RVec3Arg inFrom, JPH::RVec3Arg inTo, JPH::ColorArg inColor) override;
    void DrawTriangle(JPH::RVec3Arg inV1, JPH::RVec3Arg inV2, JPH::RVec3Arg inV3, JPH::ColorArg inColor, ECastShadow inCastShadow) override;
    void DrawText3D(JPH::RVec3Arg inPosition, const std::string_view& inString, JPH::ColorArg inColor, float inHeight) override;
    JPH::DebugRenderer::Batch CreateTriangleBatch(const JPH::DebugRenderer::Vertex *inVertices, int inVertexCount, const JPH::uint32 *inIndices, int inIndexCount) override;
    JPH::DebugRenderer::Batch CreateTriangleBatch(const JPH::DebugRenderer::Triangle *inTriangles, int inTriangleCount) override;
    void DrawGeometry(JPH::RMat44Arg inModelMatrix, const JPH::AABox &inWorldSpaceBounds, float inLODScaleSq, JPH::ColorArg inModelColor, const JPH::DebugRenderer::GeometryRef &inGeometry, JPH::DebugRenderer::ECullMode inCullMode=JPH::DebugRenderer::ECullMode::CullBackFace, JPH::DebugRenderer::ECastShadow inCastShadow=JPH::DebugRenderer::ECastShadow::On, JPH::DebugRenderer::EDrawMode inDrawMode=JPH::DebugRenderer::EDrawMode::Solid) override;

    void SetCameraPosition(float x, float y, float z);
protected:
    JPH::Vec3 _cameraPos;
};
