#pragma once

#include "PhysicsLayers.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>

class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface
{
public:
    BPLayerInterfaceImpl();

    JPH::uint GetNumBroadPhaseLayers() const override;

    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override;

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char * GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override;
#endif

private:
    JPH::BroadPhaseLayer mObjectToBroadPhase[PhysicsLayers::NUM_LAYERS];
};

class ObjectVsBroadPhaseLayerFilterImpl final : public JPH::ObjectVsBroadPhaseLayerFilter
{
public:
    bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override;
};

class ObjectLayerPairFilterImpl final : public JPH::ObjectLayerPairFilter
{
public:
    bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::ObjectLayer inLayer2) const override;
};
