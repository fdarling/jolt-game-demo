#include "PhysicsBroadphase.h"

BPLayerInterfaceImpl::BPLayerInterfaceImpl()
{
    mObjectToBroadPhase[PhysicsLayers::NON_MOVING] = JPH::BroadPhaseLayer(0);
    mObjectToBroadPhase[PhysicsLayers::MOVING] = JPH::BroadPhaseLayer(1);
}

JPH::uint BPLayerInterfaceImpl::GetNumBroadPhaseLayers() const
{
    return 2;
}

JPH::BroadPhaseLayer BPLayerInterfaceImpl::GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const
{
    return mObjectToBroadPhase[inLayer];
}

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
const char * BPLayerInterfaceImpl::GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const
{
    switch (static_cast<JPH::BroadPhaseLayer::Type>(inLayer))
    {
        case 0: return "NON_MOVING";
        case 1: return "MOVING";
        default: JPH_ASSERT(false); return "INVALID";
    }
}
#endif

bool ObjectVsBroadPhaseLayerFilterImpl::ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const
{
    switch (inLayer1)
    {
        case PhysicsLayers::NON_MOVING:
            return inLayer2 == JPH::BroadPhaseLayer(1); // Non-moving collides with moving
        case PhysicsLayers::MOVING:
            return true;
        default:
            JPH_ASSERT(false);
            return false;
    }
}

bool ObjectLayerPairFilterImpl::ShouldCollide(JPH::ObjectLayer inLayer1, JPH::ObjectLayer inLayer2) const
{
    switch (inLayer1)
    {
        case PhysicsLayers::NON_MOVING:
            return inLayer2 == PhysicsLayers::MOVING;
        case PhysicsLayers::MOVING:
            return true;
        default:
            JPH_ASSERT(false);
            return false;
    }
}