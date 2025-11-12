#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>

namespace PhysicsLayers {
    static constexpr JPH::ObjectLayer NON_MOVING = 0;
    static constexpr JPH::ObjectLayer MOVING = 1;
    static constexpr JPH::ObjectLayer NUM_LAYERS = 2;
} // namespace PhysicsLayers
