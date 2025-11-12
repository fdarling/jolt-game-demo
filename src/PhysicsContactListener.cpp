#include "PhysicsContactListener.h"

#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/Body.h>

JumpPadListener::JumpPadListener(JPH::BodyID inJumpPadID, JPH::BodyInterface &inBodyInterface) :
    mJumpPadID(inJumpPadID),
    mBodyInterface(inBodyInterface)
{
}

JumpPadListener::~JumpPadListener()
{
}

void JumpPadListener::OnContactAdded(const JPH::Body &inBody1, const JPH::Body &inBody2, const JPH::ContactManifold &inManifold, JPH::ContactSettings &ioSettings)
{
    JPH::BodyID dynamic_id;
    if (inBody1.GetID() == mJumpPadID && inBody2.GetMotionType() == JPH::EMotionType::Dynamic)
    {
        dynamic_id = inBody2.GetID();
    }
    else if (inBody2.GetID() == mJumpPadID && inBody1.GetMotionType() == JPH::EMotionType::Dynamic)
    {
        dynamic_id = inBody1.GetID();
    }
    if (!dynamic_id.IsInvalid())
    {
        const std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
        CooldownMap::const_iterator it = mJumpCooldowns.find(dynamic_id);
        if (it == mJumpCooldowns.end() || (now - it->second) > std::chrono::milliseconds(100))
        {
            mBodiesToJump.insert(dynamic_id);
            mJumpCooldowns[dynamic_id] = now;
        }
    }
}

void JumpPadListener::OnContactPersisted(const JPH::Body &inBody1, const JPH::Body &inBody2, const JPH::ContactManifold &inManifold, JPH::ContactSettings &ioSettings)
{
}

void JumpPadListener::ApplyQueuedJumps()
{
    for (const JPH::BodyID &id : mBodiesToJump)
    {
        JPH::Vec3 vel = mBodyInterface.GetLinearVelocity(id);
        vel.SetY(10.0f);
        mBodyInterface.SetLinearVelocity(id, vel);
    }
    mBodiesToJump.clear();
}

void JumpPadListener::CleanupCooldowns()
{
    const auto now = std::chrono::steady_clock::now();
    const auto cutoff = now - std::chrono::seconds(5);
    CooldownMap::iterator it = mJumpCooldowns.begin();
    while (it != mJumpCooldowns.end())
    {
        if (it->second < cutoff)
            it = mJumpCooldowns.erase(it);
        else
            ++it;
    }
}
