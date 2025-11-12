#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/ContactListener.h>

#include <chrono>
#include <unordered_set>
#include <unordered_map>

JPH_NAMESPACE_BEGIN
class BodyInterface;
JPH_NAMESPACE_END

class JumpPadListener final : public JPH::ContactListener {
public:
    JumpPadListener(JPH::BodyID inJumpPadID, JPH::BodyInterface &inBodyInterface);
    ~JumpPadListener();

    void OnContactAdded(const JPH::Body &inBody1, const JPH::Body &inBody2, const JPH::ContactManifold &inManifold, JPH::ContactSettings &ioSettings) override;
    void OnContactPersisted(const JPH::Body &inBody1, const JPH::Body &inBody2, const JPH::ContactManifold &inManifold, JPH::ContactSettings &ioSettings) override;

    void ApplyQueuedJumps();
    void CleanupCooldowns();
private:
    JPH::BodyID mJumpPadID;
    JPH::BodyInterface &mBodyInterface;
    std::unordered_set<JPH::BodyID> mBodiesToJump;
    typedef std::unordered_map<JPH::BodyID, std::chrono::steady_clock::time_point> CooldownMap;
    CooldownMap mJumpCooldowns;
};
