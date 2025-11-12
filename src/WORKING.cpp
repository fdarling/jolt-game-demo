#include <SDL2/SDL.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Renderer/DebugRendererSimple.h>

#include <iostream>
#include <string_view>
#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <unordered_map>
#include <chrono>

#define JPH_DEBUG_RENDERER 1

// Define layers
namespace Layers {
    static constexpr JPH::ObjectLayer NON_MOVING = 0;
    static constexpr JPH::ObjectLayer MOVING = 1;
    static constexpr JPH::ObjectLayer NUM_LAYERS = 2;
}

// Broad phase layer interface
class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface {
public:
    BPLayerInterfaceImpl() {
        mObjectToBroadPhase[Layers::NON_MOVING] = JPH::BroadPhaseLayer(0);
        mObjectToBroadPhase[Layers::MOVING] = JPH::BroadPhaseLayer(1);
    }

    virtual JPH::uint GetNumBroadPhaseLayers() const override {
        return 2;
    }

    virtual JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override {
        return mObjectToBroadPhase[inLayer];
    }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    virtual const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override {
        switch (static_cast<JPH::BroadPhaseLayer::Type>(inLayer)) {
            case 0: return "NON_MOVING";
            case 1: return "MOVING";
            default: JPH_ASSERT(false); return "INVALID";
        }
    }
#endif

private:
    JPH::BroadPhaseLayer mObjectToBroadPhase[Layers::NUM_LAYERS];
};

// Object vs broad phase layer filter
class ObjectVsBroadPhaseLayerFilterImpl final : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
    virtual bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override {
        switch (inLayer1) {
            case Layers::NON_MOVING:
                return inLayer2 == JPH::BroadPhaseLayer(1); // Non-moving collides with moving
            case Layers::MOVING:
                return true;
            default:
                JPH_ASSERT(false);
                return false;
        }
    }
};

// Object layer pair filter
class ObjectLayerPairFilterImpl final : public JPH::ObjectLayerPairFilter {
public:
    virtual bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::ObjectLayer inLayer2) const override {
        switch (inLayer1) {
            case Layers::NON_MOVING:
                return inLayer2 == Layers::MOVING;
            case Layers::MOVING:
                return true;
            default:
                JPH_ASSERT(false);
                return false;
        }
    }
};

// Custom debug renderer using OpenGL immediate mode
class OpenGLDebugRenderer final : public JPH::DebugRendererSimple {
public:
    OpenGLDebugRenderer() {
        Initialize();
    }

    virtual void DrawLine(JPH::RVec3Arg inFrom, JPH::RVec3Arg inTo, JPH::ColorArg inColor) override {
        glColor4ub(inColor.r, inColor.g, inColor.b, inColor.a);
        glBegin(GL_LINES);
        glVertex3d(inFrom.GetX(), inFrom.GetY(), inFrom.GetZ());
        glVertex3d(inTo.GetX(), inTo.GetY(), inTo.GetZ());
        glEnd();
    }

    virtual void DrawTriangle(JPH::RVec3Arg inV1, JPH::RVec3Arg inV2, JPH::RVec3Arg inV3, JPH::ColorArg inColor, ECastShadow inCastShadow) override {
        glColor4ub(inColor.r, inColor.g, inColor.b, inColor.a);
        glBegin(GL_TRIANGLES);
        glVertex3d(inV1.GetX(), inV1.GetY(), inV1.GetZ());
        glVertex3d(inV2.GetX(), inV2.GetY(), inV2.GetZ());
        glVertex3d(inV3.GetX(), inV3.GetY(), inV3.GetZ());
        glEnd();
    }

    virtual void DrawText3D(JPH::RVec3Arg inPosition, const std::string_view& inString, JPH::ColorArg inColor, float inHeight) override {
        // Skipping text rendering for simplicity (requires additional libraries like SDL_ttf)
    }
};

// Jump pad collision listener (defers jumps to avoid lock violations)
class JumpPadListener final : public JPH::ContactListener {
public:
    JumpPadListener(JPH::BodyID inJumpPadID, JPH::BodyInterface& inBodyInterface)
        : mJumpPadID(inJumpPadID), mBodyInterface(inBodyInterface) {}

    // Queue dynamic body for jump (only on initial contact)
    virtual void OnContactAdded(const JPH::Body& inBody1, const JPH::Body& inBody2, const JPH::ContactManifold& inManifold, JPH::ContactSettings& ioSettings) override {
        JPH::BodyID dynamic_id(0);
        if (inBody1.GetID() == mJumpPadID && inBody2.GetMotionType() == JPH::EMotionType::Dynamic) {
            dynamic_id = inBody2.GetID();
        } else if (inBody2.GetID() == mJumpPadID && inBody1.GetMotionType() == JPH::EMotionType::Dynamic) {
            dynamic_id = inBody1.GetID();
        }
        if (!dynamic_id.IsInvalid()) {
            auto now = std::chrono::steady_clock::now();
            auto it = mJumpCooldowns.find(dynamic_id);
            if (it == mJumpCooldowns.end() || (now - it->second) > std::chrono::milliseconds(100)) {
                mBodiesToJump.insert(dynamic_id);
                mJumpCooldowns[dynamic_id] = now;
            }
        }
    }

    virtual void OnContactPersisted(const JPH::Body& inBody1, const JPH::Body& inBody2, const JPH::ContactManifold& inManifold, JPH::ContactSettings& ioSettings) override {
        // No-op: Only jump on initial contact to avoid continuous force
    }

    // Apply queued jumps (call this after physics.Update() from main thread)
    void ApplyQueuedJumps() {
        for (auto id : mBodiesToJump) {
            JPH::Vec3 vel = mBodyInterface.GetLinearVelocity(id);
            vel.SetY(10.0f);
            mBodyInterface.SetLinearVelocity(id, vel);
        }
        mBodiesToJump.clear();
    }

    // Clear old cooldowns periodically (e.g., every few seconds)
    void CleanupCooldowns() {
        auto now = std::chrono::steady_clock::now();
        auto cutoff = now - std::chrono::seconds(5);
        auto it = mJumpCooldowns.begin();
        while (it != mJumpCooldowns.end()) {
            if (it->second < cutoff) {
                it = mJumpCooldowns.erase(it);
            } else {
                ++it;
            }
        }
    }

private:
    JPH::BodyID mJumpPadID;
    JPH::BodyInterface& mBodyInterface;
    std::unordered_set<JPH::BodyID> mBodiesToJump;
    std::unordered_map<JPH::BodyID, std::chrono::steady_clock::time_point> mJumpCooldowns;
};

int main(int argc, char* argv[]) {
    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL could not initialize! SDL_Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    // Set OpenGL attributes (compatibility profile for immediate mode)
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_COMPATIBILITY);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

    // Create window
    SDL_Window* window = SDL_CreateWindow("Jolt Physics Debug Demo", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 800, 600, SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
    if (window == nullptr) {
        std::cerr << "Window could not be created! SDL_Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return 1;
    }

    // Create OpenGL context
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    if (gl_context == nullptr) {
        std::cerr << "OpenGL context could not be created! SDL_Error: " << SDL_GetError() << std::endl;
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    // Enable depth test and culling
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    // FPS-style camera controls
    SDL_SetRelativeMouseMode(SDL_TRUE);
    SDL_ShowCursor(SDL_FALSE);

    // Initialize Jolt
    JPH::RegisterDefaultAllocator();
    JPH::Factory::sInstance = new JPH::Factory();
    JPH::RegisterTypes();

    // Create instances for layers and filters
    BPLayerInterfaceImpl broad_phase_layer_interface;
    ObjectVsBroadPhaseLayerFilterImpl object_vs_broadphase_layer_filter;
    ObjectLayerPairFilterImpl object_layer_pair_filter;

    // Setup physics system
    JPH::PhysicsSystem physics_system;
    const int max_bodies = 1024;
    const int num_body_mutexes = 0;
    const int max_body_pairs = 65536;
    const int max_contact_constraints = 10240;
    physics_system.Init(max_bodies, num_body_mutexes, max_body_pairs, max_contact_constraints,
                        broad_phase_layer_interface, object_vs_broadphase_layer_filter, object_layer_pair_filter);

    // Set gravity
    physics_system.SetGravity(JPH::Vec3(0, -9.81f, 0));

    // Create debug renderer and set as instance
    OpenGLDebugRenderer* renderer = new OpenGLDebugRenderer();
    JPH::DebugRenderer::sInstance = renderer;

    JPH::BodyInterface& body_interface = physics_system.GetBodyInterface();

    // Create floor
    JPH::BodyCreationSettings floor_settings(new JPH::BoxShape(JPH::Vec3(100.0f, 1.0f, 100.0f)),
                                             JPH::RVec3(0.0, -1.0, 0.0), JPH::Quat::sIdentity(),
                                             JPH::EMotionType::Static, Layers::NON_MOVING);
    JPH::BodyID floor_id = body_interface.CreateAndAddBody(floor_settings, JPH::EActivation::DontActivate);

    // Create jump pad (static box on ground, sized like elevator)
    JPH::BodyCreationSettings jump_pad_settings(new JPH::BoxShape(JPH::Vec3(10.0f, 1.0f, 10.0f)),
                                                JPH::RVec3(30.0, 0.5, 0.0), JPH::Quat::sIdentity(),
                                                JPH::EMotionType::Static, Layers::NON_MOVING);
    JPH::BodyID jump_pad_id = body_interface.CreateAndAddBody(jump_pad_settings, JPH::EActivation::DontActivate);

    // Set up contact listener for jump pad
    JumpPadListener* jump_listener = new JumpPadListener(jump_pad_id, body_interface);
    physics_system.SetContactListener(jump_listener);

    // Create elevator (kinematic box platform)
    JPH::BodyCreationSettings elevator_settings(new JPH::BoxShape(JPH::Vec3(10.0f, 1.0f, 10.0f)),
                                                JPH::RVec3(0.0, 1.0, 0.0), JPH::Quat::sIdentity(),
                                                JPH::EMotionType::Kinematic, Layers::MOVING);
    JPH::BodyID elevator_id = body_interface.CreateAndAddBody(elevator_settings, JPH::EActivation::Activate);

    // Create stack of 3 boxes on the elevator (offset to the side, leaving room for sphere in center)
    JPH::Ref<JPH::Shape> box_shape = new JPH::BoxShape(JPH::Vec3(3.0f, 0.3f, 3.0f));  // ~1/3 size of elevator
    JPH::BodyCreationSettings box_settings(box_shape, JPH::RVec3(0.0, 0.0, 0.0), JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic, Layers::MOVING);
    box_settings.mLinearDamping = 0.01f;
    box_settings.mAngularDamping = 0.01f;
    box_settings.mRestitution = 0.5f;

    // Bottom box
    box_settings.mPosition = JPH::RVec3(-8.0, 2.3, 0.0);  // On elevator top (y=2) + half-height (0.3)
    JPH::BodyID box1_id = body_interface.CreateAndAddBody(box_settings, JPH::EActivation::Activate);

    // Middle box (stacked on bottom)
    box_settings.mPosition = JPH::RVec3(-8.0, 2.9, 0.0);  // Bottom top (2.3 + 0.6)
    JPH::BodyID box2_id = body_interface.CreateAndAddBody(box_settings, JPH::EActivation::Activate);

    // Top box (stacked on middle)
    box_settings.mPosition = JPH::RVec3(-8.0, 3.5, 0.0);  // Middle top (2.9 + 0.6)
    JPH::BodyID box3_id = body_interface.CreateAndAddBody(box_settings, JPH::EActivation::Activate);

    // Create falling sphere (starts above center, room to interact without hitting stack immediately)
    JPH::BodyCreationSettings sphere_settings(new JPH::SphereShape(2.0f),
                                              JPH::RVec3(0.0, 10.0, 0.0), JPH::Quat::sIdentity(),
                                              JPH::EMotionType::Dynamic, Layers::MOVING);
    sphere_settings.mLinearDamping = 0.01f;
    sphere_settings.mAngularDamping = 0.01f;
    sphere_settings.mRestitution = 0.5f; // Some bounciness
    JPH::BodyID sphere_id = body_interface.CreateAndAddBody(sphere_settings, JPH::EActivation::Activate);

    // Temp allocator and job system (simple single-threaded for demo)
    JPH::TempAllocatorImpl temp_allocator(10 * 1024 * 1024);
    JPH::JobSystemThreadPool job_system(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, 0); // 0 threads for single-threaded

    // Camera variables
    float camera_x = 0.0f;
    float camera_y = 10.0f;
    float camera_z = 30.0f;
    float yaw = 0.0f;
    float pitch = 0.0f;
    const float mouse_sensitivity = 0.002f;
    const float move_speed = 50.0f;
    const float PI = 3.1415926535f;

    // Elevator animation variables
    float total_time = 0.0f;
    const float cycle_time = 12.0f; // 4s up + 2s pause + 4s down + 2s pause
    const float travel_time = 4.0f; // Time to travel 20m at 5m/s
    const float pause_time = 2.0f;
    const float min_y = 1.0f;
    const float max_y = 21.0f;

    // Main loop
    bool quit = false;
    float time_accumulator = 0.0;
    Uint32 last_time = SDL_GetTicks();
    while (!quit) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                quit = true;
            } else if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.scancode == SDL_SCANCODE_ESCAPE) {
                    quit = true;
                    SDL_SetRelativeMouseMode(SDL_FALSE);
                    SDL_ShowCursor(SDL_TRUE);
                }
            } else if (event.type == SDL_MOUSEMOTION) {
                // Mouse look
                yaw += static_cast<float>(event.motion.xrel) * mouse_sensitivity;
                pitch -= static_cast<float>(event.motion.yrel) * mouse_sensitivity;
                // Clamp pitch
                if (pitch > PI / 2.0f - 0.01f) pitch = PI / 2.0f - 0.01f;
                if (pitch < -PI / 2.0f + 0.01f) pitch = -PI / 2.0f + 0.01f;
            } else if (event.type == SDL_MOUSEBUTTONDOWN) {
                if (event.button.button == SDL_BUTTON_LEFT) {
                    // Shoot a ball
                    JPH::Vec3 forward(cos(yaw) * cos(pitch),
                                      sin(pitch),
                                      sin(yaw) * cos(pitch));
                    JPH::Vec3 velocity = forward * 5.0f;

                    JPH::BodyCreationSettings bullet_settings(new JPH::SphereShape(0.5f),
                                                              JPH::RVec3(camera_x, camera_y, camera_z),
                                                              JPH::Quat::sIdentity(),
                                                              JPH::EMotionType::Dynamic, Layers::MOVING);
                    bullet_settings.mLinearVelocity = velocity;
                    bullet_settings.mLinearDamping = 0.01f;
                    bullet_settings.mAngularDamping = 0.01f;
                    bullet_settings.mRestitution = 0.5f;
                    body_interface.CreateAndAddBody(bullet_settings, JPH::EActivation::Activate);
                }
            }
        }

        // Calculate delta time
        Uint32 current_time = SDL_GetTicks();
        float delta_time = (current_time - last_time) / 1000.0f;
        last_time = current_time;

        // Update physics (1 collision step, 0 sub steps for simplicity)
        const int MAX_SUBSTEPS = 10;
        const float FIXED_TIMESTEP = 1.0/60.0;
        time_accumulator += delta_time;
        for (int i = 0; i < MAX_SUBSTEPS && time_accumulator >= FIXED_TIMESTEP; i++, time_accumulator -= FIXED_TIMESTEP)
        {
            // Animate elevator using MoveKinematic (before physics update)
            total_time += FIXED_TIMESTEP;
            float t = fmod(total_time, cycle_time);
            float target_y;
            if (t < travel_time) {
                // Moving up
                target_y = min_y + (max_y - min_y) * (t / travel_time);
            } else if (t < travel_time + pause_time) {
                // Pause at top
                target_y = max_y;
            } else if (t < 2 * travel_time + pause_time) {
                // Moving down
                float down_t = t - (travel_time + pause_time);
                target_y = max_y - (max_y - min_y) * (down_t / travel_time);
            } else {
                // Pause at bottom
                target_y = min_y;
            }

            body_interface.MoveKinematic(elevator_id,
                                         JPH::RVec3(0.0, target_y, 0.0),
                                         JPH::Quat::sIdentity(), FIXED_TIMESTEP);

            physics_system.Update(FIXED_TIMESTEP, 1, &temp_allocator, &job_system);
        }

        // Apply deferred jumps (safe post-step)
        jump_listener->ApplyQueuedJumps();
        if (fmod(total_time, 5.0f) < delta_time) {  // Cleanup roughly every 5s
            jump_listener->CleanupCooldowns();
        }

        // Update camera position (FPS-style: WASD for movement, Space/Shift for up/down)
        const Uint8* state = SDL_GetKeyboardState(NULL);
        float forward_x = cos(yaw) * cos(pitch);
        float forward_y = sin(pitch);
        float forward_z = sin(yaw) * cos(pitch);
        float right_x = -sin(yaw);
        float right_z = cos(yaw);

        float dx = 0.0f, dy = 0.0f, dz = 0.0f;
        if (state[SDL_SCANCODE_W]) {
            dx += forward_x * move_speed * delta_time;
            dy += forward_y * move_speed * delta_time;
            dz += forward_z * move_speed * delta_time;
        }
        if (state[SDL_SCANCODE_S]) {
            dx -= forward_x * move_speed * delta_time;
            dy -= forward_y * move_speed * delta_time;
            dz -= forward_z * move_speed * delta_time;
        }
        if (state[SDL_SCANCODE_A]) {
            dx -= right_x * move_speed * delta_time;
            dz -= right_z * move_speed * delta_time;
        }
        if (state[SDL_SCANCODE_D]) {
            dx += right_x * move_speed * delta_time;
            dz += right_z * move_speed * delta_time;
        }
        if (state[SDL_SCANCODE_SPACE]) {
            dy += move_speed * delta_time;
        }
        if (state[SDL_SCANCODE_LSHIFT] || state[SDL_SCANCODE_RSHIFT]) {
            dy -= move_speed * delta_time;
        }

        camera_x += dx;
        camera_y += dy;
        camera_z += dz;

        // Clear screen
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Set projection
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(60.0, 800.0 / 600.0, 0.1, 1000.0);

        // Set view (dynamic camera)
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt(camera_x, camera_y, camera_z,  // Eye position
                  camera_x + forward_x, camera_y + forward_y, camera_z + forward_z,  // Look at
                  0.0, 1.0, 0.0);  // Up

        // Draw physics debug
        JPH::BodyManager::DrawSettings draw_settings;
        draw_settings.mDrawShape = true;
        draw_settings.mDrawShapeWireframe = true; // Wireframe for debug
        draw_settings.mDrawBoundingBox = false;
        physics_system.DrawBodies(draw_settings, JPH::DebugRenderer::sInstance);
        physics_system.DrawConstraints(JPH::DebugRenderer::sInstance);

        // Swap buffers
        SDL_GL_SwapWindow(window);
    }

    // Cleanup
    SDL_SetRelativeMouseMode(SDL_FALSE);
    SDL_ShowCursor(SDL_TRUE);

    physics_system.GetBodyInterface().RemoveBody(sphere_id);
    physics_system.GetBodyInterface().DestroyBody(sphere_id);
    physics_system.GetBodyInterface().RemoveBody(box3_id);
    physics_system.GetBodyInterface().DestroyBody(box3_id);
    physics_system.GetBodyInterface().RemoveBody(box2_id);
    physics_system.GetBodyInterface().DestroyBody(box2_id);
    physics_system.GetBodyInterface().RemoveBody(box1_id);
    physics_system.GetBodyInterface().DestroyBody(box1_id);
    physics_system.GetBodyInterface().RemoveBody(elevator_id);
    physics_system.GetBodyInterface().DestroyBody(elevator_id);
    physics_system.GetBodyInterface().RemoveBody(jump_pad_id);
    physics_system.GetBodyInterface().DestroyBody(jump_pad_id);
    physics_system.GetBodyInterface().RemoveBody(floor_id);
    physics_system.GetBodyInterface().DestroyBody(floor_id);

    delete jump_listener;
    physics_system.SetContactListener(nullptr);

    delete JPH::DebugRenderer::sInstance;
    JPH::DebugRenderer::sInstance = nullptr;

    delete JPH::Factory::sInstance;
    JPH::Factory::sInstance = nullptr;

    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}