#include <GL/glew.h> // must be before any other OpenGL headers
#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/Body.h>

#include <iostream> // for std::cout, std::cerr, and std::cout
#include <algorithm> // for std::clamp
#include <memory> // for std::unique_ptr
#include <cmath> // for sin(), cos(), and M_PI

#include "PhysicsLayers.h"
#include "PhysicsBroadphase.h"
#include "PhysicsDebugRenderer.h"
#include "PhysicsContactListener.h"

// constants
constexpr float FLOOR_WIDTH = 25.0;
constexpr float FLOOR_THICKNESS = 1.0;
constexpr float ELEVATOR_THICKNESS = 0.25;
constexpr float ELEVATOR_WIDTH = 6.0;
constexpr float JUMP_PAD_THICKNESS = 0.25;
constexpr float JUMP_PAD_WIDTH = 2.0;
constexpr float STACKED_BOX_THICKNESS = 0.125;
constexpr float STACKED_BOX_WIDTH = 2.0;
constexpr float SPHERE_DIAMETER = 1.0;
constexpr float BALL_DIAMETER = 0.5;
constexpr float BALL_INITIAL_SPEED = 15.0;
constexpr float ELEVATOR_TRAVEL_TIME = 4.0;
constexpr float ELEVATOR_PAUSE_TIME = 3.0;
constexpr float ELEVATOR_CYCLE_TIME = 2.0*ELEVATOR_TRAVEL_TIME + 2.0*ELEVATOR_PAUSE_TIME;
constexpr float ELEVATOR_LOWER_Y = ELEVATOR_THICKNESS/2.0;
constexpr float ELEVATOR_UPPER_Y = 22.0 - ELEVATOR_THICKNESS/2.0;
constexpr float LADDER_HEIGHT = 10.0;
constexpr float LADDER_WIDTH = 2.0;
constexpr float BALLOON_DIAMETER = 1.0;
constexpr float BALLOON_RADIUS = BALLOON_DIAMETER/2.0;
constexpr float MOUSELOOK_SENSITIVITY = 0.002f;
constexpr float WALK_SPEED = 20.0f;

// configure Jolt physics maximum limits
constexpr int MAX_SUBSTEPS = 10;
constexpr float FIXED_TIMESTEP = 1.0/120.0;
constexpr int JOLT_MAX_BODIES = 1024;
constexpr int JOLT_NUM_BODY_MUTEXES = 0;
constexpr int JOLT_MAX_BODY_PAIRS = 65536;
constexpr int JOLT_MAX_CONTACT_CONSTRAINTS = 10240;

// forward declaration
static int mainBody(int argc, char **argv);

int main(int argc, char **argv)
{
    if (SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        std::cerr << "SDL could not initialize! SDL_Error: " << SDL_GetError() << std::endl;
        return EXIT_FAILURE;
    }

    const int returnValue = mainBody(argc, argv);

    SDL_Quit();

    return returnValue;
}

static int mainBody(int argc, char **argv)
{
    // request OpenGL attributes
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_COMPATIBILITY);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

    // create a window
    std::unique_ptr<SDL_Window, decltype(&SDL_DestroyWindow)> window(
        SDL_CreateWindow(
            "Jolt Physics Debug Demo",
            SDL_WINDOWPOS_UNDEFINED,
            SDL_WINDOWPOS_UNDEFINED,
            1440,
            900,
            SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN
        ),
        SDL_DestroyWindow
    );
    if (!window)
    {
        std::cerr << "Window could not be created! SDL_Error: " << SDL_GetError() << std::endl;
        return EXIT_FAILURE;
    }

    // create OpenGL context
    std::unique_ptr<std::remove_pointer<SDL_GLContext>::type, decltype(&SDL_GL_DeleteContext)> gl_context(SDL_GL_CreateContext(window.get()), SDL_GL_DeleteContext);
    if (!gl_context)
    {
        std::cerr << "OpenGL context could not be created! SDL_Error: " << SDL_GetError() << std::endl;
        return EXIT_FAILURE;
    }

    // initialize GLEW (to get all the OpenGL functions of interest at runtime)
    {
        GLenum err = GLEW_OK;
        if ((err = glewInit()) != GLEW_OK)
        {
            std::cerr << "OpenGL GLEW could not initialize! GLEW Error: " << glewGetErrorString(err) << std::endl;
            return EXIT_FAILURE;
        }
    }

    // OpenGL settings
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    // grab the mouse and hide the cursor for "mouselook" camera
    SDL_SetRelativeMouseMode(SDL_TRUE);
    SDL_ShowCursor(SDL_FALSE);

    // setup Jolt's memory allocator before instantiating anything
    JPH::RegisterDefaultAllocator();

    // setup Jolt JPH::Factory singleton
    auto factory_deleter = [](JPH::Factory *ptr) {
        JPH::UnregisterTypes();
        JPH::Factory::sInstance = nullptr;
        delete ptr;
    };
    std::unique_ptr<JPH::Factory, decltype(factory_deleter)> factory(new JPH::Factory(), factory_deleter);
    JPH::Factory::sInstance = factory.get();
    JPH::RegisterTypes();

    // setup Jolt physics system
    BPLayerInterfaceImpl broad_phase_layer_interface;
    ObjectVsBroadPhaseLayerFilterImpl object_vs_broadphase_layer_filter;
    ObjectLayerPairFilterImpl object_layer_pair_filter;
    JPH::PhysicsSystem physics_system;
    physics_system.Init(JOLT_MAX_BODIES, JOLT_NUM_BODY_MUTEXES, JOLT_MAX_BODY_PAIRS, JOLT_MAX_CONTACT_CONSTRAINTS,
                        broad_phase_layer_interface, object_vs_broadphase_layer_filter, object_layer_pair_filter);
    physics_system.SetGravity(JPH::Vec3(0, -9.81f, 0));

    // create debug renderer
    std::unique_ptr<OpenGLDebugRenderer> renderer(new OpenGLDebugRenderer()); // NOTE: JPH::DebugRenderer::DebugRenderer() sets the singleton pointer!

    // use the "body interface" as a sort of factory for the bodies we'll create
    JPH::BodyInterface& body_interface = physics_system.GetBodyInterface();

    // add a floor box
    JPH::BodyCreationSettings floor_settings(new JPH::BoxShape(JPH::Vec3(FLOOR_WIDTH/2.0, FLOOR_THICKNESS/2.0, FLOOR_WIDTH/2.0)),
                                             JPH::RVec3(0.0, -FLOOR_THICKNESS/2.0, 0.0), JPH::Quat::sIdentity(),
                                             JPH::EMotionType::Static, PhysicsLayers::NON_MOVING);
    JPH::BodyID floor_id = body_interface.CreateAndAddBody(floor_settings, JPH::EActivation::DontActivate);

    // create a "jump pad" (anything that touches it gets launched upward)
    JPH::BodyCreationSettings jump_pad_settings(new JPH::BoxShape(JPH::Vec3(JUMP_PAD_WIDTH/2.0, JUMP_PAD_THICKNESS/2.0, JUMP_PAD_WIDTH/2.0)),
                                                JPH::RVec3(7.0, JUMP_PAD_THICKNESS/2.0, 7.0), JPH::Quat::sIdentity(),
                                                JPH::EMotionType::Static, PhysicsLayers::NON_MOVING);
    jump_pad_settings.mIsSensor = true;
    JPH::BodyID jump_pad_id = body_interface.CreateAndAddBody(jump_pad_settings, JPH::EActivation::DontActivate);

    // set up contact listener (only for the jump pad currently)
    auto listener_deleter = [&](JumpPadListener *ptr) {
        physics_system.SetContactListener(nullptr);
        delete ptr;
    };
    std::unique_ptr<JumpPadListener, decltype(listener_deleter)> jump_listener(new JumpPadListener(jump_pad_id, body_interface), listener_deleter);
    physics_system.SetContactListener(jump_listener.get());

    // add elevator (kinematic box, animated to move up and down)
    JPH::BodyCreationSettings elevator_settings(new JPH::BoxShape(JPH::Vec3(ELEVATOR_WIDTH/2.0, ELEVATOR_THICKNESS/2.0, ELEVATOR_WIDTH/2.0)),
                                                JPH::RVec3(0.0, ELEVATOR_LOWER_Y, 0.0), JPH::Quat::sIdentity(),
                                                JPH::EMotionType::Kinematic, PhysicsLayers::MOVING);
    JPH::BodyID elevator_id = body_interface.CreateAndAddBody(elevator_settings, JPH::EActivation::Activate);

    // add stack of 3 boxes on the elevator (offset to the side, leaving room for sphere in center)
    JPH::Ref<JPH::Shape> box_shape = new JPH::BoxShape(JPH::Vec3(STACKED_BOX_WIDTH/2.0, STACKED_BOX_THICKNESS/2.0, STACKED_BOX_WIDTH/2.0));
    JPH::BodyCreationSettings box_settings(box_shape, JPH::RVec3(0.0, 0.0, 0.0), JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic, PhysicsLayers::MOVING);
    // TODO set the mass!
    box_settings.mLinearDamping = 0.01f;
    box_settings.mAngularDamping = 0.01f;
    box_settings.mRestitution = 0.5f;
    box_settings.mPosition = JPH::RVec3(-ELEVATOR_WIDTH/2.0 + STACKED_BOX_WIDTH/2.0, ELEVATOR_THICKNESS + 0.5*STACKED_BOX_THICKNESS, -ELEVATOR_WIDTH/2.0 + STACKED_BOX_WIDTH/2.0);  // On elevator top (y=2) + half-height (0.3)
    JPH::BodyID box1_id = body_interface.CreateAndAddBody(box_settings, JPH::EActivation::Activate);
    box_settings.mPosition = JPH::RVec3(-ELEVATOR_WIDTH/2.0 + STACKED_BOX_WIDTH/2.0, ELEVATOR_THICKNESS + 1.5*STACKED_BOX_THICKNESS, -ELEVATOR_WIDTH/2.0 + STACKED_BOX_WIDTH/2.0);  // Bottom top (2.3 + 0.6)
    JPH::BodyID box2_id = body_interface.CreateAndAddBody(box_settings, JPH::EActivation::Activate);
    box_settings.mPosition = JPH::RVec3(-ELEVATOR_WIDTH/2.0 + STACKED_BOX_WIDTH/2.0, ELEVATOR_THICKNESS + 2.5*STACKED_BOX_THICKNESS, -ELEVATOR_WIDTH/2.0 + STACKED_BOX_WIDTH/2.0);  // Middle top (2.9 + 0.6)
    JPH::BodyID box3_id = body_interface.CreateAndAddBody(box_settings, JPH::EActivation::Activate);

    // create falling sphere (starts above elevator, falls onto it)
    JPH::BodyCreationSettings sphere_settings(new JPH::SphereShape(SPHERE_DIAMETER/2.0),
                                              JPH::RVec3(0.0, 10.0, 0.0), JPH::Quat::sIdentity(),
                                              JPH::EMotionType::Dynamic, PhysicsLayers::MOVING);
    // TODO set the mass!
    sphere_settings.mLinearDamping = 0.01f;
    sphere_settings.mAngularDamping = 0.01f;
    sphere_settings.mRestitution = 0.1f; // Some bounciness
    JPH::BodyID sphere_id = body_interface.CreateAndAddBody(sphere_settings, JPH::EActivation::Activate);

    // Jolt physics temp allocator and job system (single-threaded for now)
    JPH::TempAllocatorImpl temp_allocator(10 * 1024 * 1024);
    JPH::JobSystemThreadPool job_system(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, 0); // 0 threads for single-threaded

    // create "ladder" box
    JPH::BodyCreationSettings ladder_settings(new JPH::BoxShape(JPH::Vec3(LADDER_WIDTH/2.0, LADDER_HEIGHT/2.0, LADDER_WIDTH/2.0)), JPH::RVec3(-6.0, LADDER_HEIGHT/2.0, 6.0), JPH::Quat::sIdentity(),
                                              JPH::EMotionType::Static, PhysicsLayers::NON_MOVING);
    JPH::BodyID ladder_id = body_interface.CreateAndAddBody(ladder_settings, JPH::EActivation::DontActivate);

    // create "balloon" (no gravity, floating) that will be attached to the "ladder"
    JPH::BodyCreationSettings balloon_settings(new JPH::SphereShape(BALLOON_DIAMETER/2.0), JPH::RVec3(0.0, 12.0, 0.0), JPH::Quat::sIdentity(),
                                               JPH::EMotionType::Dynamic, PhysicsLayers::MOVING);
    balloon_settings.mGravityFactor = 0.0f;
    JPH::BodyID balloon_id = body_interface.CreateAndAddBody(balloon_settings, JPH::EActivation::Activate);

    // create constraint to "attach" the balloon to the surface of the "ladder"
    JPH::SixDOFConstraintSettings constraint_settings;
    constexpr float LADDER_MARGIN = 0.001;
    constraint_settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
    constraint_settings.mLimitMin[JPH::SixDOFConstraintSettings::EAxis::TranslationX] = -LADDER_WIDTH/2.0 - BALLOON_RADIUS - LADDER_MARGIN;
    constraint_settings.mLimitMax[JPH::SixDOFConstraintSettings::EAxis::TranslationX] =  LADDER_WIDTH/2.0 + BALLOON_RADIUS + LADDER_MARGIN;
    constraint_settings.mLimitMin[JPH::SixDOFConstraintSettings::EAxis::TranslationY] = -LADDER_HEIGHT/2.0 - BALLOON_RADIUS - LADDER_MARGIN;
    constraint_settings.mLimitMax[JPH::SixDOFConstraintSettings::EAxis::TranslationY] =  LADDER_HEIGHT/2.0 + BALLOON_RADIUS + LADDER_MARGIN;
    constraint_settings.mLimitMin[JPH::SixDOFConstraintSettings::EAxis::TranslationZ] = -LADDER_WIDTH/2.0 - BALLOON_RADIUS - LADDER_MARGIN;
    constraint_settings.mLimitMax[JPH::SixDOFConstraintSettings::EAxis::TranslationZ] =  LADDER_WIDTH/2.0 + BALLOON_RADIUS + LADDER_MARGIN;
    JPH::Ref<JPH::SixDOFConstraint> constraint = static_cast<JPH::SixDOFConstraint *>(body_interface.CreateConstraint(&constraint_settings, ladder_id, balloon_id));
    physics_system.AddConstraint(constraint); // TODO is this necessary?

    // for balls we shoot, spawned in the main loop
    std::vector<JPH::BodyID> ball_ids;

    // camera state
    float camera_x = 0.0f;
    float camera_y = 2.5f;
    float camera_z = 10.0f;
    float camera_yaw = -M_PI/2.0; // face the elevator
    float camera_pitch = 0.0f;

    // elevator state
    float total_time = 0.0f;

    // Main loop
    bool want_quit = false;
    float physics_time_accumulator = 0.0;
    Uint32 last_time = SDL_GetTicks();
    while (!want_quit)
    {
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT)
            {
                want_quit = true;
                break;
            }
            else if (event.type == SDL_KEYDOWN)
            {
                if (event.key.keysym.scancode == SDL_SCANCODE_ESCAPE)
                {
                    want_quit = true;
                    break;
                }
            }
            else if (event.type == SDL_MOUSEMOTION)
            {
                // mouse look
                camera_yaw += static_cast<float>(event.motion.xrel) * MOUSELOOK_SENSITIVITY;
                camera_pitch -= static_cast<float>(event.motion.yrel) * MOUSELOOK_SENSITIVITY;
                const float CAMERA_GIMBAL_LOCK_DISTANCE = 0.01;
                camera_pitch = std::clamp<float>(camera_pitch, -M_PI / 2.0f + CAMERA_GIMBAL_LOCK_DISTANCE, M_PI / 2.0f - CAMERA_GIMBAL_LOCK_DISTANCE);
            }
            else if (event.type == SDL_MOUSEBUTTONDOWN)
            {
                if (event.button.button == SDL_BUTTON_LEFT)
                {
                    // shoot a ball
                    const JPH::Vec3 forward(cos(camera_yaw) * cos(camera_pitch),
                                            sin(camera_pitch),
                                            sin(camera_yaw) * cos(camera_pitch));
                    const JPH::Vec3 velocity = forward * BALL_INITIAL_SPEED;

                    JPH::BodyCreationSettings bullet_settings(new JPH::SphereShape(BALL_DIAMETER/2.0),
                                                              JPH::RVec3(camera_x, camera_y, camera_z),
                                                              JPH::Quat::sIdentity(),
                                                              JPH::EMotionType::Dynamic, PhysicsLayers::MOVING);
                    // TODO set the mass!
                    // JPH::MassProperties msp;
                    // msp.ScaleToMass(1.0); // kg
                    // bullet_settings.mMassPropertiesOverride = msp;
                    // bullet_settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
                    bullet_settings.mLinearVelocity = velocity;
                    bullet_settings.mLinearDamping = 0.01f;
                    bullet_settings.mAngularDamping = 0.01f;
                    bullet_settings.mRestitution = 0.1f;
                    const JPH::BodyID ball_id = body_interface.CreateAndAddBody(bullet_settings, JPH::EActivation::Activate);
                    ball_ids.push_back(ball_id);
                }
            }
        }

        // break out of both loops if we are quitting
        if (want_quit)
            break;

        // calculate frame time
        const Uint32 current_time = SDL_GetTicks();
        const float delta_time = (current_time - last_time) / 1000.0f;
        last_time = current_time;

        // physics simulation loop (evaluated at fixed intervals for determinism)
        physics_time_accumulator += delta_time;
        for (int i = 0; i < MAX_SUBSTEPS && physics_time_accumulator >= FIXED_TIMESTEP; i++, physics_time_accumulator -= FIXED_TIMESTEP)
        {
            // animate elevator using MoveKinematic (before physics update)
            total_time += FIXED_TIMESTEP;
            const float t = fmod(total_time, ELEVATOR_CYCLE_TIME);
            float target_y;
            if (t < ELEVATOR_TRAVEL_TIME)
            {
                // moving up
                target_y = ELEVATOR_LOWER_Y + (ELEVATOR_UPPER_Y - ELEVATOR_LOWER_Y) * (t / ELEVATOR_TRAVEL_TIME);
            }
            else if (t < ELEVATOR_TRAVEL_TIME + ELEVATOR_PAUSE_TIME)
            {
                // pause at top
                target_y = ELEVATOR_UPPER_Y;
            }
            else if (t < 2 * ELEVATOR_TRAVEL_TIME + ELEVATOR_PAUSE_TIME)
            {
                // moving down
                const float down_t = t - (ELEVATOR_TRAVEL_TIME + ELEVATOR_PAUSE_TIME);
                target_y = ELEVATOR_UPPER_Y - (ELEVATOR_UPPER_Y - ELEVATOR_LOWER_Y) * (down_t / ELEVATOR_TRAVEL_TIME);
            }
            else
            {
                // pause at bottom
                target_y = ELEVATOR_LOWER_Y;
            }

            body_interface.MoveKinematic(elevator_id,
                                         JPH::RVec3(0.0, target_y, 0.0),
                                         JPH::Quat::sIdentity(), FIXED_TIMESTEP);

            physics_system.Update(FIXED_TIMESTEP, 1, &temp_allocator, &job_system);
        }

        // aply deferred jumps (we can't do it in the contact listener)
        jump_listener->ApplyQueuedJumps();
        jump_listener->CleanupCooldowns();

        // calculate the direction of the camera
        const float forward_x = cos(camera_yaw) * cos(camera_pitch);
        const float forward_y = sin(camera_pitch);
        const float forward_z = sin(camera_yaw) * cos(camera_pitch);

        // check keyboard state for keys that move the camera
        const Uint8 * const state = SDL_GetKeyboardState(NULL);
        {
            const float right_x = -sin(camera_yaw);
            const float right_z =  cos(camera_yaw);

            float dx = 0.0f, dy = 0.0f, dz = 0.0f;
            if (state[SDL_SCANCODE_W])
            {
                dx += forward_x * WALK_SPEED * delta_time;
                dy += forward_y * WALK_SPEED * delta_time;
                dz += forward_z * WALK_SPEED * delta_time;
            }
            if (state[SDL_SCANCODE_S])
            {
                dx -= forward_x * WALK_SPEED * delta_time;
                dy -= forward_y * WALK_SPEED * delta_time;
                dz -= forward_z * WALK_SPEED * delta_time;
            }
            if (state[SDL_SCANCODE_A])
            {
                dx -= right_x * WALK_SPEED * delta_time;
                dz -= right_z * WALK_SPEED * delta_time;
            }
            if (state[SDL_SCANCODE_D])
            {
                dx += right_x * WALK_SPEED * delta_time;
                dz += right_z * WALK_SPEED * delta_time;
            }
            if (state[SDL_SCANCODE_SPACE])
                dy += WALK_SPEED * delta_time;
            if (state[SDL_SCANCODE_LCTRL] || state[SDL_SCANCODE_RCTRL])
                dy -= WALK_SPEED * delta_time;

            // (possibly) move camera
            camera_x += dx;
            camera_y += dy;
            camera_z += dz;
        }

        // clear screen
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // projection transform
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(60.0, 800.0 / 600.0, 0.1, 1000.0);

        // camera transform
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt(camera_x, camera_y, camera_z,  // Eye position
                  camera_x + forward_x, camera_y + forward_y, camera_z + forward_z,  // Look at
                  0.0, 1.0, 0.0);  // Up

        // physics debug drawing
        JPH::BodyManager::DrawSettings draw_settings;
        draw_settings.mDrawShape = true;
        draw_settings.mDrawShapeWireframe = true; // Wireframe for debug
        draw_settings.mDrawBoundingBox = false;
        renderer->SetCameraPosition(camera_x, camera_y, camera_z);
        physics_system.DrawBodies(draw_settings, JPH::DebugRenderer::sInstance);
        // physics_system.DrawConstraints(JPH::DebugRenderer::sInstance);
        // physics_system.DrawConstraintLimits(JPH::DebugRenderer::sInstance);
        // physics_system.DrawConstraintReferenceFrame(JPH::DebugRenderer::sInstance);
        // constraint->DrawConstraintLimits(JPH::DebugRenderer::sInstance);
        

        // swap buffers
        SDL_GL_SwapWindow(window.get());
    }

    // restore mouse defaults
    SDL_SetRelativeMouseMode(SDL_FALSE);
    SDL_ShowCursor(SDL_TRUE);

    // remove all the bodies we added
    for (const JPH::BodyID &ball_id : ball_ids)
    {
        physics_system.GetBodyInterface().RemoveBody(ball_id);
        physics_system.GetBodyInterface().DestroyBody(ball_id);
    }
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

    return EXIT_SUCCESS;
}
