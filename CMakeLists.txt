cmake_minimum_required(VERSION 3.13)

project(jolt-game-demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(sdl2 REQUIRED)
find_package(Jolt REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_search_module(GLU glu REQUIRED)

add_compile_definitions(JPH_DEBUG_RENDERER)

add_executable(${PROJECT_NAME}
    src/PhysicsBroadphase.cpp
    src/PhysicsContactListener.cpp
    src/PhysicsDebugRenderer.cpp
    src/main.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${SDL2_LIBRARIES}
    Jolt::Jolt
    OpenGL::OpenGL
    ${GLU_LIBRARIES}
    GLEW::GLEW
)
