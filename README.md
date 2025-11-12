# Overview

Demo game using SDL2, OpenGL 2.1, and Jolt Physics.

# Dependencies

* [CMake](https://gitlab.kitware.com/cmake/cmake) (tested with v3.25.1)
* [GCC](https://gcc.gnu.org/) (tested with v12.2.0)
* [SDL2](https://github.com/libsdl-org/SDL) (tested with v2.26.5)
* [Jolt Physics](https://jrouwe.github.io/JoltPhysics/) (tested with v5.4.0) GitHub repo: https://github.com/jrouwe/JoltPhysics

## Jolt Physics

Download the git repository:

```
git clone https://github.com/jrouwe/JoltPhysics.git
cd JoltPhysics
```

For building in "Release" mode:

```
cd Build
./cmake_linux_clang_gcc.sh Release g++ -DCMAKE_INSTALL_PREFIX=~/apps/JPH -DUSE_AVX2=OFF -DUSE_FMADD=OFF -DUSE_LZCNT=OFF -DUSE_TZCNT=OFF -DBUILD_SHARED_LIBS=ON -DDEBUG_RENDERER_IN_DISTRIBUTION=ON -DCPP_RTTI_ENABLED=ON
cd Linux_Release
cmake --build . --config Release --target install
```

For building in "Debug" mode:

```
cd Build
./cmake_linux_clang_gcc.sh Debug g++ -DCMAKE_INSTALL_PREFIX=~/apps/JPH_debug -DUSE_AVX2=OFF -DUSE_FMADD=OFF -DUSE_LZCNT=OFF -DUSE_TZCNT=OFF -DBUILD_SHARED_LIBS=ON -DDEBUG_RENDERER_IN_DISTRIBUTION=ON -DCPP_RTTI_ENABLED=ON
cd Linux_Release
cmake --build . --config Debug --target install
```

# Compiling

"Release" mode:

```
cmake .. -DCMAKE_PREFIX_PATH=~/apps/JPH -DCMAKE_BUILD_TYPE=Release
```

"Debug" mode:

```
cmake .. -DCMAKE_PREFIX_PATH=~/apps/JPH_debug -DCMAKE_BUILD_TYPE=Debug
```

# Running

```
./jolt-game-demo
```

# Controls

* mouse movement to change camera angle
* mouse left-click to shoot a ball
* <kbd>W</kbd> for walk/fly forward
* <kbd>S</kbd> for walk/fly backward
* <kbd>A</kbd> for walk/fly left
* <kbd>D</kbd> for walk/fly right
* <kbd>SPACE</kbd> for jump / vertical ascent
* <kbd>LCTRL</kbd> vertical descent
* <kbd>ESC</kbd> to quit
