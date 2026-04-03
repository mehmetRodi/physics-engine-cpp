# 1. Define the minimum version of CMake required to run this script
cmake_minimum_required(VERSION 3.15)

# 2. Define the project name, version, and primary language (CXX = C++)
project(PhysicsEngine VERSION 0.1.0 LANGUAGES CXX)

# 3. Set the C++ standard to C++20 and make it mandatory
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # Ensures we use standard C++ without compiler-specific bloat

# 4. Create the executable
# 'physics' is the name of the output file. 
# For now, we only list 'main.cpp'. As you add files, you'll list them here.
add_executable(physics 
  main.cpp 
  math/Vec3.cpp
)

# Look for SFML on the system. 
# We need the graphics, window, and system modules.
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

# 5. Tell the compiler where to look for header files
# 'PRIVATE' means these paths are used to build 'physics' but not passed to things linking to it.
# '.' refers to the current directory, allowing you to #include "math/vec3.hpp"
target_include_directories(physics PRIVATE .)

# 6. (Optional for now) Add compiler warnings
# This is a "pro-move" to catch bugs early.
if(MSVC)
    target_compile_options(physics PRIVATE /W4) # Windows (Visual Studio)
else()
    target_compile_options(physics PRIVATE -Wall -Wextra -Wpedantic) # Linux/macOS (Clang/GCC)
endif()

# Links the SFML libraries to our physics executable
target_link_libraries(physics PRIVATE sfml-graphics sfml-window sfml-system)
