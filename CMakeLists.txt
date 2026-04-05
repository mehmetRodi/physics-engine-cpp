set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
cmake_minimum_required(VERSION 3.15)
project(PhysicsEngine VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 1. Find the package FIRST
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)
# 2. Define the executable and all its source files
# Grab all .cpp and .hpp files in the src/ directory
# CONFIGURE_DEPENDS tells CMake to re-configure if the directory contents change
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS 
    "*/*.cpp" 
    "*/*.h"
    "*/*.hpp"
)

# Add them to your executable or library
add_executable(physics ${SOURCES})
# 3. Tell the compiler where to find headers
target_include_directories(physics PRIVATE .)

# 4. LINK the libraries to the executable
target_link_libraries(physics PRIVATE sfml-graphics sfml-window sfml-system)

# 5. Compiler warnings (Fixed the 'endif' placement)
if(MSVC) 
    target_compile_options(physics PRIVATE /W4)
else() 
    target_compile_options(physics PRIVATE -Wall -Wextra -Wpedantic) 
endif() # This needs to be on its own line
