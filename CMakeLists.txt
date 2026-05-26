cmake_minimum_required(VERSION 3.15)
project(PhysicsEngine VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
    "math/*.cpp"
    "physics/*.cpp"
    "collision/*.cpp"
)

add_library(physics_core ${SOURCES})
target_include_directories(physics_core PUBLIC .)

if(MSVC)
  target_compile_options(physics_core PRIVATE /W4)
else()
  target_compile_options(physics_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

option(BUILD_DEMO "Build the SFML debug/demo executable" ON)

if(BUILD_DEMO)
  if(APPLE)
    foreach(SFML_PREFIX "/opt/homebrew/opt/sfml@2" "/usr/local/opt/sfml@2")
      if(EXISTS "${SFML_PREFIX}")
        list(PREPEND CMAKE_PREFIX_PATH "${SFML_PREFIX}")
      endif()
    endforeach()
  endif()

  find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

  add_executable(physics main.cpp)
  target_link_libraries(physics PRIVATE physics_core sfml-graphics sfml-window sfml-system)
endif()

option(BUILD_BENCHMARKS "Build benchmark executables" OFF)

if(BUILD_BENCHMARKS)
  add_executable(world_step_bench bench/world_step_bench.cpp)
  target_link_libraries(world_step_bench PRIVATE physics_core)

  add_executable(vec3_bench bench/vec3_bench.cpp)
  target_link_libraries(vec3_bench PRIVATE physics_core)

  add_executable(sphere_pair_bench bench/sphere_pair_bench.cpp)
  target_link_libraries(sphere_pair_bench PRIVATE physics_core)

  if(MSVC)
    target_compile_options(world_step_bench PRIVATE /W4)
    target_compile_options(vec3_bench PRIVATE /W4)
    target_compile_options(sphere_pair_bench PRIVATE /W4)
  else()
    target_compile_options(world_step_bench PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(vec3_bench PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(sphere_pair_bench PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endif()


include(CTest)

if(BUILD_TESTING)
  include(FetchContent)

  FetchContent_Declare(
      googletest
      URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
      DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )

  FetchContent_MakeAvailable(googletest)

  file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS
      "tests/*.cpp"
    )

  add_executable(physics_tests ${TEST_SOURCES})

  target_link_libraries(physics_tests PRIVATE physics_core GTest::gtest_main)

  include(GoogleTest)
  gtest_discover_tests(physics_tests)
endif()
