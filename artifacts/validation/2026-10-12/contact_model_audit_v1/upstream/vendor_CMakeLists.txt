cmake_minimum_required(VERSION 3.10)
project(gz_dartsim_vendor)

# Project-specific settings
set(LIB_VER_MAJOR 6)
set(LIB_VER_MINOR 13)
set(LIB_VER_PATCH 2)

# Derived variables
set(LIB_VER ${LIB_VER_MAJOR}.${LIB_VER_MINOR}.${LIB_VER_PATCH})

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

# Set the VERSION_MATCH to "EXACT" by default, but relax the requirement
# if we are users are building from source (determined by the
# GZ_BUILD_FROM_SOURCE environment variable) or if explicitly told to do so
# by the GZ_RELAX_VERSION_MATCH environment variable.
set(VERSION_MATCH "EXACT")
if(NOT $ENV{GZ_BUILD_FROM_SOURCE} STREQUAL "")
  set(VERSION_MATCH "")
endif()

if(NOT $ENV{GZ_RELAX_VERSION_MATCH} STREQUAL "")
  set(VERSION_MATCH "")
endif()

find_package(DART ${VERSION_MATCH} ${LIB_VER_MAJOR}.${LIB_VER_MINOR}
  CONFIG
  COMPONENTS
  collision-bullet
  collision-ode
  utils
  utils-urdf
  QUIET
)

ament_vendor(${PROJECT_NAME}
  SATISFIED ${DART_FOUND}
  VCS_URL https://github.com/dartsim/dart.git
  VCS_VERSION v${LIB_VER}
  CMAKE_ARGS
    -DDART_BUILD_GUI_OSG:BOOL=FALSE
    -DDART_BUILD_DARTPY:BOOL=FALSE
    -DDART_BUILD_EXTRAS:BOOL=FALSE
  GLOBAL_HOOK
  PATCHES patches
)

message(VERBOSE "DART_FOUND: ${DART_FOUND}")
message(VERBOSE "DART_VERSION: ${DART_VERSION}")
message(VERBOSE "DART_INCLUDE_DIRS: ${DART_INCLUDE_DIRS}")
message(VERBOSE "DART_LIBRARIES: ${DART_LIBRARIES}")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
