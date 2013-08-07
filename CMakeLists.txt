cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME point-process-experiment-core)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()


#set(CMAKE_CXX_FLAGS "-std=c++0x -pedantic -Wall -O2 -g")
#set(CMAKE_CXX_FLAGS "-std=c++0x -pedantic -Wall -O0 -g3")

add_definitions( -std=c++0x -pedantic -Wall )
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")


# The library
add_library( point-process-experiment-core SHARED
  src/simulated_data.cpp
  )
pods_install_headers(
  src/simulated_data.hpp
  DESTINATION
  point-process-experiment-core
)
pods_use_pkg_config_packages(point-process-experiment-core math-core lcmtypes_math-core probability-core lcmtypes_probability-core point-process-core )
pods_install_libraries( point-process-experiment-core )
pods_install_pkg_config_file(point-process-experiment-core
    CFLAGS
    LIBS -lpoint-process-experiment-core
    REQUIRES math-core probability-core point-process-core
    VERSION 0.0.1)


add_subdirectory( test )
