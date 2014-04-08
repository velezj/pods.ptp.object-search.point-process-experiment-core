cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME p2l-point-process-experiment-core)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()


#set(CMAKE_CXX_FLAGS "-std=c++0x -pedantic -Wall -O2 -g")
#set(CMAKE_CXX_FLAGS "-std=c++0x -pedantic -Wall -O0 -g3")

add_definitions( -std=c++0x -Wall -fdiagnostics-show-option -Wno-unused-local-typedefs -fPIC )
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")


option ( USE_PEDANTIC "Turn on -pendantic mode in gcc. This will spit out *lots* of warnings from lcm :-(, but hopefully none from the rest of the code" OFF)
if( USE_PEDANTIC )
    add_definitions( -pedantic )
endif (USE_PEDANTIC)


# The library
add_library( p2l-point-process-experiment-core SHARED
  src/simulated_data.cpp
  src/data_io.cpp
  src/experiment_utils.cpp
  src/experiment_runner.cpp
  )
pods_install_headers(
  src/simulated_data.hpp
  src/data_io.hpp
  src/experiment_utils.hpp
  src/experiment_runner.hpp
  DESTINATION
  point-process-experiment-core
)
pods_use_pkg_config_packages(p2l-point-process-experiment-core 
    p2l-common
    p2l-math-core lcmtypes_p2l-math-core 
    p2l-probability-core lcmtypes_p2l-probability-core 
    p2l-point-process-core
    p2l-planner-core
    boost-1.54.0
    boost-1.54.0-filesystem)
pods_install_libraries( p2l-point-process-experiment-core )
pods_install_pkg_config_file(p2l-point-process-experiment-core
    CFLAGS
    LIBS -lp2l-point-process-experiment-core
    REQUIRES p2l-common p2l-math-core p2l-probability-core p2l-point-process-core p2l-planner-core boost-1.54.0 boost-1.54.0-filesystem
    VERSION 0.0.1)


add_subdirectory( test )
