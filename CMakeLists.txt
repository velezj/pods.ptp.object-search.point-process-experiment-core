cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME object-search.point-process-experiment-core)
include(cmake/pods.cmake)

add_definitions( -std=c++0x -Wall -fdiagnostics-show-option -Wno-unused-local-typedefs -fPIC )
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")


option ( USE_PEDANTIC "Turn on -pendantic mode in gcc. This will spit out *lots* of warnings from lcm :-(, but hopefully none from the rest of the code" OFF)
if( USE_PEDANTIC )
    add_definitions( -pedantic )
endif (USE_PEDANTIC)


# The library
add_library( object-search.point-process-experiment-core SHARED
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
pods_use_pkg_config_packages(object-search.point-process-experiment-core 
    object-search.common
    object-search.math-core
    object-search.probability-core
    object-search.point-process-core
    object-search.planner-core
    boost-1.54.0
    boost-1.54.0-filesystem)
pods_install_libraries( object-search.point-process-experiment-core )
pods_install_pkg_config_file(object-search.point-process-experiment-core
    CFLAGS
    LIBS -lobject-search.point-process-experiment-core
    REQUIRES object-search.common object-search.math-core object-search.probability-core object-search.point-process-core object-search.planner-core boost-1.54.0 boost-1.54.0-filesystem
    VERSION 0.0.1)


add_subdirectory( test )
