
add_executable( test-simulate-line-clusters-gaussian-poisson 
  test-simulate-line-clusters-gaussian-poisson.cpp )
pods_use_pkg_config_packages( test-simulate-line-clusters-gaussian-poisson
  math-core lcmtypes_math-core
  probability-core lcmtypes_probability-core
  point-process-core
  point-process-experiment-core )
pods_install_executables( test-simulate-line-clusters-gaussian-poisson )