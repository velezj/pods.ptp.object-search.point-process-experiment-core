
add_executable( test-simulate-line-clusters-gaussian-poisson 
  test-simulate-line-clusters-gaussian-poisson.cpp )
pods_use_pkg_config_packages( test-simulate-line-clusters-gaussian-poisson
  p2l-math-core
  p2l-probability-core
  p2l-point-process-core
  p2l-point-process-experiment-core )
pods_install_executables( test-simulate-line-clusters-gaussian-poisson )
