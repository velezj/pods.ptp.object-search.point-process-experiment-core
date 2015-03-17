
add_executable( test-simulate-line-clusters-gaussian-poisson 
  test-simulate-line-clusters-gaussian-poisson.cpp )
pods_use_pkg_config_packages( test-simulate-line-clusters-gaussian-poisson
  object-search.math-core
  object-search.probability-core
  object-search.point-process-core
  object-search.point-process-experiment-core )
pods_install_executables( test-simulate-line-clusters-gaussian-poisson )
