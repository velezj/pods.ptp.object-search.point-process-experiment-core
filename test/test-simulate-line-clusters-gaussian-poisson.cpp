
#include <point-process-experiment-core/simulated_data.hpp>
#include <math-core/io.hpp>
#include <probability-core/distribution_utils.hpp>
#include <iostream>

using namespace math_core;
using namespace probability_core;
using namespace point_process_experiment_core;

int main( int argc, char** argv )
{

  // craete the window (the range)
  nd_aabox_t window;
  window.n = 1;
  window.start = point( 0 );
  window.end = point( 100 );

  // simulate some points
  std::vector<nd_point_t> points 
    = simulate_line_point_clusters_gaussian_spread_poisson_size
    ( window,
      4,
      5.0,
      5.0 );
  
  // print out the poitns
  for( size_t i = 0; i < points.size(); ++i ) {
    std::cout << "p[" << i << "]: " << points[i] << std::endl;
  }

  return 0;
}
