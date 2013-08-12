
#if !defined( __POINT_PROCESS_EXPERIMENT_CORE_SIMULATED_DATA_HPP__ )
#define __POINT_PROCESS_EXPERIMENT_CORE_SIMULATED_DATA_HPP__


#include <lcmtypes/p2l_math_core.hpp>
#include <lcmtypes/p2l_probability_core.hpp>
#include <probability-core/distributions.hpp>
#include <math-core/geom.hpp>

namespace point_process_experiment_core {

  
  // Description:
  // Creates simulated 1D points in a line as if there were
  // a set of gaussians with 0 mean and given sigma
  // with the number of poitns per gaussian followign the given poisson
  // distribution.
  std::vector<math_core::nd_point_t>
  simulate_line_point_clusters_gaussian_spread_poisson_size
  ( const math_core::nd_aabox_t& window,
    const size_t num_clusters,
    const double cluster_spread_gaussian_sigma,
    const double cluster_size_poisson_lambda );


  // Description:
  // Add zero mean gaussian noise to a set of points
  // this CHANGES the given poitns.
  // Ensures points stay within the given window
  void add_zero_mean_coordinate_independent_gaussian_noise
  ( std::vector<math_core::nd_point_t>& points,
    const double noise_sigma,
    const math_core::nd_aabox_t& window );
  

  // Description:
  // Add coordiante independet noise
  // with given distribution
  template<typename T_Distribution>
  void add_coordinate_independent_noise
  ( std::vector<math_core::nd_point_t>& points,
    const T_Distribution& noise_distribution,
    const math_core::nd_aabox_t& window ) 
  {
    
    // iterate over each poitn and add nosie to it
    for( size_t p_i = 0; p_i < points.size(); ++p_i ) {
      math_core::nd_point_t p;
      
      // now sample from the nosie distribution independentkly for
      // each coordinate and add. Do this until the new point is
      // still within the window
      do  {

	// set point to origin point
	p = points[p_i];

	// sample noise for each coordinate and add to p
	for( size_t c_i = 0; (long)c_i < p.n; ++c_i ) {
	  double noise_value = probability_core::sample_from( noise_distribution ).coordinate[0];
	  p.coordinate[c_i] += noise_value;
	}

	// loop if the point with noise is outside of window
      } while( math_core::is_inside( p, window ) == false );

      // store the noise point
      points[ p_i ] = p;
    }
  }

}

#endif

