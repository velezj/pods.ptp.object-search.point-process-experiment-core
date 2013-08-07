
#include "simulated_data.hpp"
#include <math-core/matrix.hpp>


using namespace math_core;
using namespace probability_core;

namespace point_process_experiment_core {


  //=========================================================================

  std::vector<math_core::nd_point_t>
  simulate_line_point_clusters_gaussian_spread_poisson_size
  ( const math_core::nd_aabox_t& window,
    const size_t num_clusters,
    const double cluster_spread_gaussian_sigma,
    const double cluster_size_poisson_lambda )
  {

    // the resultign points
    std::vector<nd_point_t> points;

    // create the cluster and num point distributions
    gaussian_distribution_t spread_distribution;
    spread_distribution.dimension = 1;
    spread_distribution.means.push_back( 0.0 );
    spread_distribution.covariance = to_dense_mat( Eigen::MatrixXd::Identity(1,1) * ( cluster_spread_gaussian_sigma * cluster_spread_gaussian_sigma ) );
    poisson_distribution_t num_points_distribution;
    num_points_distribution.lambda = cluster_size_poisson_lambda;
    
    // spread out the clsuter means evently
    double step_x = length( window, 0 ) / (num_clusters + 1);
    double cluster_x = 0.5 * step_x;
    
    // now simulate poitns from each clsuter
    for( size_t c_i = 0; c_i < num_clusters; ++c_i, cluster_x += step_x ) {
     
      // get number of points
      unsigned int num_points = sample_from( num_points_distribution );
      
      // make the number at ealst 1
      if( num_points < 1 )
	num_points = 1;
      
      // sample point locations from spread
      for( size_t s_i = 0; s_i < num_points; ++s_i ) {
	nd_point_t sx = sample_from( spread_distribution );
	nd_point_t p = point(cluster_x) + ( sx - zero_point(sx.n) );
	if( is_inside( p, window ) ) {
	  points.push_back( p );
	}
      }
    }
      

    return points;
  }


  //=========================================================================

  void add_zero_mean_coordinate_independent_gaussian_noise
  ( std::vector<math_core::nd_point_t>& points,
    const double noise_sigma,
    const math_core::nd_aabox_t& window )
  {
    // create a zero mean 1D gaussian for the noise
    gaussian_distribution_t noise;
    noise.dimension = 1;
    noise.means = std::vector<double>( noise.dimension, 0.0 );
    noise.covariance = to_dense_mat( Eigen::MatrixXd::Identity(1,1) * ( noise_sigma  * noise_sigma ) );
    add_coordinate_independent_noise( points, noise, window );
  }

  //=========================================================================
  //=========================================================================
  //=========================================================================
  //=========================================================================
  //=========================================================================
  //=========================================================================
  //=========================================================================
  //=========================================================================
  //=========================================================================
  //=========================================================================
  //=========================================================================
  //=========================================================================
  //=========================================================================
  //=========================================================================
  //=========================================================================
  //=========================================================================
  //=========================================================================
  //=========================================================================
  //=========================================================================
  //=========================================================================
  //=========================================================================
  //=========================================================================
  //=========================================================================
  //=========================================================================
  //=========================================================================
  //=========================================================================
  //=========================================================================
  //=========================================================================
  //=========================================================================
  //=========================================================================
  //=========================================================================



}
