
#if !defined(  __POINT_PROCESS_EXPERIMENT_CORE_SIMULATED_DATA_HPP__ )
#define __POINT_PROCESS_EXPERIMENT_CORE_SIMULATED_DATA_HPP__

#include <vector>
#include <lcmtypes/p2l_math_core.hpp>

namespace point_process_experiment_core {
  

  //------------------------------------------------------------------------

  
  // Description:
  // Given a set of points in 2D and a large region,
  // Returns the set of regions which do not have any of the points.
  // These regions are subregions of hte given region, and have a
  // margin of the given epsilon around the points
  std::vector<math_core::nd_aabox_t>
  compute_empty_regions( const std::vector<math_core::nd_point_t>& points,
			 const math_core::nd_aabox_t& region,
			 const double epsilon = 1e-7);

  //------------------------------------------------------------------------


#endif

