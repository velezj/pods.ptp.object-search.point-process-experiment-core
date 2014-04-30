
#if !defined( __P2L_POINT_PROCESS_EXPERIMENT_CORE_data_io_HPP__ )
#define __P2L_POINT_PROCESS_EXPERIMENT_CORE_data_io_HPP__


#include <lcmtypes/p2l_math_core.hpp>
#include <vector>
#include <iosfwd>

namespace point_process_experiment_core {


  // Description:
  // Given a SSV (Space Delimited Values) stream with point data,
  // returns a vector of ndpoint_t representing the point data.
  std::vector<math_core::nd_point_t> 
  parse_points_from_ssv_stream( std::istream& is, bool lisp_format=true );

}


#endif
