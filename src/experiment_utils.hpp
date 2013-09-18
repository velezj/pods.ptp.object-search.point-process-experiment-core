
#if !defined( __P2L_POINT_PROCESS_EXPERIMENT_CORE_data_io_HPP__ )
#define __P2L_POINT_PROCESS_EXPERIMENT_CORE_data_io_HPP__


#include <planner-core/planner.hpp>
#include <boost/optional.hpp>
#include <iosfwd>


namespace point_process_experiment_core {


  // Description:
  // Setsup a planner with an initial window of "known" data.
  // This takes the given ground truth and adds simulated observations 
  // to the planner with hte grounnd truth data for the given known section
  // of the world.
  //
  // This returns the actual initial window (since this will be aliased 
  // to the marked grid used by the planner! )
  math_core::nd_aabox_t
  setup_planner_with_initial_observations
  ( planner_core::grid_planner_t& planner,
    const math_core::nd_aabox_t& initial_window,
    const std::vector<math_core::nd_point_t>& ground_truth );


  // Description:
  // Runs a planner forward for a simulation experiment.
  // We are given the ground truth, and this will run the planner until
  // all of the ground truth points have been found.
  // We are given the initial window to seed the planner with and the
  // entire ground truth data of points to find.
  //
  // Returns the decision trace of observed grid cells
  std::vector<point_process_core::marked_grid_cell_t>
  simulate_run_until_all_points_found
  ( planner_core::grid_planner_t& planner,
    const math_core::nd_aabox_t& initial_window,
    const std::vector<math_core::nd_point_t>& ground_truth,
    boost::optional<std::ostream>& out_meta,
    boost::optional<std::ostream>& out_trace,
    boost::optional<std::ostream>& out_progress );

}

#endif
