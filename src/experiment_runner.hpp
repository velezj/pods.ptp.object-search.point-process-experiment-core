
#if !defined( __P2L_POINT_PROCESS_EXPERIMENT_CORE_experiment_runner_HPP__ )
#define __P2L_POINT_PROCESS_EXPERIMENT_CORE_experiment_runner_HPP__

#include <string>

namespace point_process_experiment_core {


  // Description:
  // Run an experiment.
  // With a given world,planner,and model.
  void
  run_experiment
  ( const std::string& world,
    const std::string& model,
    const std::string& planner,
    const bool add_empty_regions,
    const double& initial_window_fraction,
    const bool initial_window_is_centered,
    const double& fraction_truth_to_find,
    const std::string& experiment_id );

  //=======================================================================

  // Description:
  // Runs the given planner on a model and world.
  // We then replay different orderings of the found
  // locations, and compute the entropy of the system after each
  // observation
  void
  run_permutation_entropy_trace
  ( const std::string& world,
    const std::string& model,
    const std::string& planner,
    const std::string& experiment_id );



}

#endif

