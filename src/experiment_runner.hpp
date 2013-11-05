
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
    const std::string& experiment_id );


}

#endif

