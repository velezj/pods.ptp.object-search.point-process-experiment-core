
#include "experiment_runner.hpp"
#include "experiment_utils.hpp"
#include <p2l-common/context.hpp>
#include <iostream>
#include <fstream>


using namespace math_core;
using namespace planner_core;
using namespace point_process_core;


namespace point_process_experiment_core {


  //====================================================================

  void
  run_experiment
  ( const std::string& world,
    const std::string& model,
    const std::string& planner_id,
    const bool add_empty_regions,
    const double& initial_window_fraction,
    const std::string& experiment_id ) 
  {
    // push the experiment id as a context
    p2l::common::push_context( p2l::common::context_t( experiment_id ) );
    
    // get the wanted world points and window
    std::vector< nd_point_t > ground_truth = groundtruth_for_world( world );
    nd_aabox_t world_window = window_for_world( world );
    
    // build up the point process model
    boost::shared_ptr< mcmc_point_process_t > planner_process 
      = get_model_by_id( model, world_window );
    
    // build up the planner
    boost::shared_ptr<grid_planner_t> planner 
      = get_planner_by_id( planner_id, planner_process  );
    
    // get the initial, window
    nd_aabox_t initial_window = 
      aabox( world_window.start, 
	     world_window.start + ( world_window.end - world_window.start ) * initial_window_fraction );
    
    // seed the planner
    initial_window =
      setup_planner_with_initial_observations( planner,
					       add_empty_regions,
					       initial_window,
					       ground_truth );
    
    // create the meta and trace files
    std::ofstream out_meta( p2l::common::context_filename( "planner.meta" ) );
    std::ofstream out_trace( p2l::common::context_filename( "planner.trace" ) );
    std::ofstream out_verbose_trace( p2l::common::context_filename( "planner.verbose-trace" ) );
    std::cout << "context filename are in: " << p2l::common::context_filename( "<filename>") << std::endl;
    
    // run the planner
    std::vector<marked_grid_cell_t> trace =
      simulate_run_until_all_points_found( planner,
					   add_empty_regions,
					   initial_window,
					   ground_truth,
					   out_meta,
					   out_trace,
					   std::cout,
					   out_verbose_trace);
    
  }
  

  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  //====================================================================
  


}
