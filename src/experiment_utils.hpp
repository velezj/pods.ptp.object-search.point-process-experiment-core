
#if !defined( __P2L_POINT_PROCESS_EXPERIMENT_CORE_data_io_HPP__ )
#define __P2L_POINT_PROCESS_EXPERIMENT_CORE_data_io_HPP__


#include <planner-core/planner.hpp>
#include <boost/optional.hpp>
#include <iosfwd>
#include <boost/shared_ptr.hpp>
#include <point-process-core/point_process.hpp>
#include <boost/function.hpp>
#include <stdexcept>
#include <boost/exception/all.hpp>


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
  ( boost::shared_ptr<planner_core::grid_planner_t>& planner,
    bool add_empty_regions,
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
  ( boost::shared_ptr<planner_core::grid_planner_t>& planner,
    bool add_empty_regions, 
    const math_core::nd_aabox_t& initial_window,
    const double& fraction_truth_to_find,
    const std::vector<math_core::nd_point_t>& ground_truth,
    std::ostream& out_meta,
    std::ostream& out_trace,
    std::ostream& out_progress,
    std::ostream& out_verbose_trace );



  // Description:
  // Exception indicating that an unknown world was asked for
  struct unknown_world_exception : public virtual std::exception,
				   public virtual boost::exception
  {
  };
  struct unknown_model_exception : public virtual std::exception,
				   public virtual boost::exception
  {
  };
  struct unknown_planner_exception : public virtual std::exception,
				     public virtual boost::exception
  {
  };
  struct id_already_used_exception : public virtual std::exception,
				     public virtual boost::exception
  {
  };




  // Description:
  // Returns the ground truth for a vicen world (by id)
  std::vector<math_core::nd_point_t>
  groundtruth_for_world( const std::string& id );

  // Description:
  // Returns the window for a world (by id)
  math_core::nd_aabox_t
  window_for_world( const std::string& id );

  // Description:
  // Returns a model by id for the given window
  boost::shared_ptr<point_process_core::mcmc_point_process_t>
  get_model_by_id( const std::string& id,		   
		   const math_core::nd_aabox_t& window,
		   const std::vector<math_core::nd_point_t>& groundtruth );

  // Description:
  // Returns a planner by id which will use hte given model
  boost::shared_ptr<planner_core::grid_planner_t>
  get_planner_by_id( const std::string& id,
		     boost::shared_ptr< point_process_core::mcmc_point_process_t>& model );


  // Description:
  // Returns a bool marked grid which is used as the action grid for the
  // given planner. Furthermore, the grid is marked with true iff that grid
  // point contains data from the given world
  point_process_core::marked_grid_t<bool>
  get_grid_for_setup( const std::string& world_id,
		      const std::string& model_id,
		      const std::string& planner_id );


  // Description:
  // Registers a world with a unique id.
  void
  register_world
  ( const std::string& id,
    const boost::function<std::vector<math_core::nd_point_t> ()>& groundtruth,
    const boost::function< math_core::nd_aabox_t () >& window );
  
  // Description:
  // Registers a model with id
  void
  register_model
  ( const std::string& id,
    const boost::function< boost::shared_ptr<point_process_core::mcmc_point_process_t> ( const math_core::nd_aabox_t&,
											 const std::vector<math_core::nd_point_t>& ) >& model );
  
  // Description:
  // Registeres a planer with id
  void
  register_planner
  ( const std::string& id,
    const boost::function< boost::shared_ptr<planner_core::grid_planner_t> (boost::shared_ptr< point_process_core::mcmc_point_process_t>&) >& planner );


  // Description:
  // Return all the registered worlds
  std::vector<std::string>
  get_registered_worlds();


  // Description:
  // Returns all registered models
  std::vector<std::string>
  get_registered_models();
  
  // Description:
  // Returns registered planners
  std::vector<std::string>
  get_registered_planners();


  // Description:
  // Clear all registered worlds,models,and planners
  void clear_all_registered_experiments();

}

#endif
