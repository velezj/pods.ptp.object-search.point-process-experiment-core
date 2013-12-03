
#include "experiment_utils.hpp"
#include <iostream>
#include <algorithm>
#include <math-core/io.hpp>


using namespace math_core;
using namespace point_process_core;
using namespace planner_core;

namespace point_process_experiment_core {

  //==========================================================================

  std::vector<nd_aabox_t>
  compute_empty_regions( const std::vector<nd_point_t>& points,
			 const nd_aabox_t& region,
			 const double epsilon = 1e-7)
  {
    assert( region.start.n == 2 );
    assert( region.start.n > 0 );
    if( region.start.n != 2 ) {
      throw std::domain_error( "Canno compute empty regions, only implemented 2D case!" );
    }

    // ok, first we need to sort the points by their x coordinate
    // and another sorted by their y coordinates
    int dim = region.start.n;
    std::vector<nd_point_t> points_x( points );
    std::sort( points_x.begin(),
	       points_x.end(),
	       &point_compare_x );
    std::vector<nd_point_t> points_y( points );
    std::sort( points_y.begin(),
	       points_y.end(),
	       &point_compare_y );
  
    // Now, create the x and y lines
    std::vector< nd_point_t > x_line_ticks;
    std::vector< nd_point_t > y_line_ticks;
    x_line_ticks.push_back( region.start );
    y_line_ticks.push_back( region.start );
    for( size_t i = 0; i < points_x.size(); ++i ) {
      nd_vector_t x_margin = ( epsilon * vector( axis_direction( dim, 0 ) ) );
      x_line_ticks.push_back( points_x[i] + (-1.0 * x_margin) );
      x_line_ticks.push_back( points_x[i] + x_margin );
      nd_vector_t y_margin = ( epsilon * vector( axis_direction( dim, 1 ) ) );
      y_line_ticks.push_back( points_y[i] + (-1.0 * y_margin) );
      y_line_ticks.push_back( points_y[i] + y_margin );
    }
    x_line_ticks.push_back( region.end );
    y_line_ticks.push_back( region.end );
  
    // create all sub regions given the lines and return those which do not
    // have a point in them
    std::vector<nd_aabox_t> empty_regions;
    for( size_t xi = 0; xi < x_line_ticks.size() - 1; ++xi ) {
      nd_point_t xstart = x_line_ticks[xi];
      nd_point_t xend = x_line_ticks[xi+1];
      for( size_t yi = 0; yi < y_line_ticks.size() - 1; ++yi ) {
	nd_point_t ystart = y_line_ticks[yi];
	nd_point_t yend = y_line_ticks[yi+1];

	nd_point_t start = point( xstart.coordinate[0],
				  ystart.coordinate[1] );
	nd_point_t end = point( xend.coordinate[0],
				yend.coordinate[1] );
	nd_aabox_t box = aabox( start, end );
	bool empty = true;
	for( size_t k = 0; k < points.size(); ++k ) {
	  if( is_inside( points[k], box ) ) {
	    empty = false;
	    break;
	  }
	}
	if( empty ) {
	  empty_regions.push_back( box );
	}
      }
    }
  
    return empty_regions;
  }

  //==========================================================================

  nd_aabox_t
  setup_planner_with_initial_observations
  ( boost::shared_ptr<grid_planner_t>& planner,
    bool add_empty_regions,
    const nd_aabox_t& initial_window,
    const std::vector<nd_point_t>& ground_truth )
  {

    // grab the grid to use from hte planner (just as structure!)
    marked_grid_t<bool> grid = planner->visited_grid();

    // Grab the cell in hte planner's visited grid which are inside of the 
    // initial window
    std::vector<marked_grid_cell_t> cells 
      = cells_fully_inside_window( grid,
				   initial_window );
  
    // compute the actual window used (by hte grid resolution)
    nd_aabox_t actual_window = enclosing_window_for_cells( grid, cells );

    // grab the ground truth points inside of the window
    std::vector<nd_point_t> seen_points;
    if( !undefined(actual_window) ) {
      seen_points = points_inside_window(actual_window,
					 ground_truth);
    }

    std::cout << "-- #groundtruth: " << ground_truth.size() << std::endl;
    std::cout << "-- #seen points: " << seen_points.size() << std::endl;
    std::cout << "-- init window: " << initial_window << std::endl;
    std::cout << "-- actual window: " << actual_window << std::endl;
    std::cout << "-- #cells: " << cells.size() << std::endl;


    // Ok, since we are batch updating the planner, temporarily set the
    // update_model_mcmc_iterations to 0
    grid_planner_parameters_t old_params 
      = planner->get_grid_planner_parameters();
    grid_planner_parameters_t batch_params = old_params;
    batch_params.update_model_mcmc_iterations = 0;
    planner->set_grid_planner_parameters( batch_params );


    if( true ) {
      for( size_t i = 0; i < seen_points.size(); ++i ) {
	if( seen_points[i].n != 2 ) {
	  std::cout << "!! seen_points[" << i << "].n = " << seen_points[i].n << std::endl;
	} 
      }
      for( size_t i = 0; i < ground_truth.size(); ++i ) {
	if( ground_truth[i].n != 2 ) {
	  std::cout << "!! ground_truth[" << i << "].n = " << ground_truth[i].n << std::endl;
	} 
      }
    }

    // store the last of the seen points to add at the very end
    std::vector<nd_point_t> last_seen_point;
    last_seen_point.push_back( seen_points.back() );
    seen_points.pop_back();
  
    // now update the planner with the observations of the points
    planner->add_observations( seen_points );

    if( true ) {
      std::cout << "-- adding negative regions" << std::endl;
    }

    // now add the fully negative cell regions
    std::vector<marked_grid_cell_t> partial_cells;
    for( size_t i = 0; i < cells.size(); ++i ) {
      nd_aabox_t region = grid.region( cells[ i ] );
      if( points_inside_window( region, ground_truth ).empty() ) {
      
	// this is a fully negative cell, so add it to the planner
	planner->add_negative_observation( cells[i] );
      } else {
    
	// this is a partial cell (so some points inside)!
	// store for future processing
	partial_cells.push_back( cells[i] );
      }
    }

    if( true && add_empty_regions) {
      std::cout << "-- adding empty regions " << partial_cells.size() << std::endl;
    }

    // deal with partial cells, which have some obseved points in them.
    // We need to add the "emptyu space" around hte observed points so
    // that we get good inference
    if( add_empty_regions ) {
      for( size_t i = 0; i < partial_cells.size(); ++i ) {
	
	// grab the points in the cell's region
	nd_aabox_t region = grid.region( partial_cells[i] );
	std::vector<nd_point_t> points = points_inside_window( region,
							       ground_truth );
	
	// now compute the empty region of the cell
	std::vector<nd_aabox_t> empty_regions = compute_empty_regions( points,
								       region );
	
	// add the empty region to the planner
	for( size_t j = 0; j < empty_regions.size(); ++j ) {
	  planner->add_empty_region( empty_regions[ j ] );
	}
      }
    }

    // restore the grid planner params for non-batch use
    planner->set_grid_planner_parameters( old_params );

    // force a *single* model update sequence of mcmc steps
    // for the entire batch of new observations
    planner->add_observations( last_seen_point );


    // mark all of the inital cells as visited
    for( size_t i = 0; i < cells.size(); ++i ) {
      planner->add_visited_cell( cells[ i ] );
    }

    // setup the current position
    planner->set_current_position( actual_window.start +
				   ( 0.5 * (actual_window.end - actual_window.start) ) );

    
    // return the actual window used
    return actual_window;
  }


  //==========================================================================

  std::vector<marked_grid_cell_t>
  simulate_run_until_all_points_found
  ( boost::shared_ptr<grid_planner_t>& planner,
    bool add_empty_regions,
    const nd_aabox_t& initial_window,
    const std::vector<nd_point_t>& ground_truth,
    std::ostream& out_meta,
    std::ostream& out_trace,
    std::ostream& out_progress,
    std::ostream& out_verbose_trace )
  {

    // the iteration counter
    size_t iteration = 0;

    if( true ) {
      (out_progress) << "Starting SIMULATION: " << std::endl;
      (out_progress) << "  Init Window: " << initial_window << std::endl;
      (out_progress) << "  Init #points: " << planner->observations().size() << std::endl;
      (out_progress) << "  Total Points: " << ground_truth.size() << std::endl;
    }

    // the list of chosen cells
    std::vector<marked_grid_cell_t> chosen_cells;
    std::vector<nd_aabox_t> chosen_regions;
    std::vector< bool > chosen_region_negative;

    // run the planner while we have no found everything
    while( planner->observations().size() < ground_truth.size() ) {

      // pint out the iteration number
      out_verbose_trace << "+ITERATION+ " << iteration << std::endl;
    
      // Choose the next observation cell
      marked_grid_cell_t next_cell = 
	planner->choose_next_observation_cell();
    
      // Take any points inside the cell
      // and add as observations
      std::vector<nd_point_t> new_obs;
      nd_aabox_t region = planner->visited_grid().region( next_cell );
      std::vector<nd_point_t> obs = planner->observations();
      for( std::size_t p_i = 0;
	   p_i < ground_truth.size();
	   ++p_i ) {

	// check if point is inside region and not already part of process
	nd_point_t point = ground_truth[ p_i ];
	if( is_inside( point, region ) &&
	    std::find( obs.begin(),
		       obs.end(),
		       point ) == obs.end() ) {
	  new_obs.push_back( point );
	}
      }

      // print out hte chosne region and cell
      out_verbose_trace << "+CHOSEN-CELL+ " << next_cell << std::endl;
      out_verbose_trace << "+CHOSEN-REGION+ " << region << std::endl;

      // print out the new observations (if any)
      out_verbose_trace << "+NEW-OBSERVATIONS+ ";
      for( size_t i = 0; i < new_obs.size(); ++i ) {
	out_verbose_trace << new_obs[i] << " ";
      }
      out_verbose_trace << std::endl;

      // update chosen cells and regions
      chosen_cells.push_back( next_cell );
      chosen_regions.push_back( region );
    
      // Ok, add new observation or a negative region if no new obs
      if( new_obs.empty() ) {
	planner->add_negative_observation( next_cell );
	chosen_region_negative.push_back( true );

	// trace this
	out_verbose_trace << "+ADD-NEGATIVE-OBSERVATION+ " << next_cell << std::endl;

      } else {
      
	// now add negative regions for the places in the cell without points
	std::vector<nd_aabox_t> empty_regs = compute_empty_regions( new_obs, region );
	if( add_empty_regions ) {
	  for( size_t i = 0; i < empty_regs.size(); ++i ) {
	    planner->add_empty_region( empty_regs[i] );
	  }
	}

	// and add teh actual observations 
	// (make sure this is AFTER the empty regions)
	planner->add_observations( new_obs );
	chosen_region_negative.push_back( false );      
	
	// trace this
	if( add_empty_regions ) {
	  out_verbose_trace << "+ADD-EMPTY-REGIONS+ ";
	  for( size_t i = 0; i < empty_regs.size(); ++i ) {
	    out_verbose_trace << empty_regs[i] << " ";
	  }
	  out_verbose_trace << std::endl;
	}
	out_verbose_trace << "+ADD-OBSERVATIONs+ ";
	for( size_t i = 0; i < new_obs.size(); ++i ) {
	  out_verbose_trace << new_obs[i] << " ";
	}
	out_verbose_trace << std::endl;

      }

      // update position
      planner->set_current_position( region.start + (region.end - region.start) * 0.5 );

      // trace thjis
      out_verbose_trace << "+SET-CURRENT-POSITION+ " << ( region.start + (region.end - region.start) * 0.5 ) << std::endl;


      // add the cell as visited to the planner
      planner->add_visited_cell( next_cell );

      // trace this
      out_verbose_trace << "+ADD-VISITED-CELL+ " << next_cell << std::endl;

      // add to trace
      if( true ) {
	(out_trace) << iteration << " "
		    << next_cell << " "
		    << new_obs.size() << " "
		    << planner->observations().size() << " "
		    << region << " ";
	for( size_t i = 0; i < new_obs.size(); ++i ) {
	  (out_trace) << new_obs[ i ] << " ";
	}
	(out_trace) << std::endl;
	(out_trace).flush();
      }

    
      // print the process model parameters to user
      if( true ) {

	// TODO: get an interface for printing out hte inner model state
	//(*out_progress) << "[" << iteration << "]   " << std::endl;
	//(*out_progress) << process->_state << std::endl;
      
	// print status to user
	(out_progress) << "[" << iteration << "]   "
		       <<  "cell: " << next_cell 
		       << "  { #new= " << new_obs.size() << " total: " << planner->observations().size() << " }" 
	  // << " entropy: " << entropy
		       << std::endl;
	(out_progress) << std::flush;
      }

      // trace the planner information (including the model parameters!)
      out_verbose_trace << "+PLANNER+ ";
      planner->print_shallow_trace( out_verbose_trace );
      out_verbose_trace << std::endl;
      out_verbose_trace << "+MODEL+ ";
      planner->print_model_shallow_trace( out_verbose_trace );
      out_verbose_trace << std::endl;
      
      
      // icrease iteration count
      ++iteration;
    }
  
    // return the chosen cells
    return chosen_cells;
  
  }



  //==========================================================================

  struct world_func_pair_t
  {
    boost::function< std::vector<math_core::nd_point_t> () > groundtruth;
    boost::function< math_core::nd_aabox_t () > window;
  };
  std::map< std::string, world_func_pair_t > _g_worlds;

  void
  register_world
  ( const std::string& id,
    const boost::function<std::vector<math_core::nd_point_t> ()>& groundtruth,
    const boost::function< math_core::nd_aabox_t () >& window )
  {
    if( _g_worlds.find( id ) != _g_worlds.end() ) {
      BOOST_THROW_EXCEPTION( id_already_used_exception() );
    }

    world_func_pair_t g;
    g.groundtruth = groundtruth;
    g.window = window;
    _g_worlds[ id ] = g;
  }


  //==========================================================================

  std::map< std::string, boost::function<  boost::shared_ptr<mcmc_point_process_t> (const math_core::nd_aabox_t&) > > _g_models;

  void
  register_model
  ( const std::string& id,
    const boost::function< boost::shared_ptr<mcmc_point_process_t> (const math_core::nd_aabox_t& ) >& model )
  {
    if( _g_models.find( id ) != _g_models.end() ) {
      BOOST_THROW_EXCEPTION( id_already_used_exception() );
    }

    _g_models[ id ] = model;
  }


  //==========================================================================

  std::map< std::string, boost::function< boost::shared_ptr<grid_planner_t> (boost::shared_ptr<point_process_core::mcmc_point_process_t>&) > > _g_planners;

  void
  register_planner
  ( const std::string& id,
    const boost::function< boost::shared_ptr<grid_planner_t> (boost::shared_ptr<point_process_core::mcmc_point_process_t>&) >& planner )
  {
    if( _g_planners.find( id ) != _g_planners.end() ) {
      BOOST_THROW_EXCEPTION( id_already_used_exception() );
    }

    _g_planners[ id ] = planner;
  }


  //==========================================================================
  
  std::vector<math_core::nd_point_t>
  groundtruth_for_world( const std::string& id )
  {
    if( _g_worlds.find( id ) == _g_worlds.end() ) {
      BOOST_THROW_EXCEPTION( unknown_world_exception() );
    }
    return _g_worlds[ id ].groundtruth();
  }

  //==========================================================================

  math_core::nd_aabox_t
  window_for_world( const std::string& id )
  {
    if( _g_worlds.find( id ) == _g_worlds.end() ) {
      BOOST_THROW_EXCEPTION( unknown_world_exception() );
    }
    return _g_worlds[ id ].window();
  }

  //==========================================================================

  boost::shared_ptr<point_process_core::mcmc_point_process_t>
  get_model_by_id( const std::string& id,
		   const math_core::nd_aabox_t& window )
  {
    if( _g_models.find( id ) == _g_models.end() ) {
      BOOST_THROW_EXCEPTION( unknown_model_exception() );
    }
    return _g_models[ id ]( window );
  }

  //==========================================================================

  boost::shared_ptr<planner_core::grid_planner_t>
  get_planner_by_id( const std::string& id,
		     boost::shared_ptr<point_process_core::mcmc_point_process_t>& model )
  {
    if( _g_planners.find( id ) == _g_planners.end() ) {
      BOOST_THROW_EXCEPTION( unknown_planner_exception() );
    }
    return _g_planners[ id ]( model );
  }
  
  
  //==========================================================================

  template< typename TK, typename TV >
  std::vector<TK>
  keys( const std::map<TK,TV>& m ) {
    std::vector<TK> k;
    for( auto item : m ) {
      k.push_back( item.first );
    }
    return k;
  }

  //==========================================================================

  std::vector<std::string>
  get_registered_worlds()
  {
    return keys( _g_worlds );
  }
  
  //==========================================================================
  
  std::vector<std::string>
  get_registered_models()
  {
    return keys( _g_models );
  }
  
  //==========================================================================

  std::vector<std::string>
  get_registered_planners()
  {
    return keys( _g_planners );
  }

  //==========================================================================

  void
  clear_all_registered_experiments()
  {
    _g_worlds.clear();
    _g_models.clear();
    _g_planners.clear();
  }

  //==========================================================================
  //==========================================================================
  //==========================================================================
  //==========================================================================
  //==========================================================================
  //==========================================================================
  //==========================================================================
  //==========================================================================
  //==========================================================================
  //==========================================================================
  //==========================================================================
  //==========================================================================
  //==========================================================================
  //==========================================================================
  //==========================================================================
  //==========================================================================
  //==========================================================================
  //==========================================================================
  //==========================================================================

}
