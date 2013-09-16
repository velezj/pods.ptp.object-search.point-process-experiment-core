
#include "experiment_utils.hpp"
#include <iostream>
#include <algorithm>


using namespace math_core;
using namespace point_process_core;
using namespace point_process_experiment_core;
using namespace planner_core;


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
  ( grid_planner_t& planner,
    const nd_aabox_t& initial_window,
    const std::vector<nd_point_t>& ground_truth )
{

  // grab the grid to use from hte planner (just as structure!)
  marked_grid_t<bool> grid = planner.visited_grid();

  // Grab the cell in hte planner's visited grid which are inside of the 
  // initial window
  std::vector<marked_grid_cell_t> cells 
    = cells_fully_inside_window( grid,
				 initial_window );
  
  // compute the actual window used (by hte grid resolution)
  nd_aabox_t actual_window = enclosing_window_for_cells( grid, cells );

  // grab the ground truth points inside of the window
  std::vector<nd_point_t> seen_points = points_inside_window( ground_truth,
							      actual_window);
  
  // now update the planner with the observations of the points
  for( size_t i = 0 ; i < seen_points.size(); ++i ) {
    planner._observations.push_back( seen_points[i] );
  }

  // now add the fully negative cell regions
  std::vector<marked_grid_cell_t> partial_cells;
  for( size_t i = 0; i < cells.size(); ++i ) {
    nd_aabox_t region = grid.region( cells[ i ] );
    if( points_inside_window( ground_truth, region ).empty() ) {
      
      // this is a fully negative cell, so add it to the planner
      planner.add_negative_observation( cells[i] );
    } else {
    
      // this is a partial cell (so some points inside)!
      // store for future processing
      partial_cells.push_back( cells[i] );
    }
  }

  // deal with partial cells, which have some obseved points in them.
  // We need to add the "emptyu space" around hte observed points so
  // that we get good inference
  for( size_t i = 0; i < partial_cells.size(); ++i ) {
    
    // grab the points in the cell's region
    nd_aabox_t region = grid.region( partial_cells[i] );
    std::vector<nd_point_t> points = points_inside_window( ground_truth,
							   region );
    
    // now compute the empty region of the cell
    std::vector<nd_aabox_t> empty_regions = compute_empty_regions( points,
								   region );
    
    // add the empty region to the planner
    for( size_t j = 0; j < empty_regions.size(); ++j ) {
      planner.add_empty_region( empty_regions[ j ] );
    }
  }

  // mark all of the inital cells as visited
  for( size_t i = 0; i < cells.size(); ++i ) {
    planner.add_visited_cell( cells[ i ] );
  }

  // return the actual window used
  return actual_window;
}


//==========================================================================

std::vector<marked_grid_cell_t>
simulate_run_until_all_points_found
( grid_planner_t& planner,
  const nd_aabox_t& initial_window,
  const std::vector<nd_point_t>& ground_truth,
  boost::optional<std::ostream>& out_meta,
  boost::optional<std::ostream>& out_trace,
  boost::optional<std::ostream>& out_progress )
{

  // the iteration counter
  size_t iteration = 0;

  // the list of chosen cells
  std::vector<marked_grid_cell_t> chosen_cells;
  std::vector<nd_aabox_t> chosen_regions;
  std::vector< bool > chosen_region_negative;

  // run the planner while we have no found everything
  while( planner.observations().size() < ground_truth.size() ) {
    
    // Choose the next observation cell
    marked_grid_cell_t next_cell = 
      planner.choose_next_observation_cell();
    
    // Take any points inside the cell
    // and add as observations
    std::vector<nd_point_t> new_obs;
    nd_aabox_t region = planner.visisted_grid().region( next_cell );
    std::vector<nd_point_t> obs = planner.observations();
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

    // update chosen cells and regions
    chosen_cells.push_back( next_cell );
    chosen_regions.push_back( region );
    
    // Ok, add new observation or a negative region if no new obs
    if( new_obs.empty() ) {
      planner.add_negative_observation( next_cell );
      chosen_region_negative.push_back( true );
    } else {
      
      // now add negative regions for the places in the cell without points
      std::vector<nd_aabox_t> empty_regs = compute_empty_regions( new_obs, region );
      for( size_t i = 0; i < empty_regs.size(); ++i ) {
	planner.add_empty_region( empty_regs[i] );
      }

      // and add teh actual observations 
      // (make sure this is AFTER the empty regions)
      planner.add_observations( new_obs );
      chosen_region_negative.push_back( false );      
    }

    // update position
    planner.set_current_position( region.start + (region.end - region.start) * 0.5 );


    // add the cell as visited to the planner
    planner.add_visited_cell( next_cell );

    // add to trace
    if( out_trace ) {
      (*out_trace) << iteration << " "
		   << next_cell << " "
		   << new_obs.size() << " "
		   << planner.observations().size() << " "
		   << region << " ";
      for( size_t i = 0; i < new_obs.size(); ++i ) {
	(*out_trace) << new_obs[ i ] << " ";
      }
      (*out_trace) << std::endl;
      (*out_trace).flush();
    }

    
    // print the process model parameters to user
    if( out_progress ) {
      (*out_progress) << "[" << iteration << "]   " << std::endl;
      (*out_progress) << process->_state << std::endl;
      
      // print status to user
      (*out_progress) << "[" << iteration << "]   "
		      <<  "cell: " << next_cell 
		      << "  { #new= " << new_obs.size() << " total: " << planner.observations().size() << " }" 
	// << " entropy: " << entropy
		      << std::endl;
      (*out_progress) << std::flush;
    }
      
    // icrease iteration count
    ++iteration;
  }

  // return the chosen cells
  return chosen_cells;

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
//==========================================================================
//==========================================================================
//==========================================================================
//==========================================================================
//==========================================================================
//==========================================================================
//==========================================================================