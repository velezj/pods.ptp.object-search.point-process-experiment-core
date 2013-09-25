
#include "geometry.hpp"
#include <math_core/geom.hpp>
#include <stdexcept>
#include <algorithm>



using namespace math_core;
  

namespace point_process_experiment_core {



  //=========================================================================

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

  
  //=========================================================================
  

}
