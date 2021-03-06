
#include "data_io.hpp"
#include <iostream>
#include <math-core/geom.hpp>
#include <string>
#include <sstream>

#define VERBOSE false

using namespace math_core;


namespace point_process_experiment_core {


  //===========================================================================

  std::vector<nd_point_t> 
  parse_points_from_ssv_stream( std::istream& is, bool lisp_format )
  {

    std::vector<nd_point_t> points;
  
    // We assume that there is exactly one point per line (or empty lines)
    std::string line;
    while( std::getline( is, line ) ) {

      // read the data s a vector of doubles
      std::vector<double> data;
      double temp;
      std::string stemp;
      std::istringstream line_stream( line );
      while( line_stream >> stemp ) {
	std::istringstream iss;
	if( lisp_format ) {
	  iss.str( stemp.substr(0,stemp.size() - 2) );
	} else {
	  iss.str( stemp );
	}
	if( iss >> temp && data.size() < 2 ) {
	  data.push_back( temp );
	}
      }

      // add a new point structure from the data vector
      points.push_back( point( data ) );

      if( VERBOSE ) {
	std::cout << "..parsed " << data.size() << "-dim point" << std::endl;
      }
    }

    return points;
  }

  //===========================================================================
  //===========================================================================
  //===========================================================================
  //===========================================================================
  //===========================================================================
  //===========================================================================
  //===========================================================================
  //===========================================================================
  //===========================================================================
  //===========================================================================
  //===========================================================================
  //===========================================================================
  //===========================================================================
  //===========================================================================
}
