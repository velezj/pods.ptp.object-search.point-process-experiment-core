
#include <point-process-experiment-core/experiment_runner.hpp>
#include <boost/program_options.hpp>
#include <iostream>
#include <sstream>
#include <ctime>


int main( int argc, char** argv )
{

  // get the possible worlds, models, and planners
  std::ostringstream worlds_ss;
  std::string default_world;
  for( auto id : get_registered_worlds() ) {
    worlds_ss << id << ",";
    default_world = id;
  }
  std::ostringstream models_ss;
  std::string default_model;
  for( auto id : get_registered_models() ) {
    models_ss << id << ",";
    default_model = id;
  }
  std::ostringstream planers_ss;
  std::string default_planner;
  for( auto id : get_registered_planners() ) {
    planers_ss << id << ",";
    default_planner = id;
  }
  
  // setup the program options
  po::options_description po_desc( "Experiment Runner Options" );
  po_desc.add_options()
    ( "help", "usage and help message")
    ( "world", 
      po::value<std::string>()->default_value( default_world ), 
      "Which world to load. Current supported worlds are: " + worlds_ss.str() )
    ( "model",
      po::value<std::string>()->default_value( default_model ),
      "Which model to load. Current supported models are: " + models_ss.str() )
    ( "planner",
      po::value<std::string>()->default_value( default_planner ),
      "Which planner should we use. Current supported planners are: " + planers_ss.str() )
    ( "add-empty-regions",
      po::value<bool>()->default_value(true),
      "Compute and add emty regions within observed cells")
    ( "initial-window-fraction",
      po::value<double>()->default_value( 0.1 ),
      "The fraction of the world window initially 'seen' by the planner")
    ( "experiment-id",
      po::value<std::string>(),
      "The experiment ID. By default this will be a timestamp" );
      
  
  // parse the program options
  po::variables_map po_vm;
  po::store( po::parse_command_line( argn, argv, po_desc ), po_vm );
  po::notify( po_vm );
  
  // show usage if wanted
  if( po_vm.count( "help" )) {
    std::cout << po_desc << std::endl;
    return 1;
  }

  // generate the experiment id if watend (not given)
  std::string experiment_id;
  if( !po_vm.count( "experiment-id" ) ) {
    std::ostringstream oss;
    oss << "ex-" << time();
    experiment_id = oss.str();
  } else {
    experiment_id = po_vm[ "experiment-id" ].as<std::string>();
  }

  // ok, now simply run hte wanted experiment
  point_process_experiment_core::run_experiment
    ( po_vm["world"].as<std::string>(),
      po_vm["model"].as<std::string>(),
      po_vm["planner"].as<std::string>(),
      po_vm["add-empty-regions"].as<bool>(),
      po_vm["initial-window-fraction"].as<double>(),
      experiment_id );

}
