/// @file
///
/// This demo sets up a single quadruped leg that uses 
/// an open loop controller that applies a step input 
/// for 1 second

#include <memory>

#include <gflags/gflags.h>

#include <drake/systems/controllers/pid_controlled_system.h>
#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
// #include "drake/common/text_logging_gflags.h"
// #include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

#include "drake/common/find_resource.h"
// #include "drake/common/is_approx_equal_abstol.h"
// #include "drake/examples/quadrotor/quadrotor_geometry.h"
// #include "drake/examples/quadrotor/quadrotor_plant.h"
#include "drake/geometry/drake_visualizer.h"
// #include "drake/lcm/drake_lcm.h"
// #include "drake/systems/analysis/simulator.h"
// #include "drake/systems/framework/diagram.h"
// #include "drake/systems/framework/diagram_builder.h"

// DEFINE_int32(simulation_trials, 10, "Number of trials to simulate.");
DEFINE_double(simulation_real_time_rate, 1.0, "Real time rate");
DEFINE_double(simulation_time, 7.0, "How long to simulate the system for");
DEFINE_double(max_time_step, 1.0e-4, "Simulation timstep used for integrator.");
DEFINE_double(Kp_, 100.0, "Kp");
DEFINE_double(Ki_, 0.0, "Ki");
DEFINE_double(Kd_, 0.0, "Kd");


namespace drake {
// using systems::DiagramBuilder;
// using systems::Simulator;
// using systems::Context;
// using systems::ContinuousState;
// using systems::VectorBase;

namespace examples {
namespace quadleg {
namespace {

static const char* const kQuadlegURDFPath = 
  "../models/urdf/leg.urdf"; // <--relative_path
  // "/home/mrako/Documents/EMBIR/cpp_sim_intro/models/urdf/leg.urdf"; <--global_path

int do_main() {
  // lcm::DrakeLcm lcm;
  DRAKE_DEMAND(FLAGS_simulation_time > 0);

  systems::DiagramBuilder<double> builder;

  geometry::SceneGraph<double>& scene_graph = 
    *builder.AddSystem<geometry::SceneGraph>();
  scene_graph.set_name("scene_graph");

  //Load and parse the leg URDF file for the MultibodyPlant
  multibody::MultibodyPlant<double>* quadleg = 
    builder.AddSystem<multibody::MultibodyPlant<double>>(FLAGS_max_time_step);
  quadleg->set_name("plant");
  quadleg->RegisterAsSourceForSceneGraph(&scene_graph);

  multibody::Parser parser(quadleg);
  // const std::string urdf_path = FindResourceOrThrow(kQuadlegURDFPath);
  const std::string urdf_path = kQuadlegURDFPath;
  multibody::ModelInstanceIndex plant_model_instance_index = 
    parser.AddModelFromFile(urdf_path);
  (void)plant_model_instance_index;

  // quadleg->AddJointActuator("a1", quadleg->GetJointByName("Actuator1")); //Use more descriptive names for actual robot
  // quadleg->AddJointActuator("a2", quadleg->GetJointByName("Actuator2")); //Use more descriptive names for actual robot

  // Now the MultibodyPlant is complete
  quadleg->Finalize();

  // Create the Controller (TODO: decide if InverseDynamicsController or InverseDynamics w/ xtra computation after)
  const Eigen::VectorXd Kp = 
    Eigen::VectorXd::Ones(quadleg->num_actuators()) * FLAGS_Kp_;
  const Eigen::VectorXd Ki = 
    Eigen::VectorXd::Ones(quadleg->num_actuators()) * FLAGS_Ki_;
  const Eigen::VectorXd Kd = 
    Eigen::VectorXd::Ones(quadleg->num_actuators()) * FLAGS_Kd_;
  std::cout << quadleg->num_actuators() << " actuators\n";
  // TODO: ADD P_x and P_y matrices bc this is a floating body model!!!!
  Eigen::MatrixXd P_x =
    Eigen::MatrixXd(quadleg->num_actuators() * 2, quadleg->num_multibody_states()) * 0.0; // quadleg->num_multibody_states() - 13
  P_x(0,7)  = 1.0;
  P_x(1,8)  = 1.0;
  P_x(2,15) = 1.0;
  P_x(3,16) = 1.0;
  const Eigen::MatrixXd P_y =
     Eigen::MatrixXd::Identity(quadleg->num_actuators(), quadleg->num_actuators());
  const auto* const pid = 
    builder.AddSystem<systems::controllers::PidController<double>>(P_x,P_y,Kp,Ki,Kd);

  builder.Connect(quadleg->get_state_output_port(),
                  pid->get_input_port_estimated_state());
  builder.Connect(pid->get_output_port_control(),
                  quadleg->get_actuation_input_port());

  // Set PID desired states
  auto desired_base_source = 
    builder.AddSystem<systems::ConstantVectorSource<double>>(
      Eigen::VectorXd::Zero(quadleg->num_actuators() * 2) //quadleg->num_multibody_states()
    );
  builder.Connect(desired_base_source->get_output_port(),
                  pid->get_input_port_desired_state());

  // Connect plant with scene_graph to get collision information
  DRAKE_DEMAND(!!quadleg->get_source_id());
  builder.Connect(
    quadleg->get_geometry_poses_output_port(),
    scene_graph.get_source_pose_port(quadleg->get_source_id().value())
  );
  builder.Connect(
    scene_graph.get_query_output_port(),
    quadleg->get_geometry_query_input_port()
  );

  // geometry::ConnectDrakeVisualizer(&builder, scene_graph); <-- this is old code from the example
  geometry::DrakeVisualizer<double> viz;
  viz.AddToBuilder(&builder, scene_graph);

  auto diagram = builder.Build();
  std::unique_ptr<systems::Context<double>> diagram_context = 
    diagram->CreateDefaultContext();

  // Create plant_context to set velocity
  systems::Context<double>& plant_context = 
    diagram->GetMutableSubsystemContext(*quadleg, diagram_context.get());
  // Set initial position
  Eigen::VectorXd positions = Eigen::VectorXd::Zero(quadleg->num_positions());
  positions[0] = 1.0;
  positions[8] = 0.4;
  quadleg->SetPositions(&plant_context, positions);

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(FLAGS_simulation_real_time_rate);
  simulator.Initialize();
  std::cout << "Simulator is intialized and I am ready to advance it!\n";
  simulator.get_mutable_integrator().set_target_accuracy(5e-5);
  simulator.AdvanceTo(FLAGS_simulation_time);
  std::cout << "Finished advancing simulation\n";

  return 0;
}

}  // namespace
}  // namespace quadleg
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::quadleg::do_main();
}
