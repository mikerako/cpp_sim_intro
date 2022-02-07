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
DEFINE_double(initial_height, 1.0, "initial model height");


namespace drake {
// using systems::DiagramBuilder;
// using systems::Simulator;
// using systems::Context;
// using systems::ContinuousState;
// using systems::VectorBase;

// namespace examples {
namespace quadleg {
// namespace {

static const char* const kQuadlegURDFPath = 
  "../models/urdf/leg.urdf"; // <--relative_path
  // "/home/mrako/Documents/EMBIR/cpp_sim_intro/models/urdf/leg.urdf"; <--global_path



/// Prints contact results to terminal at every timestep of simulation
///
/// @system
/// name: ContactObserver
/// input_ports:
/// - state
/// output_ports:
/// - none
/// @endsystem
///
/// This class has no public constructor; instead use the AddToBuilder() static
/// method to create and add it to a DiagramBuilder directly.
class ContactObserver final : public systems::LeafSystem<double> {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContactObserver);
  // ~ContactObserver() final;

  // /// Returns the frame of the geometry registered with a SceneGraph.  This can
  // /// be useful, e.g., if one would like to add a camera to the quadrotor.
  // geometry::FrameId get_frame_id() const { return frame_id_; }

  /// Creates, adds, and connects a QuadrotorGeometry system into the given
  /// `builder`.  Both the `quadrotor_state.get_system()` and `scene_graph`
  /// systems must have been added to the given `builder` already.
  ///
  /// The `scene_graph` pointer is not retained by the %QuadrotorGeometry
  /// system.  The return value pointer is an alias of the new
  /// %QuadrotorGeometry system that is owned by the `builder`.
  static const ContactObserver* AddToBuilder(
      systems::DiagramBuilder<double>& builder,
      const systems::OutputPort<double>& plant_contact_port,
      geometry::SceneGraph<double>& scene_graph) {
        DRAKE_THROW_UNLESS(&builder != nullptr);
        DRAKE_THROW_UNLESS(&scene_graph != nullptr);

        auto contact_observer = builder.AddSystem(
            std::unique_ptr<ContactObserver>(
                new ContactObserver()));
        builder.Connect(
            plant_contact_port,
            contact_observer->get_input_port(0));

        return contact_observer;
  }

 private:
  // explicit QuadrotorGeometry(geometry::SceneGraph<double>*);
  // void OutputGeometryPose(const systems::Context<double>&,
  //                         geometry::FramePoseVector<double>*) const;

  ContactObserver() {
    DeclareAbstractInputPort("contact", Value<multibody::ContactResults<double>>());
    // this->DeclareAbstractOutputPort(
    // DeclareAbstractOutputPort("publishing", &(this->PrintContactResults));
    DeclarePerStepPublishEvent(&ContactObserver::PrintContactResults);
    std::cout << "Constructed the ContactObserver\n";
  }

  systems::EventStatus PrintContactResults(const systems::Context<double>& context) const {
      // DRAKE_DEMAND(frame_id_.is_valid());

      // const multibody::ContactResults<double>& contact_results = get_input_port(0).Eval(context); // VectorX<double>
      const auto& contact_results = get_input_port(0).Eval<multibody::ContactResults<double>>(context); // VectorX<double>
      // std::cout << "Size of Eval: " << contact_results.size() << "\n";

      std::cout << "num point pair: " << contact_results.num_point_pair_contacts() <<
                   "\t|\tnum hydroelastic: " << contact_results.num_hydroelastic_contacts() << "\n";

      int num_of_contacts = contact_results.num_point_pair_contacts();
      for(int i = 0; i < num_of_contacts; ++i){
        multibody::PointPairContactInfo<double> pair_i = contact_results.point_pair_contact_info(i);
        std::cout << "Contact force for pair " << i << " is: " << pair_i.contact_force() << "\n";
      }
      return systems::EventStatus::Succeeded();
    }

  // // Geometry source identifier for this system to interact with SceneGraph.
  // geometry::SourceId source_id_{};
  // // The id for the quadrotor body.
  // geometry::FrameId frame_id_{};
};
//     : systems::LeafSystem<T>(systems::SystemTypeTag<QuadrotorPlant>{}),
//       g_{9.81}, m_(m_arg), L_(L_arg), kF_(kF_arg), kM_(kM_arg), I_(I_arg) {
//   // Four inputs -- one for each propellor.
//   this->DeclareInputPort("propellor_force", systems::kVectorValued, 4);
//   // State is x ,y , z, roll, pitch, yaw + velocities.
//   auto state_index = this->DeclareContinuousState(12);
//   this->DeclareStateOutputPort("state", state_index);
//   // TODO(russt): Declare input limits.  R2 has @ 2:1 thrust to weight ratio.
// }



// std::unique_ptr<systems::AffineSystem<double>> StabilizingLQRController(
//     const QuadrotorPlant<double>* quadrotor_plant,
//     Eigen::Vector3d nominal_position) {
//   auto quad_context_goal = quadrotor_plant->CreateDefaultContext();

//   Eigen::VectorXd x0 = Eigen::VectorXd::Zero(12);
//   x0.topRows(3) = nominal_position;

//   // Nominal input corresponds to a hover.
//   Eigen::VectorXd u0 = Eigen::VectorXd::Constant(
//       4, quadrotor_plant->m() * quadrotor_plant->g() / 4);

//   quadrotor_plant->get_input_port(0).FixValue(quad_context_goal.get(), u0);
//   quad_context_goal->SetContinuousState(x0);

//   // Setup LQR cost matrices (penalize position error 10x more than velocity
//   // error).
//   Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(12, 12);
//   Q.topLeftCorner<6, 6>() = 10 * Eigen::MatrixXd::Identity(6, 6);

//   Eigen::Matrix4d R = Eigen::Matrix4d::Identity();

//   return systems::controllers::LinearQuadraticRegulator(
//       *quadrotor_plant, *quad_context_goal, Q, R);
// }


multibody::MultibodyPlant<double>* add_leg(systems::DiagramBuilder<double>& builder, geometry::SceneGraph<double>& scene_graph){
  //Load and parse the leg URDF file for the MultibodyPlant
  multibody::MultibodyPlant<double>* quadleg = 
    builder.AddSystem<multibody::MultibodyPlant<double>>(FLAGS_max_time_step);
  quadleg->set_name("plant");
  quadleg->RegisterAsSourceForSceneGraph(&scene_graph);
  return quadleg;
}

void add_ground(multibody::MultibodyPlant<double>* quadleg){
  const math::RigidTransform<double> X_BG(Vector3<double>{0.0, 0.0, 0.0});
  multibody::CoulombFriction<double> surface_friction = multibody::CoulombFriction(0.7, 0.5); //static_friction, dynamic_friction
  quadleg->RegisterCollisionGeometry(quadleg->world_body(),X_BG,geometry::HalfSpace(),"ground_collision",surface_friction);
  quadleg->RegisterVisualGeometry(quadleg->world_body(),X_BG,geometry::HalfSpace(),"ground_visual",Vector4<double>{0.15234375,0.95703125,0.421875,0.65});
}

int do_main() {
  // lcm::DrakeLcm lcm;
  DRAKE_DEMAND(FLAGS_simulation_time > 0);

  systems::DiagramBuilder<double> builder;

  geometry::SceneGraph<double>& scene_graph = 
    *builder.AddSystem<geometry::SceneGraph>();
  scene_graph.set_name("scene_graph");

  // //Load and parse the leg URDF file for the MultibodyPlant
  // multibody::MultibodyPlant<double>* quadleg = 
  //   builder.AddSystem<multibody::MultibodyPlant<double>>(FLAGS_max_time_step);
  // quadleg->set_name("plant");
  // quadleg->RegisterAsSourceForSceneGraph(&scene_graph);
  multibody::MultibodyPlant<double>* quadleg = add_leg(builder, scene_graph);

  multibody::Parser parser(quadleg);
  // const std::string urdf_path = FindResourceOrThrow(kQuadlegURDFPath);
  const std::string urdf_path = kQuadlegURDFPath;
  multibody::ModelInstanceIndex plant_model_instance_index = 
    parser.AddModelFromFile(urdf_path);
  (void)plant_model_instance_index;


  add_ground(quadleg);
  // const math::RigidTransform<double> X_BG(Vector3<double>{0.0, 0.0, 0.0});
  // multibody::CoulombFriction<double> surface_friction = multibody::CoulombFriction(0.7, 0.5); //static_friction, dynamic_friction
  // quadleg->RegisterCollisionGeometry(quadleg->world_body(),X_BG,geometry::HalfSpace(),"ground_collision",surface_friction);
  // quadleg->RegisterVisualGeometry(quadleg->world_body(),X_BG,geometry::HalfSpace(),"ground_visual",Vector4<double>{0.15234375,0.95703125,0.421875,0.65});


  // quadleg->AddJointActuator("a1", quadleg->GetJointByName("Actuator1")); //Use more descriptive names for actual robot
  // quadleg->AddJointActuator("a2", quadleg->GetJointByName("Actuator2")); //Use more descriptive names for actual robot

  // Now the MultibodyPlant is complete
  quadleg->Finalize();

  // Create the Controller (TODO: decide if InverseDynamicsController or InverseDynamics w/ xtra computation after)
  // const Eigen::VectorXd Kp = 
  //   Eigen::VectorXd::Ones(quadleg->num_actuators()) * FLAGS_Kp_;
  // const Eigen::VectorXd Ki = 
  //   Eigen::VectorXd::Ones(quadleg->num_actuators()) * FLAGS_Ki_;
  // const Eigen::VectorXd Kd = 
  //   Eigen::VectorXd::Ones(quadleg->num_actuators()) * FLAGS_Kd_;
  // std::cout << quadleg->num_actuators() << " actuators\n";
  // // TODO: ADD P_x and P_y matrices bc this is a floating body model!!!!
  // Eigen::MatrixXd P_x =
  //   Eigen::MatrixXd(quadleg->num_actuators() * 2, quadleg->num_multibody_states()) * 0.0; // quadleg->num_multibody_states() - 13
  // P_x(0,7)  = 1.0;
  // P_x(1,8)  = 1.0;
  // P_x(2,15) = 1.0;
  // P_x(3,16) = 1.0;
  // const Eigen::MatrixXd P_y =
  //    Eigen::MatrixXd::Identity(quadleg->num_actuators(), quadleg->num_actuators());
  // const auto* const pid = 
  //   builder.AddSystem<systems::controllers::PidController<double>>(P_x,P_y,Kp,Ki,Kd);

  // builder.Connect(quadleg->get_state_output_port(),
  //                 pid->get_input_port_estimated_state());
  // builder.Connect(pid->get_output_port_control(),
  //                 quadleg->get_actuation_input_port());

  // // Set PID desired states
  // auto desired_base_source = 
  //   builder.AddSystem<systems::ConstantVectorSource<double>>(
  //     Eigen::VectorXd::Zero(quadleg->num_actuators() * 2) //quadleg->num_multibody_states()
  //   );
  // builder.Connect(desired_base_source->get_output_port(),
  //                 pid->get_input_port_desired_state());

  /// temp fix to remove the controller!
  auto desired_base_source = 
    builder.AddSystem<systems::ConstantVectorSource<double>>(
      Eigen::VectorXd::Zero(quadleg->num_actuators()) //quadleg->num_multibody_states()
    );
  builder.Connect(desired_base_source->get_output_port(),
                  quadleg->get_actuation_input_port()); 
  ///
                 


  ContactObserver::AddToBuilder(
      builder, quadleg->get_contact_results_output_port(), scene_graph);



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

  // Build the diagram and get a default context
  auto diagram = builder.Build();
  std::unique_ptr<systems::Context<double>> diagram_context = 
    diagram->CreateDefaultContext();


  // Create plant_context to set position and velocity of joints
  systems::Context<double>& plant_context = 
    diagram->GetMutableSubsystemContext(*quadleg, diagram_context.get());
  // Set initial position
  Eigen::VectorXd positions = Eigen::VectorXd::Zero(quadleg->num_positions());
  positions[0] = 1.0;
  positions[8] = 0.4;
  quadleg->SetPositions(&plant_context, positions);
  // Set intial velocity
  Eigen::VectorXd velocities = Eigen::VectorXd::Zero(quadleg->num_velocities());
  // positions[5] = 55.5;
  quadleg->SetVelocities(&plant_context, velocities);


  // Set initial location (rpy,xyz) of floating body for multibodyplant
  systems::State<double>& plant_state =
      diagram->GetMutableSubsystemState(*quadleg, diagram_context.get());
  const math::RigidTransform<double> X_WB(
      Vector3<double>{0.0, 0.0, FLAGS_initial_height});
  quadleg->SetFreeBodyPose(
      plant_context, &plant_state, quadleg->GetBodyByName("base_link"), X_WB); 


  // Initialize and run the simulation
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

// }  // namespace
}  // namespace quadleg
// }  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::quadleg::do_main();
}
