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
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

#include "drake/common/find_resource.h"
#include "drake/geometry/drake_visualizer.h"

DEFINE_double(simulation_real_time_rate, 1.0, "Real time rate");
DEFINE_double(simulation_time, 7.0, "How long to simulate the system for");
DEFINE_double(max_time_step, 1.0e-4, "Simulation timstep used for integrator.");
DEFINE_double(Kp_, 100.0, "Kp");
DEFINE_double(Ki_, 0.0, "Ki");
DEFINE_double(Kd_, 0.0, "Kd");
DEFINE_double(initial_height, 1.0, "initial model height");


namespace drake {
namespace quadleg {

static const char* const kQuadlegURDFPath = 
  "../models/urdf/leg.urdf"; // <--relative_path
  // "/home/mrako/Documents/EMBIR/cpp_sim_intro/models/urdf/leg.urdf"; <--global_path


/// Prints contact results to terminal at every timestep of simulation
///
/// @system
/// name: ContactObserver
/// input_ports:
/// - contact_results from MBP
/// output_ports:
/// - none
/// @endsystem
///
/// This class has no public constructor; instead use the AddToBuilder() static
/// method to create and add it to a DiagramBuilder directly.
class ContactObserver final : public systems::LeafSystem<double> {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContactObserver);

  /// Creates, adds, and connects a ContactObserver system into the given
  /// `builder`.
  ///
  /// The return value pointer is an alias of the new
  /// %ContactObserver system that is owned by the `builder`.
  static const ContactObserver* AddToBuilder(
      systems::DiagramBuilder<double>& builder,
      const systems::OutputPort<double>& plant_contact_port) {
        DRAKE_THROW_UNLESS(&builder != nullptr);

        auto contact_observer = builder.AddSystem(
            std::unique_ptr<ContactObserver>(
                new ContactObserver()));
        builder.Connect(
            plant_contact_port,
            contact_observer->get_input_port(0));

        return contact_observer;
  }

private:
  ContactObserver() {
    DeclareAbstractInputPort("contact", Value<multibody::ContactResults<double>>());
    DeclarePerStepPublishEvent(&ContactObserver::PrintContactResults);
    std::cout << "Constructed the ContactObserver\n";
  }

  systems::EventStatus PrintContactResults(const systems::Context<double>& context) const {
    const auto& contact_results = get_input_port(0).Eval<multibody::ContactResults<double>>(context); // VectorX<double>

    // std::cout << "num point pair: " << contact_results.num_point_pair_contacts() <<
    //               "\t|\tnum hydroelastic: " << contact_results.num_hydroelastic_contacts() << "\n";

    // int num_of_contacts = contact_results.num_point_pair_contacts();
    // for(int i = 0; i < num_of_contacts; ++i){
    //   multibody::PointPairContactInfo<double> pair_i = contact_results.point_pair_contact_info(i);
    //   std::cout << "Contact force for pair " << i << " is: " << pair_i.contact_force() << "\n";
    // }
    return systems::EventStatus::Succeeded();
  }
};


/// Custom controller that computes torque required to achieve a 
/// desired ground reaction force (GRF)
///
/// @system
/// name: GRFController
/// input_ports:
/// - state
/// - reference GRF to achieve
/// output_ports:
/// - control inputs for multibodyplant (MBP)
/// @endsystem
///
/// This class has no public constructor; instead use the AddToBuilder() static
/// method to create and add it to a DiagramBuilder directly.
class GRFController final : public systems::LeafSystem<double> {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GRFController);

  /// Creates, adds, and connects a GRFController system into the given
  /// `builder`.
  ///
  /// The return value pointer is an alias of the new
  /// %GRFController system that is owned by the `builder`.
  static const GRFController* AddToBuilder(
      systems::DiagramBuilder<double>& builder,
      // const systems::OutputPort<double>& plant_state_port,
      // const systems::InputPort<double>& plant_actuation_port,
      const systems::ConstantVectorSource<double>* reference,
      const multibody::MultibodyPlant<double>* mbp) {
        DRAKE_THROW_UNLESS(&builder != nullptr);

        auto grf_controller = builder.AddSystem(
            std::unique_ptr<GRFController>(
                new GRFController(mbp)));
        builder.Connect(
            mbp->get_state_output_port(),
            grf_controller->get_input_port(0));
        builder.Connect(
            reference->get_output_port(),
            grf_controller->get_input_port(1));
        builder.Connect(
            grf_controller->get_output_port(0),
            mbp->get_actuation_input_port());

        return grf_controller;
  }

 private:
  GRFController(const multibody::MultibodyPlant<double>* mbp) 
    : plant(mbp) {
    // Define controller input ports
    DeclareVectorInputPort("state", mbp->num_multibody_states());
    DeclareVectorInputPort("reference_grf", 3); //size 3 vector of forces at foot wrt foot frame

    // Define controller output ports
    DeclareVectorOutputPort("output_torques", mbp->num_actuators(), &GRFController::CalcGeneralizedOutput);
    std::unique_ptr<systems::Context<double>> multibody_plant_context = plant->CreateDefaultContext();

    // Declare cache entry for the multibody plant context.
    multibody_plant_context_cache_index =
        this->DeclareCacheEntry(
                "multibody_plant_context_cache",
                *multibody_plant_context,
                &GRFController::SetMultibodyContext,
                {this->input_port_ticket(
                    get_input_port(0).get_index())})
            .cache_index();

    std::cout << "Constructed the GRFController\n";
  }


  void SetMultibodyContext(const systems::Context<double>& context,
                           systems::Context<double>* multibody_plant_context) const {
    // const VectorX<double>& x = get_input_port_estimated_state().Eval(context);
    const VectorX<double>& x = get_input_port(0).Eval(context);

    // if (this->is_pure_gravity_compensation()) {
    //   // Velocities remain zero, as set in the constructor, for pure gravity
    //   // compensation mode.
    //   const VectorX<T> q = x.head(multibody_plant_->num_positions());
    //   multibody_plant_->SetPositions(multibody_plant_context, q);
    // } else {
      // Set the plant positions and velocities.
      plant->SetPositionsAndVelocities(multibody_plant_context, x);
    // }
  }


  void CalcGeneralizedOutput(const systems::Context<double>& context, systems::BasicVector<double>* output) const {
      auto full_state = get_input_port(0).Eval(context);
      auto desired_grf = get_input_port(1).Eval(context);
      // plant.SetPositionsAndVelocities(full_state)

      // const auto& contact_results = get_input_port(0).Eval<multibody::ContactResults<double>>(context); // VectorX<double>
      // std::cout << "Size of Eval: " << contact_results.size() << "\n";

      // plant.CalcGravityGeneralizedForces(context);

      // # Compute foot jacobian
      //   J = self.plant.CalcJacobianSpatialVelocity(self.context,
      //                                              JacobianWrtVariable.kV,
      //                                              self.foot_frame,
      //                                              np.zeros(3),
      //                                              self.plant.world_frame(),
      //                                              self.plant.world_frame())

      const int num_generalized_positions = plant->num_positions();
      Matrix6X<double> Jq_V_WEp(6, num_generalized_positions);

      // auto& multibodyplant = *plant;

      const auto& multibody_plant_context =
          this->get_cache_entry(multibody_plant_context_cache_index)
              .Eval<systems::Context<double>>(context);

      VectorX<double> tau_g = -plant->CalcGravityGeneralizedForces(multibody_plant_context);
      std::cout << tau_g << "\n\n";

      // systems::Context<double>& plant_context = systems::Diagram<double>::GetMutableSubsystemContext(plant, context);
      plant->CalcJacobianSpatialVelocity(multibody_plant_context,//plant->get_context(), 
                                         multibody::JacobianWrtVariable::kQDot, 
                                         plant->GetFrameByName("foot_link"), 
                                         Vector3<double>{0.0,0.0,0.0},
                                         plant->GetFrameByName("base_link"), 
                                         plant->GetFrameByName("base_link"),
                                         &Jq_V_WEp);

      std::cout << Jq_V_WEp << "\n\n";

      //torque = Ji.T * Ri.T * fi;
      auto Ji = Jq_V_WEp.bottomRightCorner<3,2>();
      // std::cout << Ji << "\n\n";
      // std::cout << desired_grf << "\n\n";
      // std::cout << Ji.transpose() << "\n\n";


      auto torque = Ji.transpose() * desired_grf;
      std::cout << "torque:" << torque << "\n\n";
      std::cout << "torque+tau_g:" << torque + tau_g.tail(2) << "\n\n";
      // set output
      output->set_value(FLAGS_Kp_ * (torque + tau_g.tail(2)));
      // output->set_value(Vector2<double>{20.0,0.0});
      // output[0] = 0;
      // output[1] = 0;
    }

    const multibody::MultibodyPlant<double>* plant;
    // std::unique_ptr<systems::Context<double>> multibody_plant_context;
    systems::CacheIndex multibody_plant_context_cache_index;
};



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
  DRAKE_DEMAND(FLAGS_simulation_time > 0);

  // make a DiagramBuilder
  systems::DiagramBuilder<double> builder;

  // make a SceneGraph
  geometry::SceneGraph<double>& scene_graph = 
    *builder.AddSystem<geometry::SceneGraph>();
  scene_graph.set_name("scene_graph");

  // //Load and parse the leg URDF file for the MultibodyPlant
  multibody::MultibodyPlant<double>* quadleg = add_leg(builder, scene_graph);

  multibody::Parser parser(quadleg);
  const std::string urdf_path = kQuadlegURDFPath;
  multibody::ModelInstanceIndex plant_model_instance_index = 
    parser.AddModelFromFile(urdf_path);
  (void)plant_model_instance_index;

  // add collision geometry and visuals for the ground
  add_ground(quadleg);

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
      // Eigen::VectorXd::Zero(3) //Eigen::VectorXd::Zero(quadleg->num_actuators()) //quadleg->num_multibody_states()
      Vector3<double>{0.0,0.0,50.0}
    );
  ///////////// builder.Connect(desired_base_source->get_output_port(),
  /////////////                 quadleg->get_actuation_input_port()); 
  ///
  GRFController::AddToBuilder(builder, desired_base_source, quadleg);
                 


  ContactObserver::AddToBuilder(
      builder, quadleg->get_contact_results_output_port());



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
  positions[7] = -1.4;
  positions[8] = 2.4;
  quadleg->SetPositions(&plant_context, positions);
  // Set intial velocity
  Eigen::VectorXd velocities = Eigen::VectorXd::Zero(quadleg->num_velocities());
  // positions[5] = 55.5;
  quadleg->SetVelocities(&plant_context, velocities);


  // Set initial location (rpy,xyz) of floating body for multibodyplant
  systems::State<double>& plant_state =
      diagram->GetMutableSubsystemState(*quadleg, diagram_context.get());
  const math::RigidTransform<double> X_WB(
      math::RollPitchYaw<double>(Vector3<double>{0.0,3.14,0.0}),
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
