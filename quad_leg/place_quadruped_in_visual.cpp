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
DEFINE_double(max_time_step, 2.0e-3, "Simulation timstep used for integrator.");
DEFINE_double(Kp_, 20.0, "Kp");
DEFINE_double(Ki_, 0.0, "Ki");
DEFINE_double(Kd_, 0.0, "Kd");
DEFINE_double(initial_height, 1.0, "initial model height");


namespace drake {
namespace quadruped {

static const char* const kQuadrupedURDFPath = 
  "../models/urdf/quadf.urdf"; // <--relative_path

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
  static ContactObserver* AddToBuilder(
      systems::DiagramBuilder<double>& builder,
      const systems::OutputPort<double>& plant_contact_port,
      const systems::InputPort<double>& contact_consumer) {
        DRAKE_THROW_UNLESS(&builder != nullptr);

        auto contact_observer = builder.AddSystem(
            std::unique_ptr<ContactObserver>(
                new ContactObserver()));
        builder.Connect(
            plant_contact_port,
            contact_observer->get_input_port(0));
        builder.Connect(
            contact_observer->get_output_port(),
            contact_consumer);

        return contact_observer;
  }

private:
  ContactObserver() {
    DeclareAbstractInputPort("contact", Value<multibody::ContactResults<double>>());
    DeclarePerStepPublishEvent(&ContactObserver::PrintContactResults);
    DeclareVectorOutputPort("contacting_feet", 4, &ContactObserver::EvalFeet);
    std::cout << "Constructed the ContactObserver\n";
  }

  void EvalFeet(const systems::Context<double>& context, systems::BasicVector<double>* output) const {
    const auto& contact_results = get_input_port(0).Eval<multibody::ContactResults<double>>(context);
    // const auto& contact_results2 = get_input_port(1).Eval<multibody::ContactResults<double>>(context);

    std::cout << "PLEASEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE222222\n";

    // Declare a vector for the foot contact results and init to zero (false)
    std::cout << "Declaration here\n";
    // Vector4<double> feet_in_contact{0.0,0.0,0.0,0.0};
    Eigen::Vector4d feet_in_contact(0.0,0.0,0.0,0.0);

    std::cout << "Evaluating feet\n";

    int num_of_contacts = contact_results.num_point_pair_contacts();
    // for(int i = 0; i < num_of_contacts; ++i){
    //   multibody::PointPairContactInfo<double> pair_i = contact_results.point_pair_contact_info(i);
    //   std::cout << "Contact force for pair " << i << " is: " << pair_i.contact_force() << "\n";
    // }
    int ground_contacts = 0;
    for(int i = 0; i < num_of_contacts; ++i){
      multibody::PointPairContactInfo<double> pair_i = contact_results.point_pair_contact_info(i);
      // std::cout << "body A idx: " << pair_i.bodyA_index() << "\t|\tbody B idx: " << pair_i.bodyB_index() << "\n";
      if (pair_i.bodyA_index() == 0 || pair_i.bodyB_index() == 0){
        //one of the colliding bodies was the ground (Michael believes)
        ground_contacts++;
      }
    }

    output->set_value(feet_in_contact);
  }

  systems::EventStatus PrintContactResults(const systems::Context<double>& context) const {
    const auto& contact_results = get_input_port(0).Eval<multibody::ContactResults<double>>(context); // VectorX<double>

    // std::cout << "num point pair: " << contact_results.num_point_pair_contacts() <<
    //               "\t|\tnum hydroelastic: " << contact_results.num_hydroelastic_contacts() << "\n";

    std::cout << "PLEASEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE\n";

    int num_of_contacts = contact_results.num_point_pair_contacts();
    // for(int i = 0; i < num_of_contacts; ++i){
    //   multibody::PointPairContactInfo<double> pair_i = contact_results.point_pair_contact_info(i);
    //   std::cout << "Contact force for pair " << i << " is: " << pair_i.contact_force() << "\n";
    // }
    int ground_contacts = 0;
    for(int i = 0; i < num_of_contacts; ++i){
      multibody::PointPairContactInfo<double> pair_i = contact_results.point_pair_contact_info(i);
      // std::cout << "body A idx: " << pair_i.bodyA_index() << "\t|\tbody B idx: " << pair_i.bodyB_index() << "\n";
      if (pair_i.bodyA_index() == 0 || pair_i.bodyB_index() == 0){
        //one of the colliding bodies was the ground (Michael believes)
        ground_contacts++;
      }
    }
    // std::cout << "There were a total of " << ground_contacts << " contacts with the ground!\n";

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
  static GRFController* AddToBuilder(
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
    DeclareVectorInputPort("reference_grf", 12); //size 12 vector of forces at feet wrt base frame
    DeclareVectorInputPort("feets_in_contact", 4);

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
    const VectorX<double>& x = get_input_port(0).Eval(context);
    plant->SetPositionsAndVelocities(multibody_plant_context, x);
  }

  void CalcGeneralizedOutput(const systems::Context<double>& context, systems::BasicVector<double>* output) const {
      // auto full_state = get_input_port(0).Eval(context);
      auto desired_grf = get_input_port(1).Eval(context);
      // std::cout << "value? " << get_input_port(2).HasValue(context) << "\n";
      // auto feets_in_contact = get_input_port(2).Eval(context);
      // auto feets_in_contact = get_input_port(2).HasValue(context);
      // std::cout << "\nfeets\n" << feets_in_contact << "\n\n";
      std::vector<bool> feet_in_contact = {true,true,true,true};

      // initialize a 12x1 array that represents the output torques 
      Eigen::Matrix<double,12,1> output_torques;
      // desired_forces << 0.0,0.0,50.0,0.0,0.0,50.0,0.0,0.0,50.0,0.0,0.0,50.0;

      // grab mbp context
      const auto& multibody_plant_context =
              this->get_cache_entry(multibody_plant_context_cache_index)
                  .Eval<systems::Context<double>>(context);

      VectorX<double> tau_g = -plant->CalcGravityGeneralizedForces(multibody_plant_context);
      std::cout << "tau_g...\n" << tau_g << "\n\n";

      for(int leg_idx = 0; leg_idx < feet_in_contact.size(); ++leg_idx){
        std::cout << "Leg " << leg_idx << " is in contact? " << feet_in_contact[leg_idx] << "\n";
        if(feet_in_contact[leg_idx]){
          // This foot is in contact, so we actually do want to calculate the reuired torques via
          // this controller
          auto fi = desired_grf.segment<3>(leg_idx*3);
          std::cout << "Desired grf for this leg is " << fi << "\n";
        
          // const int num_generalized_positions = plant->num_actuators() / 4;
          const int num_generalized_positions = plant->num_positions();
          Matrix6X<double> Jq_V_WEp(6, num_generalized_positions);

          // systems::Context<double>& plant_context = systems::Diagram<double>::GetMutableSubsystemContext(plant, context);
          switch(leg_idx) {
            case 0: 
              //front left leg
              plant->CalcJacobianSpatialVelocity(multibody_plant_context,//plant->get_context(), 
                                                 multibody::JacobianWrtVariable::kQDot, 
                                                 plant->GetFrameByName("left front LL"), 
                                                 Vector3<double>{0.0,0.0,0.0},
                                                //  plant->GetFrameByName("front left hip"), 
                                                //  plant->GetFrameByName("front left hip"),
                                                 plant->GetFrameByName("base"), 
                                                 plant->GetFrameByName("base"),
                                                 &Jq_V_WEp);
              break;
            case 1:
              //
              plant->CalcJacobianSpatialVelocity(multibody_plant_context,//plant->get_context(), 
                                                 multibody::JacobianWrtVariable::kQDot, 
                                                 plant->GetFrameByName("right front LL"), 
                                                 Vector3<double>{0.0,0.0,0.0},
                                                //  plant->GetFrameByName("front right hip"), 
                                                //  plant->GetFrameByName("front right hip"),
                                                 plant->GetFrameByName("base"), 
                                                 plant->GetFrameByName("base"),
                                                 &Jq_V_WEp);
              break;
            case 2:
              plant->CalcJacobianSpatialVelocity(multibody_plant_context,//plant->get_context(), 
                                                 multibody::JacobianWrtVariable::kQDot, 
                                                 plant->GetFrameByName("left back LL"), 
                                                 Vector3<double>{0.0,0.0,0.0},
                                                //  plant->GetFrameByName("Back left hip"), 
                                                //  plant->GetFrameByName("Back left hip"),
                                                 plant->GetFrameByName("base"), 
                                                 plant->GetFrameByName("base"),
                                                 &Jq_V_WEp);
              break;
            case 3:
              plant->CalcJacobianSpatialVelocity(multibody_plant_context,//plant->get_context(), 
                                                 multibody::JacobianWrtVariable::kQDot, 
                                                 plant->GetFrameByName("right back LL"), 
                                                 Vector3<double>{0.0,0.0,0.0},
                                                //  plant->GetFrameByName("Back right hip"), 
                                                //  plant->GetFrameByName("Back right hip"),
                                                 plant->GetFrameByName("base"), 
                                                 plant->GetFrameByName("base"),
                                                 &Jq_V_WEp);
              break;
            // no default bc we do nothing bc should never happen
          }
          
          // std::cout << Jq_V_WEp.block<3,2>(3,8+leg_idx*3) << "\n\n";

          //torque = Ji.T * Ri.T * fi;
          //// auto Ji = Jq_V_WEp.bottomRightCorner<3,2>();
          auto Ji = Jq_V_WEp.block<3,3>(3,7+leg_idx*3);
          std::cout << "Ji:\n" << Ji << "\n";
          auto torque_leg_i = Ji.transpose() * fi;
          // std::cout << "test:\n" << output_torques.segment<3>(leg_idx*3) << "\n";
          // std::cout << "test2:\n" << torque_leg_i << "\n";
          output_torques.segment<3>(leg_idx*3) = torque_leg_i;

        }
      }

      // auto torque = Ji.transpose() * desired_grf;
      std::cout << "torque:" << output_torques << "\n\n";
      // std::cout << "torque+tau_g:" << torque + tau_g.tail(2) << "\n\n";
      // // set output
      // output->set_value(FLAGS_Kp_ * (torque + tau_g.tail(2)));
      
      output->set_value(output_torques);
      // output->set_value(desired_grf);

    }

    const multibody::MultibodyPlant<double>* plant;
    // std::unique_ptr<systems::Context<double>> multibody_plant_context;
    systems::CacheIndex multibody_plant_context_cache_index;
};



multibody::MultibodyPlant<double>* add_mbp(systems::DiagramBuilder<double>& builder, geometry::SceneGraph<double>& scene_graph){
  //Load and parse the leg URDF file for the MultibodyPlant
  multibody::MultibodyPlant<double>* mbp = 
    builder.AddSystem<multibody::MultibodyPlant<double>>(FLAGS_max_time_step);
  mbp->set_name("plant");
  mbp->RegisterAsSourceForSceneGraph(&scene_graph);
  return mbp;
}

void add_ground(multibody::MultibodyPlant<double>* mbp){
  const math::RigidTransform<double> X_BG(Vector3<double>{0.0, 0.0, 0.0});
  multibody::CoulombFriction<double> surface_friction = multibody::CoulombFriction(0.7, 0.6); //static_friction, dynamic_friction
  mbp->RegisterCollisionGeometry(mbp->world_body(),X_BG,geometry::HalfSpace(),"ground_collision",surface_friction);
  mbp->RegisterVisualGeometry(mbp->world_body(),X_BG,geometry::HalfSpace(),"ground_visual",Vector4<double>{0.15234375,0.95703125,0.421875,0.65});
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
  multibody::MultibodyPlant<double>* quadruped = add_mbp(builder, scene_graph);

  multibody::Parser parser(quadruped);
  const std::string urdf_path = kQuadrupedURDFPath;
  multibody::ModelInstanceIndex plant_model_instance_index = 
    parser.AddModelFromFile(urdf_path);
  // quadruped->AddFrame(FixedOffsetFrame("left front foot",
  //                                      quadruped->GetFrameByName(""),
  //                                      RigidTransform(),
  //                                      plant_model_instance_index));
  (void)plant_model_instance_index;


  // add collision geometry and visuals for the ground
  add_ground(quadruped);

  // Now the MultibodyPlant is complete
  quadruped->Finalize();


  // temp hack for having a simple standing controller!
  Eigen::Matrix<double,12,1> desired_forces;
  double z_force = -100.0;
  desired_forces << 0.0,0.0,z_force,0.0,0.0,z_force,0.0,0.0,z_force,0.0,0.0,z_force;
  
  auto desired_base_source = 
    builder.AddSystem<systems::ConstantVectorSource<double>>(
      // Eigen::Matrix<double,12,1>{0.0,0.0,50.0,0.0,0.0,50.0,0.0,0.0,50.0,0.0,0.0,50.0}
      desired_forces
      // Eigen::VectorXd::Zero(12) // Command all actuators of the quadruped to be non-actuated
    );
  // builder.Connect(desired_base_source->get_output_port(),
  //                 quadruped->get_actuation_input_port()); 
  GRFController* grf_controller = //static const GRFController*
    GRFController::AddToBuilder(builder, desired_base_source, quadruped);
  
  std::cout << "added controller to the diagram\n";
                 
  // Add (simulation) foot contact "estimator"
  auto contact_observer = ContactObserver::AddToBuilder(builder, quadruped->get_contact_results_output_port(), grf_controller->get_input_port(2));

  // builder.Connect(
  //           contact_observer->get_output_port(),
  //           grf_controller->get_input_port(2));

  std::cout << "added custom things to the diagram\n";

  // Connect plant with scene_graph to get collision information
  DRAKE_DEMAND(!!quadruped->get_source_id());
  builder.Connect(
    quadruped->get_geometry_poses_output_port(),
    scene_graph.get_source_pose_port(quadruped->get_source_id().value())
  );
  builder.Connect(
    scene_graph.get_query_output_port(),
    quadruped->get_geometry_query_input_port()
  );

  // geometry::ConnectDrakeVisualizer(&builder, scene_graph); <-- this is old code from the example
  geometry::DrakeVisualizer<double> viz;
  viz.AddToBuilder(&builder, scene_graph);

  // Build the diagram and get a default context
  auto diagram = builder.Build();
  std::unique_ptr<systems::Context<double>> diagram_context = 
    diagram->CreateDefaultContext();


  std::cout << "Default Context\n";

  // Create plant_context to set position and velocity of joints
  systems::Context<double>& plant_context = 
    diagram->GetMutableSubsystemContext(*quadruped, diagram_context.get());
  // Set initial position
  Eigen::VectorXd positions = Eigen::VectorXd::Zero(quadruped->num_positions());
  positions[0] = 1.0;
  positions[8] = -.4;
  positions[9] = .4;
  positions[11] = .4;
  positions[12] = -.4;
  positions[14] = .4;
  positions[15] = -.4;
  positions[17] = -.4;
  positions[18] = .4;
  quadruped->SetPositions(&plant_context, positions);
  // // Set intial velocity
  // Eigen::VectorXd velocities = Eigen::VectorXd::Zero(quadleg->num_velocities());
  // // positions[5] = 55.5;
  // quadleg->SetVelocities(&plant_context, velocities);
  std::cout << quadruped->CalcTotalMass(plant_context) << "\n";

  // Set initial location (rpy,xyz) of floating body for multibodyplant
  systems::State<double>& plant_state =
      diagram->GetMutableSubsystemState(*quadruped, diagram_context.get());
  const math::RigidTransform<double> X_WB(
      // math::RollPitchYaw<double>(Vector3<double>{0.0,3.14,0.0}),
      Vector3<double>{0.0, 0.0, FLAGS_initial_height});
  quadruped->SetFreeBodyPose(
      plant_context, &plant_state, quadruped->GetBodyByName("base"), X_WB); 


  std::cout << "Hi\n";

  // Initialize and run the simulation
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  std::cout << "Here\n";
  simulator.set_publish_every_time_step(true);
  std::cout << "Here2\n";
  simulator.set_target_realtime_rate(FLAGS_simulation_real_time_rate);
  std::cout << "Here3\n";
  simulator.Initialize();
  std::cout << "Here4\n";
  std::cout << "Simulator is intialized and I am ready to advance it!\n";
  simulator.get_mutable_integrator().set_target_accuracy(5e-5);
  simulator.AdvanceTo(FLAGS_simulation_time);
  std::cout << "Finished advancing simulation\n";

  return 0;
}

}  // namespace quadleg
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::quadruped::do_main();
}
