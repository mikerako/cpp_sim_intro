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
#include "drake/systems/primitives/zero_order_hold.h"

#include "drake/common/find_resource.h"
#include "drake/geometry/drake_visualizer.h"

DEFINE_double(simulation_realtime_rate, 1.0, "Realtime rate");
DEFINE_double(simulation_time, 7.0, "How long to simulate the system for");
DEFINE_double(max_timestep, 2.0e-3, "Simulation timstep used for integrator.");
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
/// - contact_results (from MBP)
/// output_ports:
/// - feet_in_contact (which feet are in contact with the ground)
/// @endsystem
///
/// usage: Use the AddToBuilder() static method to construct
/// this class and add it to a DiagramBuilder directly.
class ContactObserver final : public systems::LeafSystem<double> {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContactObserver);

  /// Creates and adds a ContactObserver system into the given
  /// `builder`.
  /// The return value pointer is an alias of the new
  /// %ContactObserver system that is owned by the `builder`.
  static ContactObserver* AddToBuilder(systems::DiagramBuilder<double>& builder) {
    DRAKE_THROW_UNLESS(&builder != nullptr);

    auto contact_observer = builder.AddSystem(
        std::unique_ptr<ContactObserver>(
            new ContactObserver()));
    return contact_observer;
  }

  ContactObserver() {
    DeclareAbstractInputPort("contact_results", Value<multibody::ContactResults<double>>());
    DeclareVectorOutputPort("feet_in_contact", 4, &ContactObserver::EvalFeet);
  }

private:
  void EvalFeet(const systems::Context<double>& context, systems::BasicVector<double>* output) const {
    const multibody::ContactResults<double>& contact_results = 
      get_input_port(0).Eval<multibody::ContactResults<double>>(context);

    // Declare a vector for the foot contact results and init to zero (false)
    Eigen::VectorXd feet_in_contact = Eigen::VectorXd::Zero(4);

    int num_of_contacts = contact_results.num_point_pair_contacts();
    for(int i = 0; i < num_of_contacts; ++i){
      multibody::PointPairContactInfo<double> pair_i = contact_results.point_pair_contact_info(i);
    //   std::cout << "Contact force for pair " << i << " is: " << pair_i.contact_force() << "\n";
    }
    int ground_contacts = 0;
    for(int i = 0; i < num_of_contacts; ++i){
      multibody::PointPairContactInfo<double> pair_i = contact_results.point_pair_contact_info(i);
      std::cout << "body A idx: " << pair_i.bodyA_index() << "\t|\tbody B idx: " << pair_i.bodyB_index() << "\n";
      // TODO ADDRESS CONCERN: IN THE CONTACT RESULTS, A COLLISION BETWEEN IDX 1 (body/torso) and the 4 femurs is always present.
      
      // check if one of the colliding bodies was the ground (body index of the world/ground is 0)
      if (pair_i.bodyA_index() == 0){
        if (pair_i.bodyB_index() == 4)        //hardcoded currently TODO make class member var (front left)
          feet_in_contact[0] = 1;
        else if (pair_i.bodyB_index() == 7)   //hardcoded currently TODO make class member var (front right)
          feet_in_contact[1] = 1;
        else if (pair_i.bodyB_index() == 10)  //hardcoded currently TODO make class member var (back left)
          feet_in_contact[2] = 1;
        else if (pair_i.bodyB_index() == 13)  //hardcoded currently TODO make class member var (back right)
          feet_in_contact[3] = 1;
        ground_contacts++;

      } else if (pair_i.bodyB_index() == 0){
        if (pair_i.bodyA_index() == 4)        //hardcoded currently TODO make class member var (front left)
          feet_in_contact[0] = 1;
        else if (pair_i.bodyA_index() == 7)   //hardcoded currently TODO make class member var (front right)
          feet_in_contact[1] = 1;
        else if (pair_i.bodyA_index() == 10)  //hardcoded currently TODO make class member var (back left)
          feet_in_contact[2] = 1;
        else if (pair_i.bodyA_index() == 13)  //hardcoded currently TODO make class member var (back right)
          feet_in_contact[3] = 1;
        ground_contacts++;
      }
    } // end for

    feet_in_contact[0] = 1;
    feet_in_contact[1] = 1;
    feet_in_contact[2] = 1;
    feet_in_contact[3] = 1;

    output->set_value(feet_in_contact);
  }
};


/// Custom controller that computes torque required to achieve a 
/// desired ground reaction force (GRF)
///
/// @system
/// name: GRFController
/// input_ports:
/// - state
/// - reference_grf   (GRFs to achieve with each foot)
/// - feet_in_contact (which feet are in contact with the ground)
/// output_ports:
/// - actuation_torques (sent to ZOH for MBP)
/// @endsystem
///
/// usage: Use the AddToBuilder() static method to create
/// this class and add it to a DiagramBuilder directly.
class GRFController final : public systems::LeafSystem<double> {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GRFController);

  /// Creates and adds a GRFController system into the given
  /// `builder`.
  /// The return value pointer is an alias of the new
  /// %GRFController system that is owned by the `builder`.
  // static GRFController* AddToBuilder(
  //     systems::DiagramBuilder<double>& builder,
  //     const systems::ConstantVectorSource<double>* reference,
  //     const multibody::MultibodyPlant<double>* mbp) {
  static GRFController* AddToBuilder(
    systems::DiagramBuilder<double>& builder,
    const multibody::MultibodyPlant<double>* mbp) {
    DRAKE_THROW_UNLESS(&builder != nullptr);

    auto grf_controller = builder.AddSystem(
         std::make_unique<GRFController>(mbp));
    // builder.Connect(
    //     mbp->get_state_output_port(),
    //     grf_controller->get_input_port(0));
    // builder.Connect(
    //     reference->get_output_port(),
    //     grf_controller->get_input_port(1));
    return grf_controller;
  }

  GRFController(const multibody::MultibodyPlant<double>* mbp) 
    : plant(mbp) {
    // Define controller input ports
    DeclareVectorInputPort("state", mbp->num_multibody_states());
    DeclareVectorInputPort("reference_grf", 12);  //size 12 vector of forces at feet wrt base frame
    DeclareVectorInputPort("feet_in_contact", 4); //size 4  vector of whether feet are in contact 

    // Define controller output ports
    DeclareVectorOutputPort("actuation_torques", mbp->num_actuators(), &GRFController::CalcGeneralizedOutput);
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
  }

private:
  void SetMultibodyContext(const systems::Context<double>& context,
                           systems::Context<double>* multibody_plant_context) const {
    const VectorX<double>& x = get_input_port(0).Eval(context);
    plant->SetPositionsAndVelocities(multibody_plant_context, x);
  }

  void CalcGeneralizedOutput(const systems::Context<double>& context, systems::BasicVector<double>* output) const {
    const VectorX<double>& desired_grf = get_input_port(1).Eval(context);
    const VectorX<double>& feet_in_contact = get_input_port(2).Eval(context);

    // initialize a 12x1 array that represents the output torques 
    Eigen::Matrix<double,12,1> output_torques;
    output_torques.setZero();

    // grab mbp context
    const auto& multibody_plant_context = this->get_cache_entry(multibody_plant_context_cache_index)
                                          .Eval<systems::Context<double>>(context);

    for(int foot_idx = 0; foot_idx < feet_in_contact.size(); ++foot_idx){
      std::cout << "foot " << foot_idx << " is in contact? " << feet_in_contact[foot_idx] << "\n";
      if(feet_in_contact[foot_idx]){
        // This foot is in contact, so we actually do want to calculate the reuired torques via
        // this controller
        auto fi = desired_grf.segment<3>(foot_idx*3);
        // std::cout << "Desired grf for this leg is " << fi << "\n";
        
        // const int num_generalized_positions = plant->num_actuators() / 4;
        const int num_generalized_positions = plant->num_positions();
        Matrix6X<double> Jq_V_WEp(6, num_generalized_positions);

        // systems::Context<double>& plant_context = systems::Diagram<double>::GetMutableSubsystemContext(plant, context);
        switch(foot_idx) {
          case 0: 
            //front left leg
            plant->CalcJacobianSpatialVelocity(multibody_plant_context,//plant->get_context(), 
                                               multibody::JacobianWrtVariable::kQDot, 
                                               plant->GetFrameByName("left front foot"), 
                                               Vector3<double>{0.0,0.0,0.0},
                                               plant->GetFrameByName("base"), 
                                               plant->GetFrameByName("base"),
                                               &Jq_V_WEp);
            break;
          case 1:
            //front right leg
            plant->CalcJacobianSpatialVelocity(multibody_plant_context,//plant->get_context(), 
                                               multibody::JacobianWrtVariable::kQDot, 
                                               plant->GetFrameByName("right front foot"), 
                                               Vector3<double>{0.0,0.0,0.0},
                                               plant->GetFrameByName("base"), 
                                               plant->GetFrameByName("base"),
                                               &Jq_V_WEp);
            break;
          case 2:
            //back left leg
            plant->CalcJacobianSpatialVelocity(multibody_plant_context,//plant->get_context(), 
                                               multibody::JacobianWrtVariable::kQDot, 
                                               plant->GetFrameByName("left back foot"), 
                                               Vector3<double>{0.0,0.0,0.0},
                                               plant->GetFrameByName("base"), 
                                               plant->GetFrameByName("base"),
                                               &Jq_V_WEp);
            break;
          case 3:
            //back right leg
            plant->CalcJacobianSpatialVelocity(multibody_plant_context,//plant->get_context(), 
                                               multibody::JacobianWrtVariable::kQDot, 
                                               plant->GetFrameByName("right back foot"), 
                                               Vector3<double>{0.0,0.0,0.0},
                                               plant->GetFrameByName("base"), 
                                               plant->GetFrameByName("base"),
                                               &Jq_V_WEp);
            break;
          // no default. we do nothing bc should never happen
        }
        //torque = Ji.T * Ri.T * fi;
        // std::cout << "\n" << Jq_V_WEp << "\n";
        // pull out the linear velocity jacobian
        auto Ji = Jq_V_WEp.block<3,3>(3,7+foot_idx*3);
        // std::cout << "Ji:\n" << Ji << "\n";
        auto torque_leg_i = Ji.transpose() * fi;
        output_torques.segment<3>(foot_idx*3) = torque_leg_i;
      }
    }

    // compensate for directionality of actuators i guess
    // for (int i = 2; i < output_torques.size() / 2.0; i += 3){
    //   output_torques[i] *= -1.0;
    // }

    std::cout << "torque:\n" << output_torques << "\n\n";

    // set output
    output->set_value(output_torques);
  }

  // private member variable of GRFController
  const multibody::MultibodyPlant<double>* plant;
  systems::CacheIndex multibody_plant_context_cache_index;
};



multibody::MultibodyPlant<double>* add_mbp(systems::DiagramBuilder<double>& builder, geometry::SceneGraph<double>& scene_graph){
  //Load and parse the leg URDF file for the MultibodyPlant
  multibody::MultibodyPlant<double>* mbp = 
    builder.AddSystem<multibody::MultibodyPlant<double>>(FLAGS_max_timestep);
  mbp->set_name("plant");
  mbp->RegisterAsSourceForSceneGraph(&scene_graph);
  return mbp;
}

void add_ground(multibody::MultibodyPlant<double>* mbp, Vector4<double> color = {0.15234375,0.95703125,0.421875,0.65}){
  const math::RigidTransform<double> X_BG(Vector3<double>{0.0, 0.0, 0.0});
  multibody::CoulombFriction<double> surface_friction = multibody::CoulombFriction(2.0, 2.0); //static_friction, dynamic_friction
  mbp->RegisterCollisionGeometry(mbp->world_body(),X_BG,geometry::HalfSpace(),"ground_collision",surface_friction);
  mbp->RegisterVisualGeometry(mbp->world_body(),X_BG,geometry::HalfSpace(),"ground_visual",color);
}

int do_main() {
  DRAKE_DEMAND(FLAGS_simulation_time > 0);

  // make a DiagramBuilder
  systems::DiagramBuilder<double> builder;

  // make a SceneGraph
  geometry::SceneGraph<double>& scene_graph = 
    *builder.AddSystem<geometry::SceneGraph>();
  scene_graph.set_name("scene_graph");

  // create and add a MultiBodyPlant to the builder
  multibody::MultibodyPlant<double>* quadruped = add_mbp(builder, scene_graph);

  // load and parse the leg URDF file for the MultibodyPlant
  multibody::Parser parser(quadruped);
  const std::string urdf_path = kQuadrupedURDFPath;
  multibody::ModelInstanceIndex plant_model_instance_index = 
    parser.AddModelFromFile(urdf_path);
  (void)plant_model_instance_index;

  // add collision geometry and visuals for the ground
  add_ground(quadruped);

  // Now, the MultibodyPlant is complete
  quadruped->Finalize();

  // temp hack for having a simple standing controller!
  Eigen::Matrix<double,12,1> desired_forces;
  double z_force = -36.0;
  desired_forces << 0.0,0.0,z_force, 0.0,0.0,z_force,0.0,0.0,z_force,0.0,0.0,z_force;
  // desired_forces << 0.0,z_force,0.0, 0.0,z_force,0.0, 0.0,z_force,0.0, 0.0,z_force,0.0;
  auto desired_base_source = builder.AddSystem<systems::ConstantVectorSource<double>>(desired_forces);

  // add a zero-order hold in between the controller and the plant actuation input so that we don't
  // create an algebraic loop
  auto ZOH = builder.AddSystem<systems::ZeroOrderHold<double>>(FLAGS_max_timestep, quadruped->num_actuators());
  builder.Connect(ZOH->get_output_port(),quadruped->get_actuation_input_port());
  
  // create a ground reaction force (grf) controller to actuate legs that are in contact
  GRFController* grf_controller = GRFController::AddToBuilder(builder, quadruped);
  builder.Connect(quadruped->get_state_output_port(),grf_controller->get_input_port(0));
  builder.Connect(desired_base_source->get_output_port(),grf_controller->get_input_port(1));
  builder.Connect(grf_controller->get_output_port(),ZOH->get_input_port());

  // Add (simulation) foot contact "estimator"
  auto contact_observer = ContactObserver::AddToBuilder(builder);
  builder.Connect(quadruped->get_contact_results_output_port(), contact_observer->get_input_port());
  builder.Connect(contact_observer->get_output_port(), grf_controller->get_input_port(2));

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

  // Create a visualizer instance and connect SceneGraph to it
  geometry::DrakeVisualizer<double> viz;
  viz.AddToBuilder(&builder, scene_graph);

  // Build the diagram and get a default context
  auto diagram = builder.Build();
  std::unique_ptr<systems::Context<double>> diagram_context = 
    diagram->CreateDefaultContext();

  // Create plant_context to set position and velocity of joints
  systems::Context<double>& plant_context = 
    diagram->GetMutableSubsystemContext(*quadruped, diagram_context.get());

  // Set initial position
  Eigen::VectorXd positions = Eigen::VectorXd::Zero(quadruped->num_positions());
  positions[0] = 1.0;
  positions[8] = .8;
  positions[9] = -1.6;
  positions[11] = -.8;
  positions[12] = 1.6;
  positions[14] = .8;
  positions[15] = -1.6;
  positions[17] = -.8;
  positions[18] = 1.6;
  quadruped->SetPositions(&plant_context, positions);
  // // Set intial velocity
  // Eigen::VectorXd velocities = Eigen::VectorXd::Zero(quadleg->num_velocities());
  // // positions[5] = 55.5;
  // quadleg->SetVelocities(&plant_context, velocities);
  std::cout << "Mass of the quadruped is: " << quadruped->CalcTotalMass(plant_context) << "\n";

  // Set initial location (rpy,xyz) of floating body for multibodyplant
  systems::State<double>& plant_state =
      diagram->GetMutableSubsystemState(*quadruped, diagram_context.get());
  const math::RigidTransform<double> X_WB(
      // math::RollPitchYaw<double>(Vector3<double>{0.0,3.14,0.0}),
      Vector3<double>{0.0, 0.0, FLAGS_initial_height});
  quadruped->SetFreeBodyPose(
      plant_context, &plant_state, quadruped->GetBodyByName("base"), X_WB); 

  // Initialize and run the simulation
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(FLAGS_simulation_realtime_rate);
  simulator.Initialize();
  simulator.get_mutable_integrator().set_target_accuracy(5e-5);
  std::cout << "Simulator is intialized and I am ready to advance it!\n";
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
