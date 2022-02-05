/// @file
///
/// This demo sets up a passive Quadrotor plant in a world described by the
/// warehouse model. The robot simply passively falls to the floor within the
/// walls of the warehouse, falling from the initial_height command line
/// argument.

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator.h"
// #include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace examples {
namespace quadleg {
namespace {

DEFINE_double(duration, 0.5, "Total duration of simulation.");
DEFINE_double(initial_height, 1.051, "Initial height of the QuadLeg.");

template <typename T>
class QuadLeg : public systems::Diagram<T> {
 public:
  QuadLeg() {
    this->set_name("QuadLeg");

    systems::DiagramBuilder<T> builder;
    auto [plant, scene_graph] = multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);
    geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph);
    multibody::Parser parser(&plant);

    parser.AddModelFromFile("/home/mrako/Documents/EMBIR/cpp_sim_intro/models/urdf/leg.urdf", "leg");
    // parser.AddModelFromFile("/../models/urdf/leg.urdf", "leg");
    // parser.AddModelFromFile(
    //     FindResourceOrThrow("drake/examples/quadrotor/warehouse.sdf"));
    plant.Finalize();

    DRAKE_DEMAND(plant.num_actuators() == 0+2);
    DRAKE_DEMAND(plant.num_positions() == 7+2);

    std::cout << "num_positions() --> " << plant.num_positions() << "\n";
    // boost::filesystem::path getcwd = full_path(boost::filesystem::current_path());
    // std::cout << "Current path is : " << boost::filesystem::current_path() << std::endl;
    // std::cout << "current directory --> " << std::filesystem::current_path() << "\n";

    auto input_source = builder.AddSystem<systems::ConstantVectorSource<double>>(systems::BasicVector({0.0,0.0}));


    builder.Connect(input_source.get_output_port(0), plant.get_input_port(0)); // systems::BasicVector({0.0, 0.0})

    builder.BuildInto(this);
    plant_ = &plant;
  }

//   void SetDefaultState(const systems::Context<T>& context,
//                        systems::State<T>* state) const override {
//     DRAKE_DEMAND(state != nullptr);
//     systems::Diagram<T>::SetDefaultState(context, state);
//     const systems::Context<T>& plant_context =
//         this->GetSubsystemContext(*plant_, context);
//     systems::State<T>& plant_state =
//         this->GetMutableSubsystemState(*plant_, state);
//     const math::RigidTransform<T> X_WB(
//         Vector3<T>{0.0, 0.0, FLAGS_initial_height});
//     plant_->SetFreeBodyPose(
//         plant_context, &plant_state, plant_->GetBodyByName("base_link"), X_WB);
//   }

 private:
  multibody::MultibodyPlant<T>* plant_{};
};

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  const QuadLeg<double> model;
  auto model_context = model.CreateDefaultContext();

//   systems::DiagramBuilder<double> builder;
//   builder.AddSystem(model);



  auto simulator = systems::Simulator(model);
  simulator.Initialize();
  simulator.set_target_realtime_rate(0.2);
  simulator.AdvanceTo(FLAGS_duration);

//   auto simulator = MakeSimulatorFromGflags(model);
//   simulator->set_target_realtime_rate(0.2);
//   simulator->AdvanceTo(FLAGS_duration);
  return 0;
}

}  // namespace
}  // namespace quadleg
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::quadleg::do_main(argc, argv);
}
