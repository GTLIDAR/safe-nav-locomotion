#include "drake/examples/kuka_iiwa_arm/iiwa.h"


namespace drake {
namespace examples {
namespace kuka_iiwa_arm{

using drake::systems::BasicVector;

  Iiwa::Iiwa()
  {  
    DeclareDiscreteState(7); 

    // When you change this port to the actual number of actuators on the robot, 
    // this class will become useful. Otherwise currently, this is just a passthrough.
    // For more info, check the header file.
    position_measured_input_port_ = 
        DeclareVectorInputPort("iiwa_position_measured", BasicVector<double>(7))
        .get_index();

    position_measured_output_port_ = 
        DeclareVectorOutputPort("iiwa_position_state", BasicVector<double>(7),
        &Iiwa::CopyDiscreteStateOut).get_index();

    DeclarePeriodicDiscreteUpdate(0.001);
  }

  

  void Iiwa::DoCalcDiscreteVariableUpdates(
      const drake::systems::Context<double>& context,
      const std::vector<const drake::systems::DiscreteUpdateEvent<double>*>& ,
      drake::systems::DiscreteValues<double>* updates) const
  {
      // Eigen::VectorXd state(7);

      // state <<  0, 1, 1, 1, 1, 1, 1;
      // updates->get_mutable_vector().SetFromVector(state);


    const auto& input_state = get_position_measured_input_port().Eval(context);
    // drake::log()->info(input_state[0]);
    updates->get_mutable_vector().SetFromVector(input_state);
  
  }

  void Iiwa::CopyDiscreteStateOut(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const 
  {
    auto d_state = context.get_discrete_state().get_vector().CopyToVector();
    output->SetFromVector(d_state);
  }

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
