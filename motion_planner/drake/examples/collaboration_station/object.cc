#include "drake/examples/collaboration_station/object.h"

#define PI 3.1416

namespace drake {
namespace examples {
namespace collaboration_station{

using drake::systems::BasicVector;

  Object::Object()
  {  
    DeclareDiscreteState(7); 

    // When you change this port to the actual number of wheels on the robot, 
    // this class will become useful. Otherwise currently, this is just a passthrough.
    position_measured_input_port_ = 
        DeclareVectorInputPort("position_measured", BasicVector<double>(7))
        .get_index();

    position_measured_output_port_ = 
        DeclareVectorOutputPort("floating_base_state", BasicVector<double>(7),
        &Object::CopyDiscreteStateOut).get_index();

    DeclarePeriodicDiscreteUpdate(0.001);

  }

  // Dead function. Not used.
  void Object::InitSystem(
      std::vector<Eigen::Matrix<double, 3, 1>> location_l)
  {
      location_list = location_l;
  }

  void Object::DoCalcDiscreteVariableUpdates(
      const drake::systems::Context<double>& context,
      const std::vector<const drake::systems::DiscreteUpdateEvent<double>*>& ,
      drake::systems::DiscreteValues<double>* updates) const
  {
    const auto& input_state = get_position_measured_input_port().Eval(context);

    // TODO: Get rid of this hack and put a throw statement after ensuring
    //        that position publisher does not send bad quaternion values
    if(input_state[0] == 0 && input_state[1] == 0 && 
        input_state[2] == 0 && input_state[3] == 0)
    {
      drake::log()->info("Object: Either plan finished or Bad quaternion value detected. Setting state to default.");
      Eigen::VectorXd state(7);
      state <<   1, 0, 0, 0, 0, 0, 0;
      updates->get_mutable_vector().SetFromVector(state);
    }
    else
    {
      updates->get_mutable_vector().SetFromVector(input_state);
    }

    // drake::log()->info("input_state size: {}\nqw: {}\nqx: {}\nqy: {}\nqz: {}\nx: {}\ny: {}\nz: {}",
    //                     input_state.size(),input_state[0],input_state[1],input_state[2],
    //                     input_state[3], input_state[4],input_state[5],input_state[6]);

    // drake::log()->info("state updated");

  }

  void Object::CopyDiscreteStateOut(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const 
  {
    auto d_state = context.get_discrete_state().get_vector().CopyToVector();
    output->SetFromVector(d_state);
  }




}  // namespace collaboration_station
}  // namespace examples
}  // namespace drake
