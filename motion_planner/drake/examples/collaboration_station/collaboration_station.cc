#include "drake/examples/collaboration_station/collaboration_station.h"

#include <list>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/find_resource.h"
#include "drake/geometry/render/render_engine_vtk_factory.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/discrete_derivative.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/sensors/rgbd_sensor.h"

namespace drake {
namespace examples {
namespace collaboration_station {

using Eigen::Vector3d;
using Eigen::VectorXd;
using geometry::SceneGraph;
using geometry::render::MakeRenderEngineVtk;
using geometry::render::RenderEngineVtkParams;
using math::RigidTransform;
using math::RigidTransformd;
using math::RollPitchYaw;
using math::RotationMatrix;
using multibody::Joint;
using multibody::MultibodyPlant;
using multibody::PrismaticJoint;
using multibody::RevoluteJoint;
using multibody::SpatialInertia;

namespace internal {

// TODO(amcastro-tri): Refactor this into schunk_wsg directory, and cover it
// with a unit test.  Potentially tighten the tolerance in
// station_simulation_test.
// @param gripper_body_frame_name Name of a frame that's attached to the
// gripper's main body.
SpatialInertia<double> MakeCompositeGripperInertia(
    const std::string& wsg_sdf_path,
    const std::string& gripper_body_frame_name) {
  MultibodyPlant<double> plant(0.0);
  multibody::Parser parser(&plant);
  parser.AddModelFromFile(wsg_sdf_path);
  plant.Finalize();
  const auto& frame = plant.GetFrameByName(gripper_body_frame_name);
  const auto& gripper_body = plant.GetRigidBodyByName(frame.body().name());
  const auto& left_finger = plant.GetRigidBodyByName("left_finger");
  const auto& right_finger = plant.GetRigidBodyByName("right_finger");
  const auto& left_slider = plant.GetJointByName("left_finger_sliding_joint");
  const auto& right_slider = plant.GetJointByName("right_finger_sliding_joint");
  const SpatialInertia<double>& M_GGo_G =
      gripper_body.default_spatial_inertia();
  const SpatialInertia<double>& M_LLo_L = left_finger.default_spatial_inertia();
  const SpatialInertia<double>& M_RRo_R =
      right_finger.default_spatial_inertia();
  auto CalcFingerPoseInGripperFrame = [](const Joint<double>& slider) {
    // Pose of the joint's parent frame P (attached on gripper body G) in the
    // frame of the gripper G.
    const RigidTransform<double> X_GP(
        slider.frame_on_parent().GetFixedPoseInBodyFrame());
    // Pose of the joint's child frame C (attached on the slider's finger body)
    // in the frame of the slider's finger F.
    const RigidTransform<double> X_FC(
        slider.frame_on_child().GetFixedPoseInBodyFrame());
    // When the slider's translational dof is zero, then P coincides with C.
    // Therefore:
    const RigidTransform<double> X_GF = X_GP * X_FC.inverse();
    return X_GF;
  };
  // Pose of left finger L in gripper frame G when the slider's dof is zero.
  const RigidTransform<double> X_GL(CalcFingerPoseInGripperFrame(left_slider));
  // Pose of right finger R in gripper frame G when the slider's dof is zero.
  const RigidTransform<double> X_GR(CalcFingerPoseInGripperFrame(right_slider));
  // Helper to compute the spatial inertia of a finger F in about the gripper's
  // origin Go, expressed in G.
  auto CalcFingerSpatialInertiaInGripperFrame =
      [](const SpatialInertia<double>& M_FFo_F,
         const RigidTransform<double>& X_GF) {
        const auto M_FFo_G = M_FFo_F.ReExpress(X_GF.rotation());
        const auto p_FoGo_G = -X_GF.translation();
        const auto M_FGo_G = M_FFo_G.Shift(p_FoGo_G);
        return M_FGo_G;
      };
  // Shift and re-express in G frame the finger's spatial inertias.
  const auto M_LGo_G = CalcFingerSpatialInertiaInGripperFrame(M_LLo_L, X_GL);
  const auto M_RGo_G = CalcFingerSpatialInertiaInGripperFrame(M_RRo_R, X_GR);
  // With everything about the same point Go and expressed in the same frame G,
  // proceed to compose into composite body C:
  // TODO(amcastro-tri): Implement operator+() in SpatialInertia.
  SpatialInertia<double> M_CGo_G = M_GGo_G;
  M_CGo_G += M_LGo_G;
  M_CGo_G += M_RGo_G;
  return M_CGo_G;
}

// TODO(russt): Get these from SDF instead of having them hard-coded (#10022).
void get_camera_poses(std::map<std::string, RigidTransform<double>>* pose_map) {
  pose_map->emplace("0", RigidTransform<double>(
                             RollPitchYaw<double>(2.549607, 1.357609, 2.971679),
                             Vector3d(-0.228895, -0.452176, 0.486308)));

  pose_map->emplace("1",
                    RigidTransform<double>(
                        RollPitchYaw<double>(2.617427, -1.336404, -0.170522),
                        Vector3d(-0.201813, 0.469259, 0.417045)));

  pose_map->emplace("2",
                    RigidTransform<double>(
                        RollPitchYaw<double>(-2.608978, 0.022298, 1.538460),
                        Vector3d(0.786258, -0.048422, 1.043315)));
}

// Load a SDF model and weld it to the MultibodyPlant.
// @param model_path Full path to the sdf model file. i.e. with
// FindResourceOrThrow
// @param model_name Name of the added model instance.
// @param parent Frame P from the MultibodyPlant to which the new model is
// welded to.
// @param child_frame_name Defines frame C (the child frame), assumed to be
// present in the model being added.
// @param X_PC Transformation of frame C relative to frame P.
template <typename T>
multibody::ModelInstanceIndex AddAndWeldModelFrom(
    const std::string& model_path, const std::string& model_name,
    const multibody::Frame<T>& parent, const std::string& child_frame_name,
    const RigidTransform<double>& X_PC, MultibodyPlant<T>* plant) {
  DRAKE_THROW_UNLESS(!plant->HasModelInstanceNamed(model_name));

  multibody::Parser parser(plant);
  const multibody::ModelInstanceIndex new_model =
      parser.AddModelFromFile(model_path, model_name);
  const auto& child_frame = plant->GetFrameByName(child_frame_name, new_model);
  plant->WeldFrames(parent, child_frame, X_PC);
  return new_model;
}

}  // namespace internal

template <typename T>
CollaborationStation<T>::CollaborationStation(double time_step)
    : owned_plant_(std::make_unique<MultibodyPlant<T>>(time_step)),
      owned_scene_graph_(std::make_unique<SceneGraph<T>>()),
      // Given the controller does not compute accelerations, it is irrelevant
      // whether the plant is continuous or discrete. We arbitrarily make it
      // continuous.
      owned_controller_plant_(std::make_unique<MultibodyPlant<T>>(0.0)) {
  // This class holds the unique_ptrs explicitly for plant and scene_graph
  // until Finalize() is called (when they are moved into the Diagram). Grab
  // the raw pointers, which should stay valid for the lifetime of the Diagram.
  plant_ = owned_plant_.get();
  scene_graph_ = owned_scene_graph_.get();
  plant_->RegisterAsSourceForSceneGraph(scene_graph_);
  scene_graph_->set_name("scene_graph");
  plant_->set_name("plant");

  this->set_name("collaboration_station");
}

template <typename T>
void CollaborationStation<T>::AddManipulandFromFile(
    const std::string& model_file, const RigidTransform<double>& X_WObject) {
  multibody::Parser parser(plant_);
  const auto model_index =
      parser.AddModelFromFile(FindResourceOrThrow(model_file));
  const auto indices = plant_->GetBodyIndices(model_index);
  // Only support single-body objects for now.
  // Note: this could be generalized fairly easily... would just want to
  // set default/random positions for the non-floating-base elements below.
  DRAKE_DEMAND(indices.size() == 1);
  object_ids_.push_back(indices[0]);

  object_poses_.push_back(X_WObject);
}


template <typename T>
void CollaborationStation<T>::SetupHomeEnvironmentV1(
    IiwaCollisionModel iiwa_collision_model, AthenaCassieCollisionModel athena_cassie_collision_model) {

  // Add the table and 80/20 workcell frame.
  {
    // const double dx_table_center_to_robot_base = 0.3257;
    const double dz_table_top_robot_base = 0.0127;
    const std::string sdf_path = FindResourceOrThrow(
        "drake/examples/collaboration_station/models/"
        "home_environment_v1.sdf");

    RigidTransform<double> X_WT(
        Vector3d(0, 0, -dz_table_top_robot_base));
    internal::AddAndWeldModelFrom(sdf_path, "table", plant_->world_frame(),
                                  "floor_1", X_WT, plant_);
  }

  // Add the cupboard.
  {
    const std::string sdf_path = FindResourceOrThrow(
        "drake/examples/collaboration_station/models/cupboard.sdf");

    RigidTransform<double> X_WC(
        RotationMatrix<double>::MakeZRotation(0),
        Vector3d(1.75, 0.5, 0.75));
    internal::AddAndWeldModelFrom(sdf_path, "cupboard", plant_->world_frame(),
                                  "cupboard_body", X_WC, plant_);
  }

  // Add the default iiwa/wsg models.
  AddDefaultIiwa(iiwa_collision_model);
  AddDefaultWsg();
  AddDefaultAthenaCassie(athena_cassie_collision_model);

  // Add default cameras.
  {
    std::map<std::string, RigidTransform<double>> camera_poses;
    internal::get_camera_poses(&camera_poses);
    // Typical D415 intrinsics for 848 x 480 resolution, note that rgb and
    // depth are slightly different. And we are not able to model that at the
    // moment.
    // RGB:
    // - w: 848, h: 480, fx: 616.285, fy: 615.778, ppx: 405.418, ppy: 232.864
    // DEPTH:
    // - w: 848, h: 480, fx: 645.138, fy: 645.138, ppx: 420.789, ppy: 239.13
    // For this camera, we are going to assume that fx = fy, and we can compute
    // fov_y by: fy = height / 2 / tan(fov_y / 2)
    const double kFocalY = 645.;
    const int kHeight = 480;
    const int kWidth = 848;
    const double fov_y = std::atan(kHeight / 2. / kFocalY) * 2;
    geometry::render::DepthCameraProperties camera_properties(
        kWidth, kHeight, fov_y, default_renderer_name_, 0.1, 2.0);
    for (const auto& camera_pair : camera_poses) {
      RegisterRgbdSensor(camera_pair.first, plant_->world_frame(),
                         camera_pair.second, camera_properties);
    }
  }
}

template <typename T>
int CollaborationStation<T>::num_iiwa_joints() const {
  DRAKE_DEMAND(iiwa_model_.model_instance.is_valid());
  return plant_->num_positions(iiwa_model_.model_instance);
}

template <typename T>
int CollaborationStation<T>::num_athena_cassie_joints() const {
  DRAKE_DEMAND(athena_cassie_model_.model_instance.is_valid());
  return plant_->num_positions(athena_cassie_model_.model_instance);
}

template <typename T>
void CollaborationStation<T>::SetDefaultState(
    const systems::Context<T>& station_context,
    systems::State<T>* state) const {
  // Call the base class method, to initialize all systems in this diagram.
  systems::Diagram<T>::SetDefaultState(station_context, state);

  T q0_gripper{0.1};

  const auto& plant_context =
      this->GetSubsystemContext(*plant_, station_context);
  auto& plant_state = this->GetMutableSubsystemState(*plant_, state);

  DRAKE_DEMAND(object_ids_.size() == object_poses_.size());

  for (uint64_t i = 0; i < object_ids_.size(); i++) {
    plant_->SetFreeBodyPose(plant_context, &plant_state,
                            plant_->get_body(object_ids_[i]), object_poses_[i]);
  }

  // Use SetIiwaPosition to make sure the controller state is initialized to
  // the IIWA state.
  SetIiwaPosition(station_context, state, GetIiwaPosition(station_context));
  SetIiwaVelocity(station_context, state, VectorX<T>::Zero(num_iiwa_joints()));
  SetWsgPosition(station_context, state, q0_gripper);
  SetWsgVelocity(station_context, state, 0);
  SetAthenaCassiePosition(station_context, state, GetAthenaCassiePosition(station_context));
  SetAthenaCassieVelocity(station_context, state, VectorX<T>::Zero(num_athena_cassie_joints()));
}

template <typename T>
void CollaborationStation<T>::MakeIiwaControllerModel() {
  // Build the controller's version of the plant, which only contains the
  // IIWA and the equivalent inertia of the gripper.
  multibody::Parser parser(owned_controller_plant_.get());
  const auto controller_iiwa_model =
      parser.AddModelFromFile(iiwa_model_.model_path, "iiwa");

  owned_controller_plant_->WeldFrames(
      owned_controller_plant_->world_frame(),
      owned_controller_plant_->GetFrameByName(iiwa_model_.child_frame->name(),
                                              controller_iiwa_model),
      iiwa_model_.X_PC);
  // Add a single body to represent the IIWA pendant's calibration of the
  // gripper.  The body of the WSG accounts for >90% of the total mass
  // (according to the sdf)... and we don't believe our inertia calibration
  // on the hardware to be so precise, so we simply ignore the inertia
  // contribution from the fingers here.
  const multibody::RigidBody<T>& wsg_equivalent =
      owned_controller_plant_->AddRigidBody(
          "wsg_equivalent", controller_iiwa_model,
          internal::MakeCompositeGripperInertia(
              wsg_model_.model_path, wsg_model_.child_frame->name()));

  // TODO(siyuan.feng@tri.global): when we handle multiple IIWA and WSG, this
  // part need to deal with the parent's (iiwa's) model instance id.
  owned_controller_plant_->WeldFrames(
      owned_controller_plant_->GetFrameByName(wsg_model_.parent_frame->name(),
                                              controller_iiwa_model),
      wsg_equivalent.body_frame(), wsg_model_.X_PC);
  owned_controller_plant_->set_name("controller_plant");
}

template <typename T>
void CollaborationStation<T>::MakeAthenaCassieControllerModel() {
  // Build the controller's version of the plant, which only contains the
  // CASSIE and the equivalent inertia of the gripper.
  multibody::Parser parser(owned_controller_plant_.get());
  const auto controller_athena_cassie_model =
      parser.AddModelFromFile(athena_cassie_model_.model_path, "athena_cassie");

  owned_controller_plant_->WeldFrames(
      owned_controller_plant_->world_frame(),
      owned_controller_plant_->GetFrameByName(athena_cassie_model_.child_frame->name(),
                                              controller_athena_cassie_model),
      athena_cassie_model_.X_PC);
  // Add a single body to represent the IIWA pendant's calibration of the
  // gripper.  The body of the WSG accounts for >90% of the total mass
  // (according to the sdf)... and we don't believe our inertia calibration
  // on the hardware to be so precise, so we simply ignore the inertia
  // contribution from the fingers here.

  // !!!SAVE THIS FOR UPPER BODY!!!
  // const multibody::RigidBody<T>& wsg_equivalent =
  //     owned_controller_plant_->AddRigidBody(
  //         "wsg_equivalent", controller_cassie_model,
  //         internal::MakeCompositeGripperInertia(
  //             wsg_model_.model_path, wsg_model_.child_frame->name()));

  // TODO(siyuan.feng@tri.global): when we handle multiple IIWA and WSG, this
  // part need to deal with the parent's (iiwa's) model instance id.

  // !!!SAVE THIS FOR UPPER BODY!!!
  // owned_controller_plant_->WeldFrames(
  //     owned_controller_plant_->GetFrameByName(wsg_model_.parent_frame->name(),
  //                                             controller_cassie_model),
  //     wsg_equivalent.body_frame(), wsg_model_.X_PC);
  // owned_controller_plant_->set_name("controller_plant");
}


template <typename T>
void CollaborationStation<T>::Finalize() {
  Finalize({});
}

template <typename T>
void CollaborationStation<T>::Finalize(
    std::map<std::string, std::unique_ptr<geometry::render::RenderEngine>>
        render_engines) {
  DRAKE_THROW_UNLESS(iiwa_model_.model_instance.is_valid());
  DRAKE_THROW_UNLESS(wsg_model_.model_instance.is_valid());
  DRAKE_THROW_UNLESS(athena_cassie_model_.model_instance.is_valid());

  MakeIiwaControllerModel();
  // MakeCassieControllerModel(); //May be uneccessary for now

  // Note: This deferred diagram construction method/workflow exists because we
  //   - cannot finalize plant until all of my objects are added, and
  //   - cannot wire up my diagram until we have finalized the plant.
  plant_->Finalize();

  // Set plant properties that must occur after finalizing the plant.
  VectorX<T> q0_iiwa(num_iiwa_joints());
  VectorX<T> q0_athena_cassie(num_athena_cassie_joints());

  // Set the initial positions of the IIWA to a comfortable configuration
  // inside the workspace of the station.
  q0_iiwa << 0, 0.6, 0, -1.75, 0, 1.0, 0;

  //TODO: Insert 0 position for cassie
  // q0_cassie << 

  drake::log()->info("homeenv plant pos: {}", plant_->num_positions());

  std::uniform_real_distribution<symbolic::Expression> x(0.4, 0.65),
      y(-0.35, 0.35), z(0, 0.05);
  const Vector3<symbolic::Expression> xyz{x(), y(), z()};
  for (const auto& body_index : object_ids_) {
    const multibody::Body<T>& body = plant_->get_body(body_index);
    plant_->SetFreeBodyRandomPositionDistribution(body, xyz);
    plant_->SetFreeBodyRandomRotationDistributionToUniform(body);
  }
  

  // Set the iiwa default configuration.
  const auto iiwa_joint_indices =
      plant_->GetJointIndices(iiwa_model_.model_instance);
  int q0_index = 0;
  for (const auto joint_index : iiwa_joint_indices) {
    multibody::RevoluteJoint<T>* joint =
        dynamic_cast<multibody::RevoluteJoint<T>*>(
            &plant_->get_mutable_joint(joint_index));
    // Note: iiwa_joint_indices includes the WeldJoint at the base.  Only set
    // the RevoluteJoints.
    if (joint) {
      joint->set_default_angle(q0_iiwa[q0_index++]);
    }
  }

  // // Set the cassie default configuration.
  // const auto cassie_joint_indices =
  //     plant_->GetJointIndices(cassie_model_.model_instance);
  // int q0_cassie_index = 0;
  // for (const auto joint_index : cassie_joint_indices) {
  //   multibody::RevoluteJoint<T>* joint =
  //       dynamic_cast<multibody::RevoluteJoint<T>*>(
  //           &plant_->get_mutable_joint(joint_index));
  //   // Note: iiwa_joint_indices includes the WeldJoint at the base.  Only set
  //   // the RevoluteJoints.
  //   if (joint) {
  //     joint->set_default_angle(q0_cassie[q0_cassie_index++]);
  //   }
  // }

  systems::DiagramBuilder<T> builder;

  builder.AddSystem(std::move(owned_plant_));
  builder.AddSystem(std::move(owned_scene_graph_));

  builder.Connect(
      plant_->get_geometry_poses_output_port(),
      scene_graph_->get_source_pose_port(plant_->get_source_id().value()));
  builder.Connect(scene_graph_->get_query_output_port(),
                  plant_->get_geometry_query_input_port());

  //IIWA
  const int num_iiwa_positions =
      plant_->num_positions(iiwa_model_.model_instance);
  DRAKE_THROW_UNLESS(num_iiwa_positions ==
                     plant_->num_velocities(iiwa_model_.model_instance));
  // Export the commanded positions via a PassThrough.
  auto iiwa_position =
      builder.template AddSystem<systems::PassThrough>(num_iiwa_positions);
  builder.ExportInput(iiwa_position->get_input_port(), "iiwa_position");
  builder.ExportOutput(iiwa_position->get_output_port(),
                       "iiwa_position_commanded");

  //CASSIE
  // const int num_cassie_positions =
  //     plant_->num_positions(cassie_model_.model_instance);
  // DRAKE_THROW_UNLESS(num_cassie_positions ==
  //                    plant_->num_velocities(cassie_model_.model_instance));
  // // Export the commanded positions via a PassThrough.
  // auto cassie_position =
  //     builder.template AddSystem<systems::PassThrough>(num_cassie_positions);
  // builder.ExportInput(cassie_position->get_input_port(), "cassie_position");
  // builder.ExportOutput(cassie_position->get_output_port(),
  //                      "cassie_position_commanded");

  // Export iiwa "state" outputs.
  {
    auto demux = builder.template AddSystem<systems::Demultiplexer>(
        2 * num_iiwa_positions, num_iiwa_positions);
    builder.Connect(plant_->get_state_output_port(iiwa_model_.model_instance),
                    demux->get_input_port(0));
    builder.ExportOutput(demux->get_output_port(0), "iiwa_position_measured");
    builder.ExportOutput(demux->get_output_port(1), "iiwa_velocity_estimated");

    builder.ExportOutput(
        plant_->get_state_output_port(iiwa_model_.model_instance),
        "iiwa_state_estimated");
  }

  // // Export cassie "state" outputs.
  // {
  //   auto demux = builder.template AddSystem<systems::Demultiplexer>(
  //       2 * num_cassie_positions, num_cassie_positions);
  //   builder.Connect(plant_->get_state_output_port(cassie_model_.model_instance),
  //                   demux->get_input_port(0));
  //   builder.ExportOutput(demux->get_output_port(0), "cassie_position_measured");
  //   builder.ExportOutput(demux->get_output_port(1), "cassie_velocity_estimated");

  //   builder.ExportOutput(
  //       plant_->get_state_output_port(cassie_model_.model_instance),
  //       "cassie_state_estimated");
  // }

  // Add the IIWA controller "stack".
  {
    owned_controller_plant_->Finalize();

    auto check_gains = [](const VectorX<double>& gains, int size) {
      return (gains.size() == size) && (gains.array() >= 0).all();
    };

    // Set default gains if.
    if (iiwa_kp_.size() == 0) {
      iiwa_kp_ = VectorXd::Constant(num_iiwa_positions, 100);
    }
    // if (cassie_kp_.size() == 0){
    //   cassie_kp_ = VectorXd::Constant(num_cassie_positions, 100);
    // }

    DRAKE_THROW_UNLESS(check_gains(iiwa_kp_, num_iiwa_positions));
    // DRAKE_THROW_UNLESS(check_gains(cassie_kp_, num_cassie_positions));

    if (iiwa_kd_.size() == 0) {
      iiwa_kd_.resize(num_iiwa_positions);
      for (int i = 0; i < num_iiwa_positions; i++) {
        // Critical damping gains.
        iiwa_kd_[i] = 2 * std::sqrt(iiwa_kp_[i]);
      }
    }
    // if (cassie_kd_.size() == 0) {
    //   cassie_kd_.resize(num_cassie_positions);
    //   for (int i = 0; i < num_cassie_positions; i++) {
    //     // Critical damping gains.
    //     cassie_kd_[i] = 2 * std::sqrt(cassie_kp_[i]);
    //   }
    // }

    DRAKE_THROW_UNLESS(check_gains(iiwa_kd_, num_iiwa_positions));
    // DRAKE_THROW_UNLESS(check_gains(cassie_kd_, num_cassie_positions));

    if (iiwa_ki_.size() == 0) {
      iiwa_ki_ = VectorXd::Constant(num_iiwa_positions, 1);
    }
    // if (cassie_ki_.size() == 0) {
    //   cassie_ki_ = VectorXd::Constant(num_cassie_positions, 1);
    // }

    DRAKE_THROW_UNLESS(check_gains(iiwa_ki_, num_iiwa_positions));
    // DRAKE_THROW_UNLESS(check_gains(cassie_ki_, num_cassie_positions));

    drake::log()->info("plant pos: {}", owned_controller_plant_->num_positions());

    // Add the inverse dynamics controller.
    auto iiwa_controller = builder.template AddSystem<
        systems::controllers::InverseDynamicsController>(
        *owned_controller_plant_, iiwa_kp_, iiwa_ki_, iiwa_kd_,
        false, iiwa_model_.model_instance);

    drake::log()->info("past id iiwa_controller");

    iiwa_controller->set_name("iiwa_controller");
    builder.Connect(plant_->get_state_output_port(iiwa_model_.model_instance),
                    iiwa_controller->get_input_port_estimated_state());

    // // Add the inverse dynamics controller.
    // auto cassie_controller = builder.template AddSystem<
    //     systems::controllers::InverseDynamicsController>(
    //     *owned_controller_plant_, cassie_kp_, cassie_ki_, cassie_kd_, 
    //     false, cassie_model_.model_instance);
    // cassie_controller->set_name("cassie_controller");
    // builder.Connect(plant_->get_state_output_port(cassie_model_.model_instance),
    //                 cassie_controller->get_input_port_estimated_state());

    drake::log()->info("past inverse dynamics");

    // Add in feedforward torque.
    auto iiwa_adder =
        builder.template AddSystem<systems::Adder>(2, num_iiwa_positions);
    builder.Connect(iiwa_controller->get_output_port_control(),
                    iiwa_adder->get_input_port(0));
    builder.ExportInput(iiwa_adder->get_input_port(1), "iiwa_feedforward_torque");
    builder.Connect(iiwa_adder->get_output_port(), plant_->get_actuation_input_port(
                                                  iiwa_model_.model_instance));

    // // Add in feedforward torque.
    // auto cassie_adder =
    //     builder.template AddSystem<systems::Adder>(2, num_cassie_positions);
    // builder.Connect(cassie_controller->get_output_port_control(),
    //                 cassie_adder->get_input_port(0));
    // builder.ExportInput(cassie_adder->get_input_port(1), "cassie_feedforward_torque");
    // builder.Connect(cassie_adder->get_output_port(), plant_->get_actuation_input_port(
    //                                               cassie_model_.model_instance));

    drake::log()->info("past ff torque");


    // Approximate desired state command from a discrete derivative of the
    // position command input port.
    auto desired_iiwa_state_from_position = builder.template AddSystem<
        systems::StateInterpolatorWithDiscreteDerivative>(
            num_iiwa_positions, plant_->time_step(),
            true /* suppress_initial_transient */);
    desired_iiwa_state_from_position->set_name("desired_iiwa_state_from_position");
    builder.Connect(desired_iiwa_state_from_position->get_output_port(),
                    iiwa_controller->get_input_port_desired_state());
    builder.Connect(iiwa_position->get_output_port(),
                    desired_iiwa_state_from_position->get_input_port());

    // // Approximate desired state command from a discrete derivative of the
    // // position command input port.
    // auto desired_cassie_state_from_position = builder.template AddSystem<
    //     systems::StateInterpolatorWithDiscreteDerivative>(
    //         num_cassie_positions, plant_->time_step(),
    //         true /* suppress_initial_transient */);
    // desired_cassie_state_from_position->set_name("desired_cassie_state_from_position");
    // builder.Connect(desired_cassie_state_from_position->get_output_port(),
    //                 cassie_controller->get_input_port_desired_state());
    // builder.Connect(cassie_position->get_output_port(),
    //                 desired_cassie_state_from_position->get_input_port());

    // Export commanded torques:
    builder.ExportOutput(iiwa_adder->get_output_port(), "iiwa_torque_commanded");
    builder.ExportOutput(iiwa_adder->get_output_port(), "iiwa_torque_measured");

    // // Export commanded torques:
    // builder.ExportOutput(cassie_adder->get_output_port(), "cassie_torque_commanded");
    // builder.ExportOutput(cassie_adder->get_output_port(), "cassie_torque_measured");
  }

  {
    auto wsg_controller = builder.template AddSystem<
        manipulation::schunk_wsg::SchunkWsgPositionController>(
        manipulation::schunk_wsg::kSchunkWsgLcmStatusPeriod, wsg_kp_, wsg_kd_);
    wsg_controller->set_name("wsg_controller");

    builder.Connect(
        wsg_controller->get_generalized_force_output_port(),
        plant_->get_actuation_input_port(wsg_model_.model_instance));
    builder.Connect(plant_->get_state_output_port(wsg_model_.model_instance),
                    wsg_controller->get_state_input_port());

    builder.ExportInput(wsg_controller->get_desired_position_input_port(),
                        "wsg_position");
    builder.ExportInput(wsg_controller->get_force_limit_input_port(),
                        "wsg_force_limit");

    auto wsg_mbp_state_to_wsg_state = builder.template AddSystem(
        manipulation::schunk_wsg::MakeMultibodyStateToWsgStateSystem<double>());
    builder.Connect(plant_->get_state_output_port(wsg_model_.model_instance),
                    wsg_mbp_state_to_wsg_state->get_input_port());

    builder.ExportOutput(wsg_mbp_state_to_wsg_state->get_output_port(),
                         "wsg_state_measured");

    builder.ExportOutput(wsg_controller->get_grip_force_output_port(),
                         "wsg_force_measured");
  }

  //Cassie stuff begins --------
  {


    VectorX<double> constant_vector = VectorX<double>::Zero(
      plant_->num_actuated_dofs(athena_cassie_model_.model_instance));
    
    auto constant_zero_source =
      builder.template AddSystem<systems::ConstantVectorSource<double>>(
      constant_vector);
    constant_zero_source->set_name("zero input");

    // Connects the blank input command
    builder.Connect(constant_zero_source->get_output_port(),
                    plant_->get_actuation_input_port(athena_cassie_model_.model_instance));

  }
  //Cassie stuff ends-----

  builder.ExportOutput(plant_->get_generalized_contact_forces_output_port(
                           iiwa_model_.model_instance),
                       "iiwa_torque_external");

  {  // RGB-D Cameras
    if (render_engines.size() > 0) {
      for (auto& pair : render_engines) {
        scene_graph_->AddRenderer(pair.first, std::move(pair.second));
      }
    } else {
      scene_graph_->AddRenderer(default_renderer_name_,
                                MakeRenderEngineVtk(RenderEngineVtkParams()));
    }

    for (const auto& info_pair : camera_information_) {
      std::string camera_name = "camera_" + info_pair.first;
      const CameraInformation& info = info_pair.second;

      const std::optional<geometry::FrameId> parent_body_id =
          plant_->GetBodyFrameIdIfExists(info.parent_frame->body().index());
      DRAKE_THROW_UNLESS(parent_body_id.has_value());
      const RigidTransform<double> X_PC =
          info.parent_frame->GetFixedPoseInBodyFrame() * info.X_PC;

      auto camera = builder.template AddSystem<systems::sensors::RgbdSensor>(
          parent_body_id.value(), X_PC, info.properties);
      builder.Connect(scene_graph_->get_query_output_port(),
                      camera->query_object_input_port());

      builder.ExportOutput(camera->color_image_output_port(),
                           camera_name + "_rgb_image");
      builder.ExportOutput(camera->depth_image_16U_output_port(),
                           camera_name + "_depth_image");
      builder.ExportOutput(camera->label_image_output_port(),
                           camera_name + "_label_image");
    }
  }

  builder.ExportOutput(scene_graph_->get_pose_bundle_output_port(),
                       "pose_bundle");

  builder.ExportOutput(plant_->get_contact_results_output_port(),
                       "contact_results");
  builder.ExportOutput(plant_->get_state_output_port(),
                       "plant_continuous_state");
  builder.ExportOutput(plant_->get_geometry_poses_output_port(),
                       "geometry_poses");

  builder.BuildInto(this);
}

template <typename T>
VectorX<T> CollaborationStation<T>::GetIiwaPosition(
    const systems::Context<T>& station_context) const {
  const auto& plant_context =
      this->GetSubsystemContext(*plant_, station_context);
  return plant_->GetPositions(plant_context, iiwa_model_.model_instance);
}

template <typename T>
void CollaborationStation<T>::SetIiwaPosition(
    const drake::systems::Context<T>& station_context, systems::State<T>* state,
    const Eigen::Ref<const drake::VectorX<T>>& q) const {
  const int num_iiwa_positions =
      plant_->num_positions(iiwa_model_.model_instance);
  DRAKE_DEMAND(state != nullptr);
  DRAKE_DEMAND(q.size() == num_iiwa_positions);
  auto& plant_context = this->GetSubsystemContext(*plant_, station_context);
  auto& plant_state = this->GetMutableSubsystemState(*plant_, state);
  plant_->SetPositions(plant_context, &plant_state, iiwa_model_.model_instance,
                       q);
}

template <typename T>
VectorX<T> CollaborationStation<T>::GetIiwaVelocity(
    const systems::Context<T>& station_context) const {
  const auto& plant_context =
      this->GetSubsystemContext(*plant_, station_context);
  return plant_->GetVelocities(plant_context, iiwa_model_.model_instance);
}

template <typename T>
void CollaborationStation<T>::SetIiwaVelocity(
    const drake::systems::Context<T>& station_context, systems::State<T>* state,
    const Eigen::Ref<const drake::VectorX<T>>& v) const {
  const int num_iiwa_velocities =
      plant_->num_velocities(iiwa_model_.model_instance);
  DRAKE_DEMAND(state != nullptr);
  DRAKE_DEMAND(v.size() == num_iiwa_velocities);
  auto& plant_context = this->GetSubsystemContext(*plant_, station_context);
  auto& plant_state = this->GetMutableSubsystemState(*plant_, state);
  plant_->SetVelocities(plant_context, &plant_state, iiwa_model_.model_instance,
                        v);
}

template <typename T>
T CollaborationStation<T>::GetWsgPosition(
    const systems::Context<T>& station_context) const {
  const auto& plant_context =
      this->GetSubsystemContext(*plant_, station_context);

  Vector2<T> positions =
      plant_->GetPositions(plant_context, wsg_model_.model_instance);
  return positions(1) - positions(0);
}

template <typename T>
T CollaborationStation<T>::GetWsgVelocity(
    const systems::Context<T>& station_context) const {
  const auto& plant_context =
      this->GetSubsystemContext(*plant_, station_context);

  Vector2<T> velocities =
      plant_->GetVelocities(plant_context, wsg_model_.model_instance);
  return velocities(1) - velocities(0);
}

template <typename T>
void CollaborationStation<T>::SetWsgPosition(
    const drake::systems::Context<T>& station_context, systems::State<T>* state,
    const T& q) const {
  DRAKE_DEMAND(state != nullptr);
  auto& plant_context = this->GetSubsystemContext(*plant_, station_context);
  auto& plant_state = this->GetMutableSubsystemState(*plant_, state);

  const Vector2<T> positions(-q / 2, q / 2);
  plant_->SetPositions(plant_context, &plant_state, wsg_model_.model_instance,
                       positions);
}

template <typename T>
void CollaborationStation<T>::SetWsgVelocity(
    const drake::systems::Context<T>& station_context, systems::State<T>* state,
    const T& v) const {
  DRAKE_DEMAND(state != nullptr);
  auto& plant_context = this->GetSubsystemContext(*plant_, station_context);
  auto& plant_state = this->GetMutableSubsystemState(*plant_, state);

  const Vector2<T> velocities(-v / 2, v / 2);
  plant_->SetVelocities(plant_context, &plant_state, wsg_model_.model_instance,
                        velocities);
}

template <typename T>
VectorX<T> CollaborationStation<T>::GetAthenaCassiePosition(
    const systems::Context<T>& station_context) const {
  const auto& plant_context =
      this->GetSubsystemContext(*plant_, station_context);
  return plant_->GetPositions(plant_context, athena_cassie_model_.model_instance);
}

template <typename T>
void CollaborationStation<T>::SetAthenaCassiePosition(
    const drake::systems::Context<T>& station_context, systems::State<T>* state,
    const Eigen::Ref<const drake::VectorX<T>>& q) const {
  const int num_athena_cassie_positions =
      plant_->num_positions(athena_cassie_model_.model_instance);
  DRAKE_DEMAND(state != nullptr);
  DRAKE_DEMAND(q.size() == num_athena_cassie_positions);
  auto& plant_context = this->GetSubsystemContext(*plant_, station_context);
  auto& plant_state = this->GetMutableSubsystemState(*plant_, state);
  plant_->SetPositions(plant_context, &plant_state, athena_cassie_model_.model_instance,
                       q);
}

template <typename T>
VectorX<T> CollaborationStation<T>::GetAthenaCassieVelocity(
    const systems::Context<T>& station_context) const {
  const auto& plant_context =
      this->GetSubsystemContext(*plant_, station_context);
  return plant_->GetVelocities(plant_context, athena_cassie_model_.model_instance);
}

template <typename T>
void CollaborationStation<T>::SetAthenaCassieVelocity(
    const drake::systems::Context<T>& station_context, systems::State<T>* state,
    const Eigen::Ref<const drake::VectorX<T>>& v) const {
  const int num_athena_cassie_velocities =
      plant_->num_velocities(athena_cassie_model_.model_instance);
  DRAKE_DEMAND(state != nullptr);
  DRAKE_DEMAND(v.size() == num_athena_cassie_velocities);
  auto& plant_context = this->GetSubsystemContext(*plant_, station_context);
  auto& plant_state = this->GetMutableSubsystemState(*plant_, state);
  plant_->SetVelocities(plant_context, &plant_state, athena_cassie_model_.model_instance,
                        v);
}

template <typename T>
std::vector<std::string> CollaborationStation<T>::get_camera_names() const {
  std::vector<std::string> names;
  names.reserve(camera_information_.size());
  for (const auto& info : camera_information_) {
    names.emplace_back(info.first);
  }
  return names;
}

template <typename T>
void CollaborationStation<T>::SetWsgGains(const double kp, const double kd) {
  DRAKE_THROW_UNLESS(!plant_->is_finalized());
  DRAKE_THROW_UNLESS(kp >= 0 && kd >= 0);
  wsg_kp_ = kp;
  wsg_kd_ = kd;
}

template <typename T>
void CollaborationStation<T>::RegisterIiwaControllerModel(
    const std::string& model_path,
    const multibody::ModelInstanceIndex iiwa_instance,
    const multibody::Frame<T>& parent_frame,
    const multibody::Frame<T>& child_frame,
    const RigidTransform<double>& X_PC) {
  // TODO(siyuan.feng@tri.global): We really only just need to make sure
  // the parent frame is a AnchoredFrame(i.e. there is a rigid kinematic path
  // from it to the world), and record that X_WP. However, the computation to
  // query X_WP given a partially constructed plant is not feasible at the
  // moment, so we are forcing the parent frame to be the world instead.
  DRAKE_THROW_UNLESS(parent_frame.name() == plant_->world_frame().name());

  iiwa_model_.model_path = model_path;
  iiwa_model_.parent_frame = &parent_frame;
  iiwa_model_.child_frame = &child_frame;
  iiwa_model_.X_PC = X_PC;

  iiwa_model_.model_instance = iiwa_instance;
}

template <typename T>
void CollaborationStation<T>::RegisterWsgControllerModel(
    const std::string& model_path,
    const multibody::ModelInstanceIndex wsg_instance,
    const multibody::Frame<T>& parent_frame,
    const multibody::Frame<T>& child_frame,
    const RigidTransform<double>& X_PC) {
  wsg_model_.model_path = model_path;
  wsg_model_.parent_frame = &parent_frame;
  wsg_model_.child_frame = &child_frame;
  wsg_model_.X_PC = X_PC;

  wsg_model_.model_instance = wsg_instance;
}

template <typename T>
void CollaborationStation<T>::RegisterAthenaCassieControllerModel(
    const std::string& model_path,
    const multibody::ModelInstanceIndex athena_cassie_instance,
    const multibody::Frame<T>& parent_frame,
    const multibody::Frame<T>& child_frame,
    const RigidTransform<double>& X_PC) {
  // TODO(siyuan.feng@tri.global): We really only just need to make sure
  // the parent frame is a AnchoredFrame(i.e. there is a rigid kinematic path
  // from it to the world), and record that X_WP. However, the computation to
  // query X_WP given a partially constructed plant is not feasible at the
  // moment, so we are forcing the parent frame to be the world instead.
  DRAKE_THROW_UNLESS(parent_frame.name() == plant_->world_frame().name());

  athena_cassie_model_.model_path = model_path;
  athena_cassie_model_.parent_frame = &parent_frame;
  athena_cassie_model_.child_frame = &child_frame;
  athena_cassie_model_.X_PC = X_PC;

  athena_cassie_model_.model_instance = athena_cassie_instance;
}

template <typename T>
void CollaborationStation<T>::RegisterRgbdSensor(
    const std::string& name, const multibody::Frame<T>& parent_frame,
    const RigidTransform<double>& X_PC,
    const geometry::render::DepthCameraProperties& properties) {
  CameraInformation info;
  info.parent_frame = &parent_frame;
  info.X_PC = X_PC;
  info.properties = properties;

  camera_information_[name] = info;
}

template <typename T>
std::map<std::string, RigidTransform<double>>
CollaborationStation<T>::GetStaticCameraPosesInWorld() const {
  std::map<std::string, RigidTransform<double>> static_camera_poses;

  for (const auto& info : camera_information_) {
    const auto& frame_P = *info.second.parent_frame;

    // TODO(siyuan.feng@tri.global): We really only just need to make sure
    // the parent frame is a AnchoredFrame(i.e. there is a rigid kinematic path
    // from it to the world). However, the computation to query X_WP given a
    // partially constructed plant is not feasible at the moment, so we are
    // looking for cameras that are directly attached to the world instead.
    const bool is_anchored =
        frame_P.body().index() == plant_->world_frame().body().index();
    if (is_anchored) {
      static_camera_poses.emplace(
          info.first,
          RigidTransform<double>(frame_P.GetFixedPoseInBodyFrame()) *
              info.second.X_PC);
    }
  }

  return static_camera_poses;
}

// Add default iiwa.
template <typename T>
void CollaborationStation<T>::AddDefaultIiwa(
    const IiwaCollisionModel collision_model) {
  std::string sdf_path;
  switch (collision_model) {
    case IiwaCollisionModel::kNoCollision:
      sdf_path = FindResourceOrThrow(
          "drake/manipulation/models/iiwa_description/iiwa7/"
          "iiwa7_no_collision.sdf");
      break;
    case IiwaCollisionModel::kBoxCollision:
      sdf_path = FindResourceOrThrow(
          "drake/manipulation/models/iiwa_description/iiwa7/"
          "iiwa7_with_box_collision.sdf");
      break;
    default:
      throw std::domain_error("Unrecognized collision_model.");
  }
  const RigidTransform<double> X_WI(RollPitchYaw<double>(0, 0, M_PI),
                                    Vector3d(1, 0.5, 0.3));
  auto iiwa_instance = internal::AddAndWeldModelFrom(
      sdf_path, "iiwa", plant_->world_frame(), "iiwa_link_0", X_WI, plant_);
  RegisterIiwaControllerModel(
      sdf_path, iiwa_instance, plant_->world_frame(),
      plant_->GetFrameByName("iiwa_link_0", iiwa_instance), X_WI);
}

// Add default wsg.
template <typename T>
void CollaborationStation<T>::AddDefaultWsg() {
  const std::string sdf_path = FindResourceOrThrow(
      "drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50.sdf");
  const multibody::Frame<T>& link7 =
      plant_->GetFrameByName("iiwa_link_7", iiwa_model_.model_instance);
  const RigidTransform<double> X_7G(RollPitchYaw<double>(M_PI_2, 0, M_PI_2),
                                    Vector3d(0, 0, 0.114));
  auto wsg_instance = internal::AddAndWeldModelFrom(sdf_path, "gripper", link7,
                                                    "body", X_7G, plant_);
  RegisterWsgControllerModel(sdf_path, wsg_instance, link7,
                             plant_->GetFrameByName("body", wsg_instance),
                             X_7G);
}

template <typename T>
void CollaborationStation<T>::AddDefaultAthenaCassie(
    const AthenaCassieCollisionModel collision_model) {
  std::string sdf_path;
  switch (collision_model) {
    case AthenaCassieCollisionModel::kNoCollision:
      sdf_path = FindResourceOrThrow(
          "drake/manipulation/models/athena_cassie_description/urdf/"
          "athena_cassie.urdf");
      break;
    case AthenaCassieCollisionModel::kBoxCollision:
      sdf_path = FindResourceOrThrow(
          "drake/manipulation/models/athena_cassie_description/urdf/"
          "athena_cassie.urdf");
      drake::log()->info("Athena Cassie Box Collision model doesn't exist. Please create a new URDF.");
      DRAKE_DEMAND(false);
      break;
    default:
      throw std::domain_error("Unrecognized collision_model.");
  }
  const auto X_WI = RigidTransform<double>::Identity();
  auto athena_cassie_instance = internal::AddAndWeldModelFrom(
      sdf_path, "athena_cassie", plant_->world_frame(), "pelvis", X_WI, plant_);
  RegisterAthenaCassieControllerModel(
      sdf_path, athena_cassie_instance, plant_->world_frame(),
      plant_->GetFrameByName("pelvis", athena_cassie_instance), X_WI);
}

}  // namespace collaboration_station
}  // namespace examples
}  // namespace drake

// TODO(russt): Support at least NONSYMBOLIC_SCALARS.  See #9573.
//   (and don't forget to include default_scalars.h)
template class ::drake::examples::collaboration_station::CollaborationStation<
    double>;
