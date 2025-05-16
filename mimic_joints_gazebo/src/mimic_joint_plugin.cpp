#include "mimic_joints_gazebo/mimic_joint_plugin.hpp"
#include <gazebo/common/Console.hh>
#include <ignition/math/Helpers.hh>

namespace gazebo
{

void MimicJointPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  model_ = _parent;
  world_ = model_->GetWorld();

  if (!model_)
  {
    gzerr << "[MimicJointPlugin] Model pointer is null!\n";
    return;
  }

  if (!_sdf->HasElement("joint") || !_sdf->HasElement("mimicJoint"))
  {
    gzerr << "[MimicJointPlugin] <joint> or <mimicJoint> not specified in SDF.\n";
    return;
  }

  joint_name_ = _sdf->Get<std::string>("joint");
  mimic_joint_name_ = _sdf->Get<std::string>("mimicJoint");

  multiplier_ = _sdf->Get<double>("multiplier", 1.0).first;
  offset_ = _sdf->Get<double>("offset", 0.0).first;
  sensitiveness_ = _sdf->Get<double>("sensitiveness", 0.0).first;
  max_effort_ = _sdf->Get<double>("maxEffort", 1.0).first;

  joint_ = model_->GetJoint(joint_name_);
  mimic_joint_ = model_->GetJoint(mimic_joint_name_);

  if (!joint_ || !mimic_joint_)
  {
    gzerr << "[MimicJointPlugin] Joint(s) not found: " << joint_name_ << " or " << mimic_joint_name_ << "\n";
    return;
  }

#if GAZEBO_MAJOR_VERSION > 2
  mimic_joint_->SetParam("fmax", 0, max_effort_);
#else
  mimic_joint_->SetMaxForce(0, max_effort_);
#endif

  updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&MimicJointPlugin::UpdateChild, this));
}

void MimicJointPlugin::UpdateChild()
{
  double desired = joint_->Position(0) * multiplier_ + offset_;
  double current = mimic_joint_->Position(0);

  if (std::abs(desired - current) >= sensitiveness_)
  {
#if GAZEBO_MAJOR_VERSION >= 4
    mimic_joint_->SetPosition(0, desired);
#else
    mimic_joint_->SetAngle(0, desired);
#endif
  }
}

GZ_REGISTER_MODEL_PLUGIN(MimicJointPlugin)

}  // namespace gazebo
