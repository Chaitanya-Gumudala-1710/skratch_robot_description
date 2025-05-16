#include "mimic_joints_gazebo/disable_link_plugin.hpp"
#include <gazebo/common/Console.hh>

namespace gazebo
{

void DisableLinkPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  model_ = _parent;
  world_ = model_->GetWorld();

  if (!_sdf->HasElement("link"))
  {
    gzerr << "[DisableLinkPlugin] No <link> element in SDF. Plugin will not load.\n";
    return;
  }

  link_name_ = _sdf->Get<std::string>("link");
  link_ = model_->GetLink(link_name_);

  if (link_)
  {
    link_->SetEnabled(false);
    gzdbg << "[DisableLinkPlugin] Link '" << link_name_ << "' disabled.\n";
  }
  else
  {
    gzerr << "[DisableLinkPlugin] Link '" << link_name_ << "' not found!\n";
  }
}

void DisableLinkPlugin::UpdateChild()
{
  // Currently not used
}

GZ_REGISTER_MODEL_PLUGIN(DisableLinkPlugin)

}  // namespace gazebo
