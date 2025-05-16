#ifndef GAZEBO_PLUGINS_DISABLE_LINK_PLUGIN_HPP
#define GAZEBO_PLUGINS_DISABLE_LINK_PLUGIN_HPP

#include <boost/bind/bind.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <string>

namespace gazebo
{
  class DisableLinkPlugin : public ModelPlugin
  {
  public:
    DisableLinkPlugin() = default;
    ~DisableLinkPlugin() override = default;

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
    void UpdateChild();

  private:
    std::string link_name_;
    bool kill_sim = false;
    physics::LinkPtr link_;
    physics::ModelPtr model_;
    physics::WorldPtr world_;
  };
}

#endif  // GAZEBO_PLUGINS_DISABLE_LINK_PLUGIN_HPP
