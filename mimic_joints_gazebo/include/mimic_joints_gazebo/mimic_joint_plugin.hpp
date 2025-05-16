#ifndef GAZEBO_PLUGINS_MIMIC_JOINT_PLUGIN_HPP
#define GAZEBO_PLUGINS_MIMIC_JOINT_PLUGIN_HPP

#include <control_toolbox/pid.hpp>
#include <boost/bind/bind.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <string>

namespace gazebo
{
  class MimicJointPlugin : public ModelPlugin
  {
  public:
    MimicJointPlugin() = default;
    ~MimicJointPlugin() override = default;

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;
    void UpdateChild();

  private:
    std::string joint_name_, mimic_joint_name_, robot_namespace_;
    double multiplier_ = 1.0;
    double offset_ = 0.0;
    double sensitiveness_ = 0.0;
    double max_effort_ = 0.0;
    bool has_pid_ = false;
    bool kill_sim = false;

    control_toolbox::Pid pid_;
    physics::JointPtr joint_;
    physics::JointPtr mimic_joint_;
    physics::ModelPtr model_;
    physics::WorldPtr world_;
    event::ConnectionPtr updateConnection;
  };
}

#endif  // GAZEBO_PLUGINS_MIMIC_JOINT_PLUGIN_HPP
