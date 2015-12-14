/*
 * magnetic_joint.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
  class MagneticJoint : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){
      this->model = _parent;
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&MagneticJointPlugin::OnUpdate, this , _1));
    }

    void OnUpdate(const common::UpdateInfo &_info){
      
    }

  private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
  };

  GZ_REGISTER_MODEL_PLUGIN(MagneticJoint)
}
