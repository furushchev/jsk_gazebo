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
        boost::bind(&MagneticJoint::OnUpdate, this , _1));
      std::string joint_name;
      try {
        joint_name = GetParam<std::string>(_sdf, "jointName");
      } catch(std::string s){
        gzerr << s;
        return;
      }
      this->joint = _parent->GetJoint(joint_name);
      if (!this->joint){
        gzerr << "joint " << joint_name << " not found\n";
        return;
      }

      double min_angle = this->joint->GetLowerLimit(0).Radian();
      try {
        near_threshold = GetParam<double>(_sdf, "nearThreshold");
      } catch(std::string s){
        gzwarn << s;
        near_threshold = min_angle + 0.157;
      }
      if (min_angle > near_threshold)
        gzerr << "value of nearThreshold must be greater than joint minimum angle " << min_angle << "\n";
      try {
        magnetic_force = GetParam<double>(_sdf, "magneticForce");
      } catch(std::string s){
        gzwarn << s;
        magnetic_force = 0.05;
      }

      this->joint->SetProvideFeedback(true);
      this->joint->SetAngle(0, 0.5);
    }

    template <class T>
    T GetParam(sdf::ElementPtr e, std::string name){
      if (e->HasElement(name)){
        return e->Get<T>(name);
      } else {
        std::stringstream ss;
        ss << "<" << name << "></" << name << "> tag missing\n";
        throw ss.str();
      }
    }

    void OnUpdate(const common::UpdateInfo &_info){
      double angle = (this->joint->GetAngle(0)).Radian();
      if (angle < near_threshold) {
        this->joint->SetForce(0, -magnetic_force);
      }
    }

  private:
    double near_threshold, magnetic_force;
    physics::ModelPtr model;
    physics::JointPtr joint;
    event::ConnectionPtr updateConnection;
  };

  GZ_REGISTER_MODEL_PLUGIN(MagneticJoint)
}
