#include <cmath>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
class RotatingWallPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    this->model_ = model;

    if (!sdf->HasElement("joint_name"))
    {
      gzerr << "[RotatingWallPlugin] Missing <joint_name> for model "
            << model->GetName() << "\n";
      return;
    }

    this->joint_name_ = sdf->Get<std::string>("joint_name");
    this->joint_ = model->GetJoint(this->joint_name_);
    if (!this->joint_)
    {
      gzerr << "[RotatingWallPlugin] Joint '" << this->joint_name_
            << "' not found in model " << model->GetName() << "\n";
      return;
    }

    this->speed_ = sdf->HasElement("speed") ? sdf->Get<double>("speed") : 1.0;
    this->force_ = sdf->HasElement("force") ? sdf->Get<double>("force") : 50.0;
    this->offset_ = sdf->HasElement("offset") ? sdf->Get<double>("offset") : 0.0;
    this->phase_ = sdf->HasElement("phase") ? sdf->Get<double>("phase") : 0.0;
    this->amplitude_ = sdf->HasElement("amplitude") ? sdf->Get<double>("amplitude") : 0.0;
    this->continuous_ =
      sdf->HasElement("continuous") ? sdf->Get<bool>("continuous") : false;

#if GAZEBO_MAJOR_VERSION >= 8
    this->initial_position_ = this->joint_->Position(0);
#else
    this->initial_position_ = this->joint_->GetAngle(0).Radian();
#endif

    this->start_time_ = model->GetWorld()->SimTime();

    // Allow the joint motor/controller to apply motion.
    this->joint_->SetParam("fmax", 0, this->force_);

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&RotatingWallPlugin::OnUpdate, this));
  }

private:
  void OnUpdate()
  {
    if (!this->joint_)
    {
      return;
    }

#if GAZEBO_MAJOR_VERSION >= 8
    const common::Time now = this->model_->GetWorld()->SimTime();
#else
    const common::Time now = this->model_->GetWorld()->GetSimTime();
#endif
    const double t = (now - this->start_time_).Double();

    if (this->continuous_)
    {
      this->joint_->SetParam("vel", 0, this->speed_);
      return;
    }

    const double target =
      this->initial_position_ + this->offset_ +
      this->amplitude_ * std::sin(this->speed_ * t + this->phase_);

#if GAZEBO_MAJOR_VERSION >= 8
    const double current = this->joint_->Position(0);
#else
    const double current = this->joint_->GetAngle(0).Radian();
#endif

    // Lightweight proportional controller for prismatic / non-continuous joints.
    const double error = target - current;
    const double commanded_velocity = std::max(-2.0, std::min(2.0, 4.0 * error));
    this->joint_->SetParam("vel", 0, commanded_velocity);
  }

  physics::ModelPtr model_;
  physics::JointPtr joint_;
  event::ConnectionPtr update_connection_;

  std::string joint_name_;
  common::Time start_time_;
  double initial_position_ = 0.0;
  double speed_ = 1.0;
  double force_ = 50.0;
  double offset_ = 0.0;
  double phase_ = 0.0;
  double amplitude_ = 0.0;
  bool continuous_ = false;
};

GZ_REGISTER_MODEL_PLUGIN(RotatingWallPlugin)
}  // namespace gazebo
