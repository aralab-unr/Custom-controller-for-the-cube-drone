#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <std_msgs/Float64MultiArray.h>
#include <thread>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

class DronePlugin : public gazebo::ModelPlugin {
public:
  DronePlugin() : gazebo::ModelPlugin() {
    std::cout << "Starting drone_plugin" << std::endl;
  }

  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    this->model = _model;

    if (_sdf->HasElement("updateRate")) {
      this->rate = _sdf->Get<double>("updateRate");
    } else {
      this->rate = 20;
    }

    if (_sdf->HasElement("publishTf")) {
      this->publish_tf = _sdf->Get<bool>("publishTf");
    } else {
      this->publish_tf = true;
    }

    if (_sdf->HasElement("rotorThrustCoeff")) {
      this->rotor_thrust_coeff = _sdf->Get<double>("rotorThrustCoeff");
    } else {
      this->rotor_thrust_coeff = 0.00025;
    }

    if (_sdf->HasElement("rotorTorqueCoeff")) {
      this->rotor_torque_coeff = _sdf->Get<double>("rotorTorqueCoeff");
    } else {
      this->rotor_torque_coeff = 0.000075;
    }

    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized");
      return;
    }

    ROS_INFO("ROS Model Plugin Loaded!");
    this->sub = this->nh.subscribe("/prop_vel", 10, &DronePlugin::Activate_Callback, this);
    this->ros_thread = std::thread(std::bind(&DronePlugin::rosThread, this));
    this->jointbackleft = _model->GetJoint("pbl");
    this->jointbackright = _model->GetJoint("pbr");
    this->jointfrontleft = _model->GetJoint("pfl");
    this->jointfrontright = _model->GetJoint("pfr");
    this->jointbacklefttop = _model->GetJoint("pblt");
    this->jointbackrighttop = _model->GetJoint("pbrt");
    this->jointfrontlefttop = _model->GetJoint("pflt");
    this->jointfrontrighttop = _model->GetJoint("pfrt");
    this->jointmidleft = _model->GetJoint("pml");
    this->jointmidright = _model->GetJoint("pmr");
  }

public:
  void OnUpdate() {
    if (this->propvel.size() != 10) {
      ROS_WARN("Propeller velocities not properly initialized.");
      return;
    }

    this->model->GetJoint("pbl")->SetVelocity(0, this->propvel[0]);
    this->model->GetJoint("pbr")->SetVelocity(0, this->propvel[1]);
    this->model->GetJoint("pfl")->SetVelocity(0, this->propvel[2]);
    this->model->GetJoint("pfr")->SetVelocity(0, this->propvel[3]);
    this->model->GetJoint("pblt")->SetVelocity(0, this->propvel[4]);
    this->model->GetJoint("pbrt")->SetVelocity(0, this->propvel[5]);
    this->model->GetJoint("pflt")->SetVelocity(0, this->propvel[6]);
    this->model->GetJoint("pfrt")->SetVelocity(0, this->propvel[7]);
    this->model->GetJoint("pml")->SetVelocity(0, this->propvel[8]);
    this->model->GetJoint("pmr")->SetVelocity(0, this->propvel[9]);


    double thrustbl = calculateThrust(this->propvel[0]);
    double thrustbr = calculateThrust(this->propvel[1]);
    double thrustfl = calculateThrust(this->propvel[2]);
    double thrustfr = calculateThrust(this->propvel[3]);
    double thrustblt = calculateThrust(this->propvel[4]);
    double thrustbrt = calculateThrust(this->propvel[5]);
    double thrustflt = calculateThrust(this->propvel[6]);
    double thrustfrt = calculateThrust(this->propvel[7]);
    double thrustml = calculatesideforce(this->propvel[8]);
    double thrustmr = calculatesideforce(this->propvel[9]);


    double torquebl = calculatetorque(this->propvel[0]);
    double torquebr = calculatetorque(this->propvel[1]);
    double torquefl = calculatetorque(this->propvel[2]);
    double torquefr = calculatetorque(this->propvel[3]);
    double torqueblt = calculatetorque(this->propvel[4]);
    double torquebrt = calculatetorque(this->propvel[5]);
    double torqueflt = calculatetorque(this->propvel[6]);
    double torquefrt = calculatetorque(this->propvel[7]);
    double torqueml = calculatetorque(this->propvel[8]);
    double torquemr = calculatetorque(this->propvel[9]);


    gazebo::physics::LinkPtr linkbl = this->model->GetLink("propbackleft");
    gazebo::physics::LinkPtr linkbr = this->model->GetLink("propbackright");
    gazebo::physics::LinkPtr linkfl = this->model->GetLink("propfrontleft");
    gazebo::physics::LinkPtr linkfr = this->model->GetLink("propfrontright");
    gazebo::physics::LinkPtr linkblt = this->model->GetLink("propbacklefttop");
    gazebo::physics::LinkPtr linkbrt = this->model->GetLink("propbackrighttop");
    gazebo::physics::LinkPtr linkflt = this->model->GetLink("propfrontlefttop");
    gazebo::physics::LinkPtr linkfrt = this->model->GetLink("propfrontrighttop");
    gazebo::physics::LinkPtr linkml = this->model->GetLink("propmidleft");
    gazebo::physics::LinkPtr linkmr = this->model->GetLink("propmidright");
    gazebo::physics::LinkPtr base = this->model->GetLink("base_link");

    linkbl->AddLinkForce(ignition::math::Vector3d(0, 0, thrustbl));
    linkbr->AddLinkForce(ignition::math::Vector3d(0, 0, thrustbr));
    linkfl->AddLinkForce(ignition::math::Vector3d(0, 0, thrustfl));
    linkfr->AddLinkForce(ignition::math::Vector3d(0, 0, thrustfr));
    linkbl->AddLinkForce(ignition::math::Vector3d(0, 0, thrustblt));
    linkbr->AddLinkForce(ignition::math::Vector3d(0, 0, thrustbrt));
    linkfl->AddLinkForce(ignition::math::Vector3d(0, 0, thrustflt));
    linkfr->AddLinkForce(ignition::math::Vector3d(0, 0, thrustfrt));
    linkml->AddLinkForce(ignition::math::Vector3d(0, 0, -thrustml));
    linkmr->AddLinkForce(ignition::math::Vector3d(0, 0, -thrustmr));

    base->AddRelativeTorque(ignition::math::Vector3d(0, 0, torquebl));
    base->AddRelativeTorque(ignition::math::Vector3d(0, 0, torquebr));
    base->AddRelativeTorque(ignition::math::Vector3d(0, 0, torquefl));
    base->AddRelativeTorque(ignition::math::Vector3d(0, 0, torquefr));
    base->AddRelativeTorque(ignition::math::Vector3d(0, 0, torqueblt));
    base->AddRelativeTorque(ignition::math::Vector3d(0, 0, torquebrt));
    base->AddRelativeTorque(ignition::math::Vector3d(0, 0, torqueflt));
    base->AddRelativeTorque(ignition::math::Vector3d(0, 0, torquefrt));

  }

  void rosThread() {
    while (ros::ok()) {
      ros::spinOnce();
    }
  }

  void Activate_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (msg->data.size() != 10) {
      ROS_WARN("Received incorrect number of propeller velocities.");
      return;
    }
    ROS_INFO("Received Message = [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f,].", msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5], msg->data[6], msg->data[7], msg->data[8], msg->data[9]);
    this->propvel = msg->data;
    this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&DronePlugin::OnUpdate, this));
  }

  double calculateThrust(double w) {
    double thrust = this->rotor_thrust_coeff * w * w;
    return thrust;
  }

  double calculatetorque(double w) {
    double thrust = - sgn(w) * this->rotor_torque_coeff * w * w;
    return thrust;
  }

  double calculatesideforce(double w) {
    double thrust = sgn(w) * this->rotor_thrust_coeff * w * w;
    return thrust;
  }
private:
  double rate;
  bool publish_tf;
  double rotor_thrust_coeff;
  double rotor_torque_coeff;
  gazebo::physics::ModelPtr model;
  gazebo::physics::JointPtr jointbackleft;
  gazebo::physics::JointPtr jointbackright;
  gazebo::physics::JointPtr jointfrontleft;
  gazebo::physics::JointPtr jointfrontright;
  gazebo::physics::JointPtr jointbacklefttop;
  gazebo::physics::JointPtr jointbackrighttop;
  gazebo::physics::JointPtr jointfrontlefttop;
  gazebo::physics::JointPtr jointfrontrighttop;
  gazebo::physics::JointPtr jointmidleft;
  gazebo::physics::JointPtr jointmidright;
  gazebo::event::ConnectionPtr updateConnection;
  ros::NodeHandle nh;           // ROS node handler
  ros::Subscriber sub;
  std::thread ros_thread;
  std::vector<double> propvel = std::vector<double>(10, 0.0);
};

GZ_REGISTER_MODEL_PLUGIN(DronePlugin)
