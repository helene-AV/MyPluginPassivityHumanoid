#include "MyPluginPassivityHumanoid.h"

#include <mc_control/GlobalPluginMacros.h>
#include <mc_tvm/Robot.h>
#include <RBDyn/FA.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/ID.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <Eigen/src/Core/IO.h>
#include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <jrl-qp/experimental/BoxAndSingleConstraintSolver.h>
#include <tvm/defs.h>

#include <cmath>

//test 

namespace mc_plugin
{

MyPluginPassivityHumanoid::~MyPluginPassivityHumanoid() = default;

void MyPluginPassivityHumanoid::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  mc_rtc::log::info("[MyPluginPassivityHumanoid][init] Init called with configuration:\n{}", config.dump(true, true));

  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  auto & robot = ctl.robot();
  auto nrDof = robot.mb().nrDof();
  solver_ = std::make_shared<jrl::qp::experimental::BoxAndSingleConstraintSolver>(robot.mb().nrJoints());

  // ========== Check for Virtual Torque Sensor to hold feedback term value ========== //
  if (!robot.hasDevice<mc_rbdyn::VirtualTorqueSensor>("virtualTorqueSensor"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[MyPluginPassivityHumanoid][Init] No \"VirtualTorqueSensor\" with the name \"virtualTorqueSensor\" found in the robot module, please add one to the robot's RobotModule.");
  }
  virtual_torque_sensor_ = &robot.device<mc_rbdyn::VirtualTorqueSensor>("virtualTorqueSensor");
  dt_ = ctl.timestep();
  coriolis_ = new rbd::Coriolis(robot.mb());
  fd_ = new rbd::ForwardDynamics(robot.mb());
  id_ = new rbd::InverseDynamics(robot.mb());
  
  // ====================  Load  config  ==================== //
  verbose_ = config("verbose", false);
  lambda_massmatrix_ = config("lambda_massmatrix", 0.5);
  lambda_id_ = config("lambda_id", 0.1);
  fast_filter_weight_ = config("fast_filter_weight",0.9);
  phi_slow_ = config("phi_slow",0.3);
  phi_fast_ = config("phi_fast",10.0);
  perc_ = config("perc",10);
  perc_target_ = config("perc",10);
  is_changing_ = false;
  filtered_activated_ = true;
  is_active_ = false; 
  coriolis_indicator_ = config("coriolis_indicator",true);
  coriolis_indicator_value_ = 1.0;
  if (coriolis_indicator_){coriolis_indicator_value_=1.0;}
  maxAngAcc_ = Eigen::Vector3d(5,5,5) * (M_PI / 180.0);
  maxLinAcc_ = Eigen::Vector3d(0.5,0.5,10);
  config("maxAngAcc", maxAngAcc_);
  config("maxLinAcc", maxLinAcc_);


  // ==================== Config loaded ==================== //

  // Initializing all variables
  exp_phi_slow_ = exp(-dt_*phi_slow_);
  exp_phi_fast_ = exp(-dt_*phi_fast_);
  alpha_r_ = Eigen::VectorXd::Zero(nrDof);
  K_ = Eigen::MatrixXd::Zero(nrDof,nrDof);
  D_ = Eigen::MatrixXd::Zero(nrDof,nrDof);
  C_ = Eigen::MatrixXd::Zero(nrDof,nrDof);
  s_ = Eigen::VectorXd::Zero(nrDof);
  new_s_ = Eigen::VectorXd::Zero(nrDof);
  prev_s_ = Eigen::VectorXd::Zero(nrDof);
  slow_filtered_s_ = Eigen::VectorXd::Zero(nrDof);
  fast_filtered_s_ = Eigen::VectorXd::Zero(nrDof);
  motor_current_ = Eigen::VectorXd::Zero(nrDof);
  tau_ = Eigen::VectorXd::Zero(nrDof);
  tau_qp_= Eigen::VectorXd::Zero(nrDof);
  tau_Out_= Eigen::VectorXd::Zero(nrDof);
  tau_coriolis_ = Eigen::VectorXd::Zero(nrDof);
  tau_current_ = Eigen::VectorXd::Zero(nrDof);
  tau_sum_ = Eigen::VectorXd::Zero(nrDof);


  addGUI(controller);
  addLOG(controller);

  count_ = 0;

  format = Eigen::IOFormat(2, 0, " ", "\n", "[", "]", " ", " ");

  ctl.controller().datastore().make_call("PassivityPlugin::activated", [this]() { 
    if ( is_active_ == false)    
    this->is_active_ = true; });

  mc_rtc::log::success("[MyPluginPassivityHumanoid][init] Initialization completed");
}

void MyPluginPassivityHumanoid::reset(mc_control::MCGlobalController & controller)
{
  removeLOG(controller);
  mc_rtc::log::info("[MyPluginPassivityHumanoid] Reset called");
}

void MyPluginPassivityHumanoid::before(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  auto & robot = ctl.robot();
  
  auto coriolis_activation = ctl.controller().datastore().get<std::string>("Coriolis");
  if (coriolis_activation.compare("Yes") == 0) {coriolis_indicator_ = true;coriolis_indicator_value_=1.0; }
  else {coriolis_indicator_ = false;coriolis_indicator_value_=0.0; }
 
  if (robot.encoderVelocities().empty()) {return;}

  rbd::paramToVector(robot.mbc().jointTorque, tau_Out_);
  tau_qp_ = tau_Out_-tau_ ;

  Eigen::VectorXd alpha_d(robot.mb().nrDof());
  Eigen::VectorXd alpha(robot.mb().nrDof());
  rbd::paramToVector(robot.alphaD(),alpha_d);
  rbd::paramToVector(robot.alpha(),alpha);
  
  alpha_r_ +=  alpha_d*dt_;

  rbd::forwardKinematics(robot.mb(), robot.mbc());
  rbd::forwardVelocity(robot.mb(), robot.mbc());
  fd_->forwardDynamics(robot.mb(),robot.mbc());  
  C_ = coriolis_->coriolis(robot.mb(),robot.mbc());

  // Calculation of the tau current motor

  // motor_current_ = Eigen::Map<Eigen::VectorXd>(robot.jointTorques().data(), robot.jointTorques().size());
  for (int i = 0; i < motor_current_.size(); ++i) { 
    motor_current_[i] = robot.jointTorques()[i];
    if (std::isnan(motor_current_(i))){tau_current_(i)= 0.0;}
    else {tau_current_(i) = motor_current_(i);} 
    // il faut utiliser la relation trouvee par mathieu et rentrer les valeurs une par une pour chacun des joints
    // et qui est dans HRP5P.xml 
  }

  // Passivity Torque Feedback and QP-based Anti-Windup if the Plugin is activated

  auto ctrl_mode = ctl.controller().datastore().get<std::string>("ControlMode");
  if (ctrl_mode.compare("Torque") == 0) {ctl.controller().datastore().call("PassivityPlugin::activated");}

  if(is_active_)
  {
    // Calculation of the error
    if (filtered_activated_) 
    {
      new_s_ = alpha_r_ - alpha;
      slow_filtered_s_ = exp_phi_slow_*slow_filtered_s_ + new_s_ - prev_s_;
      fast_filtered_s_ = exp_phi_fast_*fast_filtered_s_ + new_s_ - prev_s_;
      prev_s_ = new_s_;
      s_ = fast_filter_weight_ * fast_filtered_s_ + (1 - fast_filter_weight_) * slow_filtered_s_;
    }
    else
    {
      s_ = alpha_r_ - alpha;
    }

    tau_coriolis_ = coriolis_indicator_value_*C_* s_ ;
    K_ = lambda_massmatrix_* fd_->H() + lambda_id_*Eigen::MatrixXd::Identity(robot.mb().nrDof(),robot.mb().nrDof());
    tau_ = K_*s_;

    // Exponential integration for perc_ to maintain continuity in case the torque is constrained by the QP-Based Anti-Windup

    if (is_changing_) {
        perc_ += (perc_target_ - perc_) * 0.01;

        // Check if perc_ has reached perc_target_ 
        if (std::abs(perc_ - perc_target_) < 0.01) {
            perc_ = perc_target_;
            is_changing_ = false;
        }
    }

    // QP-Based Anti-Windup and calculation of the bound violation

    Eigen::VectorXd torqueL_(robot.mb().nrDof());
    Eigen::VectorXd torqueU_(robot.mb().nrDof());

    rbd::paramToVector(robot.tl(),torqueL_);
    rbd::paramToVector(robot.tu(),torqueU_);

    Eigen::VectorXd torqueL_prime(robot.mb().nrDof());
    Eigen::VectorXd torqueU_prime(robot.mb().nrDof());

    torqueL_prime= torqueL_ * perc_ /100;
    torqueU_prime= torqueU_ * perc_ /100;

    // Floating base

    for (int i = 0; i < robot.mb().nrJoints(); i++)
    {
      if (robot.mb().joint(i).type() == rbd::Joint::Free)
      {
        int j = robot.mb().jointPosInDof(i);
        Eigen::Vector6d acc;
        acc << maxAngAcc_, maxLinAcc_;
        torqueU_prime.segment<6>(j) = fd_->H().block<6, 6>(j, j).diagonal().array() * acc.array();
        torqueL_prime.segment<6>(j) = -torqueU_prime.segment<6>(j);
        break;
      }
    }

    /// get the multiplier of the bound violation
    double epsilonU = (tau_.array() / torqueU_prime.array()).maxCoeff();
    double epsilonL = (tau_.array() / torqueL_prime.array()).maxCoeff();
    double epsilon  = std::max(std::max(epsilonU, epsilonL),1.);

    if (epsilon >1)
    {
      double dotprod = tau_.dot(s_)/epsilon;
      if (solver_->solve(tau_,s_,dotprod,torqueL_prime,torqueU_prime)==jrl::qp::TerminationStatus::SUCCESS)
      {
        tau_ = solver_->solution();
      } 
      else
      {
        std::cout << "Mehdi QP FAILED"<<std::endl;
        std::cerr << "Mehdi QP FAILED"<<std::endl;
        tau_ /=epsilon;
      }
    }
    tau_ += tau_coriolis_; 
    virtual_torque_sensor_->torques(tau_);
  }

  else
  {
    K_ = lambda_massmatrix_* fd_->H() + lambda_id_*Eigen::MatrixXd::Identity(robot.mb().nrDof(),robot.mb().nrDof());
    slow_filtered_s_ = (1-fast_filter_weight_)*(K_+coriolis_indicator_value_*C_).inverse()*(tau_current_ - tau_qp_);
    s_ = slow_filtered_s_;
    tau_ = (K_ + coriolis_indicator_value_*C_)*s_;

}

  power_ = (tau_current_.transpose()).dot(alpha);
  tau_sum_= tau_qp_ + tau_ ;
  count_++;
}


void MyPluginPassivityHumanoid::after(mc_control::MCGlobalController &)
{
  mc_rtc::log::info("MyPluginPassivityHumanoid::after");
} 

void MyPluginPassivityHumanoid::addGUI(mc_control::MCGlobalController & controller)
{
    auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
    auto gui = ctl.controller().gui();

    gui->addElement({"Plugins", "Integral term feedback", "Configure"},
        mc_rtc::gui::Button(
            "Activate Plugin",
            [this]() {is_active_= true, mc_rtc::log::info("IntegralFeedback activated"); }
        )
    );
  gui->addElement({"Plugins","Integral term feedback","Configure"},
    mc_rtc::gui::Checkbox("Coriolis effect", this->coriolis_indicator_)
  );
  gui->addElement({"Plugins","Integral term feedback","Configure"},
    mc_rtc::gui::Checkbox("Filtered Integral if activated, simple otherwise", this->filtered_activated_)
  );
  gui->addElement({"Plugins","Integral term feedback","Configure"},
    mc_rtc::gui::NumberInput("Anti-Windup percentage",
            [this]() { return this->perc_; },
            [this](double perc_new) {
                this->perc_target_ = perc_new;
                this->is_changing_ = true;
            })
  );
  gui->addElement({"Plugins","Integral term feedback","Configure"},
    mc_rtc::gui::NumberInput("Gain Mass matrix",
      [this]() { return this-> lambda_massmatrix_; },
      [this](double lambda_massmatrix_new) {
        torque_continuity(lambda_massmatrix_new, lambda_id_);
        lambda_massmatrix_ = lambda_massmatrix_new;
      })
  );
  gui->addElement({"Plugins","Integral term feedback","Configure"},
    mc_rtc::gui::NumberInput("Gain Identity matrix",
      [this]() { return this-> lambda_id_; },
      [this](double lambda_id_new) {
        torque_continuity(lambda_massmatrix_, lambda_id_new);
        lambda_id_=lambda_id_new;
      })
  );
  gui->addElement({"Plugins","Integral term feedback","Configure"},
    mc_rtc::gui::NumberInput("Slow filter phi",
      [this]() { return phi_slow_; },
      [this](double phi) {
        phi_slow_ = phi;
        exp_phi_slow_ = exp(-phi*dt_);
      }
    )
  );
  gui->addElement({"Plugins","Integral term feedback","Configure"},
    mc_rtc::gui::NumberInput("Fast filter phi",
      [this]() { return phi_fast_; },
      [this](double phi) {
        phi_fast_ = phi;
        exp_phi_fast_ = exp(-phi*dt_);
      }
    )
  );
  gui->addElement({"Plugins","Integral term feedback","Configure"},
    mc_rtc::gui::NumberInput("Fast filter ratio", this->fast_filter_weight_)
  );

  // Logs
  gui->addElement({"Plugins","Integral term feedback","Logs"},
    mc_rtc::gui::ArrayLabel("Ref velocity",
      ctl.controller().robot().refJointOrder(),
      [this]() { return alpha_r_;}
    )
  );

  gui->addElement({"Plugins","Integral term feedback","Logs"},
    mc_rtc::gui::ArrayLabel("New non filtered integral term",
      ctl.controller().robot().refJointOrder(),
      [this]() { return new_s_;}
    )
  );
  
  gui->addElement({"Plugins","Integral term feedback","Logs"},
    mc_rtc::gui::ArrayLabel("Past non filtered integral term",
      ctl.controller().robot().refJointOrder(),
      [this]() { return prev_s_;}
    )
  );

  gui->addElement({"Plugins","Integral term feedback","Logs"},
    mc_rtc::gui::ArrayLabel("Slow filtered integral term",
      ctl.controller().robot().refJointOrder(),
      [this]() { return slow_filtered_s_;}
    )
  );

  gui->addElement({"Plugins","Integral term feedback","Logs"},
    mc_rtc::gui::ArrayLabel("Fast filtered integral term",
      ctl.controller().robot().refJointOrder(),
      [this]() { return fast_filtered_s_;}
    )
  );

  gui->addElement({"Plugins","Integral term feedback","Logs"},
    mc_rtc::gui::ArrayLabel("Integral term",
      ctl.controller().robot().refJointOrder(),
      [this]() { return s_;}
    )
  );

  gui->addElement({"Plugins","Integral term feedback","Logs"},
    mc_rtc::gui::ArrayLabel("Torque sent",
      ctl.controller().robot().refJointOrder(),
      [this]() { return tau_;}
    )
  );

  gui->addElement({"Plugins","Integral term feedback","Logs"},
    mc_rtc::gui::ArrayLabel("Torque QP",
      ctl.controller().robot().refJointOrder(),
      [this]() { return tau_qp_;}
    )
  );

  // Plots
  gui->addElement({"Plugins","Integral term feedback","Plots"},
    mc_rtc::gui::ElementsStacking::Horizontal,
    mc_rtc::gui::Button("Plot integral term",
      [this, gui]() {
        gui->addPlot("Integral term feedback",
          mc_rtc::gui::plot::X("t", [this]() { return static_cast<double>(count_) * dt_; }),
          mc_rtc::gui::plot::Y("Integral term q(0)",[this]() { return s_[0]; }, mc_rtc::gui::Color::Blue),
          mc_rtc::gui::plot::Y("Integral term q(1)",[this]() { return s_[1]; }, mc_rtc::gui::Color::Red),
          mc_rtc::gui::plot::Y("Integral term q(2)",[this]() { return s_[2]; }, mc_rtc::gui::Color::Green),
          mc_rtc::gui::plot::Y("Integral term q(3)",[this]() { return s_[3]; }, mc_rtc::gui::Color::Yellow),
          mc_rtc::gui::plot::Y("Integral term q(4)",[this]() { return s_[4]; }, mc_rtc::gui::Color::Magenta),
          mc_rtc::gui::plot::Y("Integral term q(5)",[this]() { return s_[5]; }, mc_rtc::gui::Color::Cyan),
          mc_rtc::gui::plot::Y("Integral term q(6)",[this]() { return s_[6]; }, mc_rtc::gui::Color::Black)
        );
      }
    ),
    mc_rtc::gui::Button("Remove integral term plot", [gui]() { gui->removePlot("Integral term feedback"); })
  );
}

void MyPluginPassivityHumanoid::addLOG(mc_control::MCGlobalController & controller)
{
  controller.controller().logger().addLogEntry("MyPluginPassivityHumanoid_active", [&, this]() { return this->is_active_; });
  controller.controller().logger().addLogEntry("MyPluginPassivityHumanoid_changing", [&, this]() { return this->is_changing_; });

  controller.controller().logger().addLogEntry("MyPluginPassivityHumanoid_type", [&, this]() { return static_cast<int>(this->filtered_activated_); });
  controller.controller().logger().addLogEntry("MyPluginPassivityHumanoid_percentage_of_bounds", [&, this]() { return this->perc_; });
  controller.controller().logger().addLogEntry("MyPluginPassivityHumanoid_coriolis_indicator_value", [&, this]() { return this->coriolis_indicator_value_; });

  controller.controller().logger().addLogEntry("MyPluginPassivityHumanoid_gain_mass_matrix", [&, this]() { return this->lambda_massmatrix_; });
  controller.controller().logger().addLogEntry("MyPluginPassivityHumanoid_gain_identity", [&, this]() { return this->lambda_id_; });
  controller.controller().logger().addLogEntry("MyPluginPassivityHumanoid_filter_slow_phi", [&, this]() { return this->phi_slow_; });
  controller.controller().logger().addLogEntry("MyPluginPassivityHumanoid_filter_fast_phi", [&, this]() { return this->phi_fast_; });
  controller.controller().logger().addLogEntry("MyPluginPassivityHumanoid_filter_slow_exp_phi", [&, this]() { return this->exp_phi_slow_; });
  controller.controller().logger().addLogEntry("MyPluginPassivityHumanoid_filter_fast_exp_phi", [&, this]() { return this->exp_phi_fast_; });
  controller.controller().logger().addLogEntry("MyPluginPassivityHumanoid_filter_ratio", [&, this]() { return this->fast_filter_weight_; });
  controller.controller().logger().addLogEntry("MyPluginPassivityHumanoid_integral_of_reference_acceleration", [&, this]() { return this->alpha_r_; });
  controller.controller().logger().addLogEntry("MyPluginPassivityHumanoid_K", [&, this]() { return static_cast<Eigen::VectorXd>(this->K_.diagonal()); });
  controller.controller().logger().addLogEntry("MyPluginPassivityHumanoid_s", [&, this]() { return this->s_; });
  controller.controller().logger().addLogEntry("MyPluginPassivityHumanoid_filter_new_s", [&, this]() { return this->new_s_; });
  controller.controller().logger().addLogEntry("MyPluginPassivityHumanoid_filter_previous_s", [&, this]() { return this->prev_s_; });
  controller.controller().logger().addLogEntry("MyPluginPassivityHumanoid_filter_slow_s", [&, this]() { return this->slow_filtered_s_; });
  controller.controller().logger().addLogEntry("MyPluginPassivityHumanoid_filter_fast_s", [&, this]() { return this->fast_filtered_s_; });
  controller.controller().logger().addLogEntry("MyPluginPassivityHumanoid_torque_passivity", [&, this]() { return this->tau_; });
  controller.controller().logger().addLogEntry("MyPluginPassivityHumanoid_torque_qp", [&, this]() { return this->tau_qp_; });
  controller.controller().logger().addLogEntry("MyPluginPassivityHumanoid_torque_coriolis", [&, this]() { return this->tau_coriolis_; });
  controller.controller().logger().addLogEntry("MyPluginPassivityHumanoid_torque_current", [&, this]() { return this->tau_current_; });
  controller.controller().logger().addLogEntry("MyPluginPassivityHumanoid_torque_sum", [&, this]() { return this->tau_sum_; });
  controller.controller().logger().addLogEntry("MyPluginPassivityHumanoid_power", [&, this]() { return this->power_; });

}

void MyPluginPassivityHumanoid::removeLOG(mc_control::MCGlobalController & controller)
{
  controller.controller().logger().removeLogEntry("MyPluginPassivityHumanoid_active");
  controller.controller().logger().removeLogEntry("MyPluginPassivityHumanoid_changing");

  controller.controller().logger().removeLogEntry("MyPluginPassivityHumanoid_type");
  controller.controller().logger().removeLogEntry("MyPluginPassivityHumanoid_percentage_of_bounds");
  controller.controller().logger().removeLogEntry("MyPluginPassivityHumanoid_coriolis_indicator_value");

  controller.controller().logger().removeLogEntry("MyPluginPassivityHumanoid_gain_mass_matrix");
  controller.controller().logger().removeLogEntry("MyPluginPassivityHumanoid_gain_identity");
  controller.controller().logger().removeLogEntry("MyPluginPassivityHumanoid_filter_slow_phi");
  controller.controller().logger().removeLogEntry("MyPluginPassivityHumanoid_filter_fast_phi");
  controller.controller().logger().removeLogEntry("MyPluginPassivityHumanoid_filter_slow_exp_phi");
  controller.controller().logger().removeLogEntry("MyPluginPassivityHumanoid_filter_fast_exp_phi");
  controller.controller().logger().removeLogEntry("MyPluginPassivityHumanoid_filter_ratio");
  controller.controller().logger().removeLogEntry("MyPluginPassivityHumanoid_integral_of_reference_acceleration");
  controller.controller().logger().removeLogEntry("MyPluginPassivityHumanoid_K");
  controller.controller().logger().removeLogEntry("MyPluginPassivityHumanoid_s");
  controller.controller().logger().removeLogEntry("MyPluginPassivityHumanoid_filter_new_s");
  controller.controller().logger().removeLogEntry("MyPluginPassivityHumanoid_filter_previous_s");
  controller.controller().logger().removeLogEntry("MyPluginPassivityHumanoid_filter_slow_s");
  controller.controller().logger().removeLogEntry("MyPluginPassivityHumanoid_filter_fast_s");
  controller.controller().logger().removeLogEntry("MyPluginPassivityHumanoid_torque_passivity");
  controller.controller().logger().removeLogEntry("MyPluginPassivityHumanoid_torque_qp");
  controller.controller().logger().removeLogEntry("MyPluginPassivityHumanoid_torque_coriolis");
  controller.controller().logger().removeLogEntry("MyPluginPassivityHumanoid_torque_current");
  controller.controller().logger().removeLogEntry("MyPluginPassivityHumanoid_torque_sum");
  controller.controller().logger().removeLogEntry("MyPluginPassivityHumanoid_power");

}


void MyPluginPassivityHumanoid::torque_continuity(double lambda_massmatrix,double lambda_id)
{
  D_.diagonal() = fd_->H().diagonal();
  Eigen::MatrixXd L_new = coriolis_indicator_value_*C_ + lambda_massmatrix * fd_->H() + lambda_id*Eigen::MatrixXd::Identity(s_.size(),s_.size());
  Eigen::MatrixXd update_matrix = L_new.inverse()*(coriolis_indicator_value_*C_+K_);
  // Disjonction for filtered or simple integration 
  if (is_active_){
    if (filtered_activated_) {
    slow_filtered_s_ = update_matrix* slow_filtered_s_;
    fast_filtered_s_ = update_matrix* fast_filtered_s_;
    s_ = fast_filter_weight_ * fast_filtered_s_ + (1 - fast_filter_weight_) * slow_filtered_s_;
    }
    else {
    s_ = update_matrix*s_;
    }
  }
}

mc_control::GlobalPlugin::GlobalPluginConfiguration MyPluginPassivityHumanoid::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = false;
  return out;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("MyPluginPassivityHumanoid", mc_plugin::MyPluginPassivityHumanoid)
