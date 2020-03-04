#include <cmath>
#include <cstdint>
#include <cstdlib>

#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float32.h>  // battery, gyro, and steering angle
#include <std_msgs/Float64.h>  // battery, gyro, and steering angle
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>  //
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>

#include <robotnik_msgs/MotorStatus.h>

#include <self_test/self_test.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <diagnostic_updater/publisher.h>

#include <robotnik_base_hw_lib/robotnik_base_hw.h>

#include <signal.h>

sighandler_t old_handler;
bool g_ok = true;

void signal_handler(int sgn)
{
  ROS_INFO("SIGNAL!");
  g_ok = false;
}

enum
{
  HW_STATE_INIT,
  HW_STATE_HOMING,
  HW_STATE_READY,
  HW_STATE_RESTART
};

class RobotnikBaseHWMain
{
public:
  RobotnikBaseHWMain() : nh_(""), pnh_("~")
  {
    state = HW_STATE_INIT;
    desired_freq_ = ROBOTNIK_DEFAULT_HZ;
    must_reset_hw_ = false;
    must_start_hw_ = false;
    started_ = false;
    manual_mode_ = true;
  }

  void rosSetup()
  {
    pnh_.param<double>("desired_freq", desired_freq_, desired_freq_);

    double period = 10;
    pnh_.param<double>("recovery_period", period, period);
    recovery_period_ = ros::Duration(period);

    auto_recovery_ = false;
    pnh_.param<bool>("auto_recovery", auto_recovery_, auto_recovery_);

    if (auto_recovery_ == true and period == 0)
    {
      ROS_WARN_STREAM_NAMED("RobotnikBaseHW",
                            "RobotnikBaseHW: you set auto_recovery, but a recovery period of 0. This is non sense. I "
                            "will disable auto recovery");
      auto_recovery_ = false;
    }

    reset_service_ = pnh_.advertiseService("reset_hw", &RobotnikBaseHWMain::resetHW, this);
    start_service_ = pnh_.advertiseService("start_hw", &RobotnikBaseHWMain::startHW, this);
    manual_service_ = pnh_.advertiseService("manual_hw", &RobotnikBaseHWMain::manualMode, this);
  }

  void run()
  {
    // Create the hardware interface specific to your robot
    robotnik_base_hw_lib_.reset(new RobotnikBaseHW(nh_));
    robotnik_base_hw_lib_->initHardwareInterface();
    while (g_ok)  // ros::ok())
    {
      ROS_INFO_STREAM("Waiting to start: " << must_start_hw_);
      if (must_start_hw_ == true)
      {
        if (robotnik_base_hw_lib_->Setup() == Component::ReturnValue::OK)
        {
          if (robotnik_base_hw_lib_->Start() == Component::ReturnValue::OK)
          {
            break;
          }
        }
      }
      ros::spinOnce();
      ros::Duration(5.0).sleep();
    }

    // Start the control loop
    controller_manager_.reset(new controller_manager::ControllerManager(&(*robotnik_base_hw_lib_)));

    // check that system is ready. previously this was done using the robotnik_base_hw_lib_->WaitToBeReady(), but it is
    // a
    // blocking function
    // so now we do it separately and call the controller_manager_.update
    robotnik_base_hw_lib_->InitSystem();

    ros::WallRate loop_rate(desired_freq_);
    ros::Time last_time = ros::Time::now();
    ros::SteadyTime last_steady_time = ros::SteadyTime::now();

    robotnik_base_hw_lib_->setMotorsToRunningFrequency();
    double last_time_check_in_current_state = robotnik_base_hw_lib_->GetTimeInCurrentState();
    started_ = true;
    must_start_hw_ = false;
    bool reset_controllers = false;
    while (g_ok)  // ros::ok())
    {
      loop_rate.sleep();

      ros::Time current_time = ros::Time::now();

      ros::SteadyTime current_steady_time = ros::SteadyTime::now();
      ros::Duration elapsed_time((current_steady_time - last_steady_time).toSec());
      last_steady_time = current_steady_time;

      last_time = current_time;

      robotnik_base_hw_lib_->update();

      robotnik_base_hw_lib_->read(elapsed_time);

      double time_in_current_state = robotnik_base_hw_lib_->GetTimeInCurrentState();
      if ((time_in_current_state - last_time_check_in_current_state) > recovery_period_.toSec())
      {
        last_time_check_in_current_state = time_in_current_state;

        if (robotnik_base_hw_lib_->GetComponentState() == Component::EMERGENCY_STATE)
        {
          if (robotnik_base_hw_lib_->isSafetyEnabled())
          {
            if (auto_recovery_ == true)
            {
              ROS_WARN_STREAM_NAMED("RobotnikBaseHW",
                                    "RobotnikBaseHW is in emergency for more than "
                                        << recovery_period_.toSec()
                                        << " seconds, but safety is enabled and auto recovery is enabled, so "
                                           "let's wait until safety is disable to try to recover from this");
            }
            else
            {
              ROS_WARN_STREAM_NAMED("RobotnikBaseHW",
                                    "RobotnikBaseHW is in emergency for more than "
                                        << recovery_period_.toSec()
                                        << " seconds, but safety is enabled and auto recovery is not enabled, "
                                           "maybe I cannot                          recover from this when safety "
                                           "is disabled");
            }
          }
          else
          {
            if (auto_recovery_ == false)
            {
              ROS_WARN_STREAM_NAMED("RobotnikBaseHW", "RobotnikBaseHW is in emergency for more than "
                                                          << recovery_period_.toSec()
                                                          << " seconds, safety is not enabled but auto "
                                                          << " recovery is not enabled. Maybe I will keep in this "
                                                             "state until the end of time");
            }
            else
            {
              ROS_WARN_STREAM_NAMED("RobotnikBaseHW", "RobotnikBaseHW is in emergency for more than "
                                                          << recovery_period_.toSec()
                                                          << " seconds, safety is not enabled and auto recovery "
                                                             "is enabled. I'm trying to restart the system");
              must_reset_hw_ = true;
            }
          }
        }
      }

      if (started_) // started_ should not be exist. should be a method of base_hw_lib. this is a quick and dirty hack
      {
        bool base_hw_not_in_ready = (robotnik_base_hw_lib_->GetComponentState() != Component::READY_STATE);
        controller_manager_->update(current_time, elapsed_time, (reset_controllers or manual_mode_ or base_hw_not_in_ready));
        reset_controllers = false;
      }

      if (must_reset_hw_)
      {
        state = HW_STATE_RESTART;
      }

      switch (state)
      {
        case HW_STATE_INIT:
          if (robotnik_base_hw_lib_->IsSystemReady())
          {
            state = HW_STATE_HOMING;
            bool force_home = false;
            robotnik_base_hw_lib_->SendToHome(force_home);
          }
          break;

        case HW_STATE_HOMING:
          if (robotnik_base_hw_lib_->IsHomed())
          {
            state = HW_STATE_READY;
          }
          break;

        case HW_STATE_READY:
          robotnik_base_hw_lib_->write(elapsed_time);
          break;
        case HW_STATE_RESTART:
          if (must_reset_hw_) {
            robotnik_base_hw_lib_->Stop();
            robotnik_base_hw_lib_->ShutDown();
            robotnik_base_hw_lib_->destroyMotorDrives();
            must_reset_hw_ = false;
            started_ = false;
          }
          if (must_start_hw_) {
            robotnik_base_hw_lib_->createMotorDrives();
            robotnik_base_hw_lib_->Setup();
            robotnik_base_hw_lib_->Start();
            state = HW_STATE_INIT;
            must_start_hw_ = false;
            started_ = true;
            reset_controllers = true;
          }
          break;
      }
    }
  }

  void stop()
  {
    ROS_INFO("Stop");
    robotnik_base_hw_lib_->Stop();
    robotnik_base_hw_lib_->ShutDown();
    robotnik_base_hw_lib_->destroyMotorDrives();
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  int state;
  double desired_freq_;
  // must reset changes HW_STATE machine to the reset state. Can be set due to the auto recovery or the reset service
  bool must_reset_hw_;
  bool must_start_hw_;
  bool started_;
  bool manual_mode_;

  bool auto_recovery_;
  ros::Duration recovery_period_;

  boost::shared_ptr<RobotnikBaseHW> robotnik_base_hw_lib_;
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  ros::ServiceServer reset_service_;
  ros::ServiceServer start_service_;
  ros::ServiceServer manual_service_;

public:
  bool resetHW(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
  {
    must_reset_hw_ = true;
    return true;
  }
  
  bool startHW(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
  {
    must_start_hw_ = true;
    return true;
  }

  bool manualMode(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
  {
    ROS_INFO_STREAM("I'm in " << (manual_mode_ ? "manual": "auto") << " and will switch to "
                            << (request.data ? "manual" : "auto"));
    manual_mode_ = request.data;

    robotnik_base_hw_lib_->setModeSrvCallback(request, response);
    return true;
  }
};

// MAIN
int main(int argc, char** argv)
{
  ros::init(argc, argv, "robotnik_base_hw_node", ros::init_options::NoSigintHandler);

  old_handler = signal(SIGINT, signal_handler);

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(2);
  spinner.start();

  RobotnikBaseHWMain base;

  base.rosSetup();
  base.run();

  ros::Rate r(2);
  while (g_ok)
  {
    ROS_INFO("Sleep");
    r.sleep();
  }

  // base.stop();

  // Wait until shutdown signal recieved
  // ros::waitForShutdown();

  return 0;
}
