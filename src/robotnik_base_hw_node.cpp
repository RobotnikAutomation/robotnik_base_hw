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
    RobotnikBaseHWMain()
    {
      state = HW_STATE_INIT;
      desired_freq_ = ROBOTNIK_DEFAULT_HZ;
    }

    void rosSetup()
    {  
      pnh_.param<double>("desired_freq", desired_freq_, desired_freq_);
      
      double period = 5;
      pnh_.param<double>("recovery_period", period, period);
      recovery_period_ = ros::Duration(period);
      
      reset_service_ = pnh_.advertiseService("reset_hw", &RobotnikBaseHWMain::resetHW, this);
    }

    void run()
    {
      // Create the hardware interface specific to your robot
      robotnik_base_hw_lib_.reset(new RobotnikBaseHW(nh_));
      robotnik_base_hw_lib_->initHardwareInterface();
      while (ros::ok())
      {
        if (robotnik_base_hw_lib_->Setup() == Component::ReturnValue::OK)
        {
          if (robotnik_base_hw_lib_->Start() == Component::ReturnValue::OK)
          {
            break;
          }
        }
        ros::Duration(5.0).sleep();
      }

      // Start the control loop
      controller_manager_.reset(new controller_manager::ControllerManager(&(*robotnik_base_hw_lib_)));

      ros::Rate loop_rate(desired_freq_);
      ros::Time last_time = ros::Time::now();

      // check that system is ready. previously this was done using the robotnik_base_hw_lib_->WaitToBeReady(), but it is a
      // blocking function
      // so now we do it separately and call the controller_manager_.update
      robotnik_base_hw_lib_->InitSystem();

      last_time = ros::Time::now();
      robotnik_base_hw_lib_->setMotorsToRunningFrequency();
      while (ros::ok())
      {
        loop_rate.sleep();

        ros::Time current_time = ros::Time::now();
        ros::Duration elapsed_time = current_time - last_time;
        last_time = current_time;

        robotnik_base_hw_lib_->update();

        robotnik_base_hw_lib_->read(elapsed_time);

        double time_in_current_state = robotnik_base_hw_lib_->GetTimeInCurrentState();
        if (time_in_current_state > recovery_period_.toSec()) 
        {
          if (robotnik_base_hw_lib_->GetComponentState() == Component::EMERGENCY_STATE and true)
         //     not robotnik_base_hw_lib_->isSecurityEnabled())
          {
            ROS_WARN_STREAM("RobotnikBaseHW is in emergency for more than " << recovery_period_.toSec() << " seconds, and security is not enabled. I'm trying to restart the system");
            must_restart_hw_ = true;
          }
        }

        controller_manager_->update(current_time, elapsed_time);

        if (must_restart_hw_)
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
            must_restart_hw_ = false;
            robotnik_base_hw_lib_->Stop();
            ros::Duration(5).sleep();
            robotnik_base_hw_lib_->RestartCan();
            robotnik_base_hw_lib_->Start();
            state = HW_STATE_INIT;
            break;
        }
      }
    }

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    int state;
    double desired_freq_;
    bool must_restart_hw_;
    boost::shared_ptr<RobotnikBaseHW> robotnik_base_hw_lib_;
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
    
    ros::ServiceServer reset_service_;
    ros::Duration recovery_period_;

  public:
    bool resetHW(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
    {
      must_restart_hw_ = true;
      return true;
    }
};



// MAIN
int main(int argc, char** argv)
{
  ros::init(argc, argv, "robotnik_base_hw_node");

  // TODO: remove debug level
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }


  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(2);
  spinner.start();

  RobotnikBaseHWMain base;

  base.rosSetup();
  base.run();


  // Wait until shutdown signal recieved
  ros::waitForShutdown();

  return 0;
}
