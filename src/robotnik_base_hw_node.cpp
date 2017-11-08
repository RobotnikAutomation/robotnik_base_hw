#include <cmath>
#include <cstdint>
#include <cstdlib>

#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float32.h>               // battery, gyro, and steering angle
#include <std_msgs/Float64.h>               // battery, gyro, and steering angle
#include <std_msgs/Int32.h>               
#include <std_msgs/Bool.h>                  // 
#include <sensor_msgs/JointState.h>

#include <robotnik_msgs/MotorStatus.h>

#include <self_test/self_test.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <diagnostic_updater/publisher.h>

#include <robotnik_base_hw_lib/robotnik_base_hw.h>

enum{
	HW_STATE_INIT,
	HW_STATE_HOMING,
	HW_STATE_READY
};


// MAIN
int main(int argc, char** argv)
{
    ros::init(argc, argv, "robotnik_base_hw_node");
    int state = HW_STATE_INIT;
    double desired_freq_ = ROBOTNIK_DEFAULT_HZ;
    
    // TODO: remove debug level
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) { 
        ros::console::notifyLoggerLevelsChanged();
    } 

    ros::NodeHandle nh("");
    ros::NodeHandle pnh("~");
	pnh.param<double>("desired_freq", desired_freq_, desired_freq_);

    // NOTE: We run the ROS loop in a separate thread as external calls such
    // as service callbacks to load controllers can block the (main) control loop
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Create the hardware interface specific to your robot
    boost::shared_ptr<RobotnikBaseHW> robotnik_base_hw_lib (new RobotnikBaseHW(nh));   
    robotnik_base_hw_lib->initHardwareInterface();
    robotnik_base_hw_lib->Setup();
    robotnik_base_hw_lib->Start();

     // Start the control loop
    controller_manager::ControllerManager cm(&(*robotnik_base_hw_lib));

    ros::Rate loop_rate(desired_freq_);
    ros::Time last_time = ros::Time::now();

    // check that system is ready. previously this was done using the robotnik_base_hw_lib->WaitToBeReady(), but it is a blocking function
    // so now we do it separately and call the cm.update
    robotnik_base_hw_lib->InitSystem();

    last_time = ros::Time::now();
    robotnik_base_hw_lib->setMotorsToRunningFrequency();
    while (ros::ok())
    {
        loop_rate.sleep();

        ros::Time current_time = ros::Time::now();
        ros::Duration elapsed_time = current_time - last_time;
        last_time = current_time;

        robotnik_base_hw_lib->update();
        
        robotnik_base_hw_lib->read(elapsed_time);

        cm.update(current_time, elapsed_time);
        
        switch(state){
			
			case HW_STATE_INIT:
				if (robotnik_base_hw_lib->IsSystemReady()) {
					state = HW_STATE_HOMING;
					robotnik_base_hw_lib->SendToHome();
				}
			break;
			
			case HW_STATE_HOMING:
				if (robotnik_base_hw_lib->IsHomed()) {
					state = HW_STATE_READY;
					
				}
			break;
			
			case HW_STATE_READY:
				robotnik_base_hw_lib->write(elapsed_time);
			break;
		}
		
    }

    // Wait until shutdown signal recieved
    ros::waitForShutdown();

    return 0;
}
