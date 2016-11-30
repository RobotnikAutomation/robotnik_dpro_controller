/** \file robotnik_dpro_controller.cpp
 * \author Robotnik Automation S.L.L.
 * \version 1.0
 * \date    2015
 *
 * \brief robotnik_dpro_controller ros node
 * Component to control a set of dynamixel pro motors
 * (C) 2015 Robotnik Automation, SLL
 * uses ROBOTS Dynamixel PRO SDK
*/
#include <string.h>
#include <vector>
#include <stdint.h>
#include <ros/ros.h>
#include <math.h>
#include <cstdlib>

#include "self_test/self_test.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"

#include <robotnik_msgs/set_float_value.h>
#include "dynamixel.h"
#include "bulkread.h"

#include <sensor_msgs/JointState.h>
#include <robotnik_msgs/inputs_outputs.h>

#include <std_srvs/Empty.h>
//#include <std_msgs/Bool.h>					

#include <time.h>
#include <sys/time.h>

//#define	 DXL_MIN_COMMAND_REC_FREQ	10.0
//#define	 DXL_MAX_COMMAND_REC_FREQ	20.0

#define	 DXL_MIN_COMMAND_REC_FREQ	5.0
#define	 DXL_MAX_COMMAND_REC_FREQ	25.0
#define  DPRO_IDLE_TIME				0.0

/*
 * 
// Pps to angle conversions 
#define  PPS2DEG 0.001185185 	// 180.0/151875.0
#define  PPS2RAD 0.000020685 	// PI/151875.0 
*/

// Control table addresses
#define OPERATING_MODE     	11
#define HOME_OFFSET        	13
#define MAX_VELOCITY       	32
#define TORQUE_ENABLE      	562
#define GOAL_POSITION      	596
#define GOAL_VELOCITY	   	600
#define GOAL_TORQUE  	   	604
#define GOAL_ACCELERATION  	606

#define PRESENT_CURRENT    	621
#define PRESENT_VELOCITY  	615
#define PRESENT_POSITION   	611
#define INPUT_VOLTAGE      	632
#define PRESENT_TEMPERATURE 625
#define HW_ERROR_STATUS     892
#define P_MOVING 			610 

#define	VELOCITY_I_GAIN		586
#define	VELOCITY_P_GAIN		588
#define	POSITION_P_GAIN		594


// other settings
#define DEFAULT_BAUDNUM     1 		// 57600 bps to match Dynamixel PRO's
#define CONTROL_PERIOD      (10) 	// arbitrary value

#define MAX_JOINTS         10    	// Max joints per usb/485 interface

#define MOVING_PID_MODE		1
#define IDLE_PID_MODE		2


using namespace DXL_PRO;


/**
 * Struct that describes each servo's place in the system including 
 * which joint it corresponds to. 
 */
struct dynamixel_info 
{
	int id;
	std::string joint_name;
	std::string model_name;
	uint16_t model_number;
	uint32_t model_info;
	int cpr;
	double gear_reduction;
	double max_vel_value;  		// max velocity value to be programmed in the register
	double max_vel_rads;   		// equivalent max velocity in rad/s
	std::string joint_type;     // prismatic, revolute
	double min_rad;				// max position in rad
	double max_rad;        		// min position in rad (for linear axis max < min in absolute is accepted)
	double min_m;          		// min position in meter
	double max_m;          		// max position in meter (always max>min)
	int lim_sw_pos;				// if defined, these are indexes to digital_inputs inverted logic that mark the limit switch 
	int lim_sw_neg; 			// positive and negative
	int vel_i_gain;				// Velocity I gain param
	int vel_p_gain;				// Velocity P gain param
	int pos_p_gain;				// Position P gain param
	int vel_i_gain_idle;		// Velocity I gain param when the motor is idle
	int vel_p_gain_idle;		// Velocity P gain param when the motor is idle
	int pos_p_gain_idle;		// Position P gain param when the motor is idle
	bool dynamic_pid;			// Applies a dynamic regulation depending the motor state
	bool is_moving;				// Flag active when the servo is moving
	int current_pid_mode;		// 1 for std, 2 for idle
};

/**
 * The different control modes available on the dynamixel servos. 
 */
enum control_mode
{
	POSITION_CONTROL = 3,
	VELOCITY_CONTROL = 1,
	TORQUE_CONTROL = 0,
	UNKOWN  = -1
};

/**
 * A struct that describes the state of a dynamixel servo motor
 */
struct dynamixel_status
{
	int id;
	control_mode mode; 
	bool torque_enabled;
};


class robotnik_dpro_controller_node
{

public:

	self_test::TestRunner self_test_;
	ros::NodeHandle node_handle_;
	ros::NodeHandle private_node_handle_;
	int error_count_;
	int slow_count_;
	double desired_freq_;
	diagnostic_updater::Updater diagnostic_;			// General status diagnostic updater
	diagnostic_updater::FrequencyStatus freq_diag_;		         // Component frequency diagnostics
	diagnostic_updater::HeaderlessTopicDiagnostic *subs_command_freq; // Topic reception frequency diagnostics
	ros::Time last_command_time_;				// Last moment when the component received a command	
	ros::Time last_read_time_;					// Last moment when the positions were read
	ros::Time last_io_time_;					// Last moment when the component received io data
	ros::Time startup_time_; 
    diagnostic_updater::FunctionDiagnosticTask command_freq_;

	// Current axis position in pulses    
	long position_pps_[MAX_JOINTS];
	
	//! Parameters	
	//! Port for USB DXL converter
	std::string port_;
	
	//! Motor driver component 
	Dynamixel* driver_;
	
	//! Node running
	bool running;	

	//! Error counters and flags
	std::string was_slow_;
	std::string error_status_;
	
	//! Topic to read I/O to control limit switch
	std::string io_topic_;

	//! Ros service stop 
	ros::ServiceServer srv_stop_;
	
	//! Ros service home
	ros::ServiceServer srv_home_;

    //! Publishers
    ros::Publisher joint_state_pub_;
	
	//! Subscribers
	ros::Subscriber joint_state_sub_;
	ros::Subscriber io_sub_;          
	
	//! Joint state message
    sensor_msgs::JointState joint_commands_;
    
    //! Joint state message
    sensor_msgs::JointState joint_states_;
    	    
	//! Maps joint_name <-> motor data and index <-> motor status 
	std::map<std::string, dynamixel_info> joint2dynamixel_;
	std::map<std::string, ros::Time> joint2time_;
	std::map<std::string, int> joint2pidmode_;
    std::map<int, dynamixel_status> id2status_;

	//! Flag to inform about shutdown
	bool shutting_down_;
	
	//! Flag to inform that a joint command has been received
	bool bJointCommandReceived_;
	
	//! Flag to mark if commands are being received
	bool bReceiving_;

	bool bReceivingIO_;
		
	//! Read write cycle measured frequecny
	double cycle_freq_;
	
	//! Flag to mark if the node needs to receive io data
	bool bIODataNeeded_;
	
	//! 
	long present_position_prismatic_;
	int id_prismatic_;

	
  
/*!	\fn robotnik_dpro_controller_node::robotnik_dpro_controller_node()
 * 	\brief Public constructor
*/
robotnik_dpro_controller_node(ros::NodeHandle h) : self_test_(), diagnostic_(),
  node_handle_(h), private_node_handle_("~"), 
  error_count_(0),
  slow_count_(0),
  desired_freq_(20),
  freq_diag_(diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, 0.05)   ),
  command_freq_("Command frequency check", boost::bind(&robotnik_dpro_controller_node::check_command_subscriber, this, _1))  
{
    running = false;
    ros::NodeHandle robotnik_dpro_controller_node(node_handle_, "robotnik_dpro_controller");
    private_node_handle_.param<std::string>("port", port_, "/dev/ttyUSB0");
    private_node_handle_.param<std::string>("io_topic", io_topic_, "/modbus_io/input_output");
     
    //! Self test
    self_test_.add("Connect Test", this, &robotnik_dpro_controller_node::ConnectTest);  

    //! Component frequency diagnostics
    diagnostic_.setHardwareID("robotnik_dpro_controller");
    diagnostic_.add("Motor Controller", this, &robotnik_dpro_controller_node::controller_diagnostic);
    diagnostic_.add( freq_diag_ );
    diagnostic_.add( command_freq_ );
    
    ROS_INFO("Port = %s", port_.c_str());
    
    // Create new motor driver
    driver_ = new Dynamixel(port_.c_str());

    // Open device
    if( driver_->Connect() == 0 ) {
        ROS_ERROR("robotnik_dpro_controller_node: Failed to open USB2Dynamixel!" );
        exit(-1);
		}
    else
        ROS_INFO("robotnik_dpro_controller_node: Succeed to open USB2Dynamixel!");

    if(driver_->SetBaudrate(DEFAULT_BAUDNUM) == true) {
    	ROS_INFO( "robotnik_dpro_controller_node: Succeed to change the baudrate!" );
		}
    else {
        ROS_ERROR( "robotnik_dpro_controller_node: Failed to change the baudrate!\n" );
        exit(-1);
		}

    //! Services and Topics
    //! Service to stop robot
	srv_stop_ = private_node_handle_.advertiseService("stop", &robotnik_dpro_controller_node::srvCallback_Stop, this);
	srv_home_ = private_node_handle_.advertiseService("home", &robotnik_dpro_controller_node::srvCallback_SetHomeOffset, this);
	
	//! Publish joint states for wheel motion visualization
	joint_state_pub_ = private_node_handle_.advertise<sensor_msgs::JointState>("/joint_states", 10);
	
	//! Subcscribing
	//Start listening to command messages. There is a queue size of 1k so that
    //we don't accidentally miss commands that are sent to us in batches for many joints at once. 
    joint_state_sub_ = private_node_handle_.subscribe<sensor_msgs::JointState>("/joint_commands", 
        1000, &robotnik_dpro_controller_node::cmdJointStateCallback, this);
    
    io_sub_ = private_node_handle_.subscribe<robotnik_msgs::inputs_outputs>(io_topic_, 
        10, &robotnik_dpro_controller_node::cmdIOCallback, this);
    
    //! Component frequency diagnostics
    diagnostic_.setHardwareID("robotnik_dpro_controller_node");
    diagnostic_.add("Motor Controller", this, &robotnik_dpro_controller_node::controller_diagnostic);
    diagnostic_.add( freq_diag_ );
    diagnostic_.add( command_freq_ );
      
    //! Topics freq control 
    //! For /rb1_base/cmd_vel
    double min_freq = DXL_MIN_COMMAND_REC_FREQ; // If you update these values, the
    double max_freq = DXL_MAX_COMMAND_REC_FREQ; // HeaderlessTopicDiagnostic will use the new values.
    ROS_INFO("Desired freq %5.2f", desired_freq_);
    subs_command_freq = new diagnostic_updater::HeaderlessTopicDiagnostic("/joint_commands", diagnostic_,
	                    diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq, 0.1, 10));
    subs_command_freq->addTask(&command_freq_); // Adding an additional task to the control
	
	//! Read servos parameters from parameter server and initialize global maps
	bIODataNeeded_ = false; // by default not needed, if limit switches defined in config files this flag is activated
	read_servo_parameters();
	
	shutting_down_ = false;
	
	bJointCommandReceived_ = false;
	bReceiving_ = false;
        bReceivingIO_= false;
		
	cycle_freq_ = 0.0; 

    startup_time_ = ros::Time::now();
    
    present_position_prismatic_ = 0;
    id_prismatic_ = 0;
}
	
	
/*!	\fn robotnik_dpro_controller_node::read_servo_parameters()
 * 	\brief Reads parameters from parameter server and initializes internal structs
*/
void read_servo_parameters()
{

    // read in the information regarding the servos that we're supposed to connect to 
    if (private_node_handle_.hasParam("servos")) {
        XmlRpc::XmlRpcValue servos;
        private_node_handle_.getParam("servos", servos);
        //If there is no servos array in the param server, return
        if (!servos.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            ROS_ERROR("Invalid/missing servo information on the param server");
            ROS_BREAK();
			}
        
        // For every servo, load and verify its information        
        for (int i = 0; i < servos.size(); i++) {
            dynamixel_info info;
            if (!servos[i].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                ROS_ERROR("Invalid/Missing info-struct for servo index %d", i);
                ROS_BREAK();
				}

            if (!servos[i]["id"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                ROS_ERROR("Invalid/Missing id for servo index %d", i);
                ROS_BREAK();
				}
            else {
                //store the servo's ID                
                info.id = static_cast<int>(servos[i]["id"]);
                ROS_INFO("id = %d", info.id);
				}

            if (!servos[i]["joint_name"].getType() == XmlRpc::XmlRpcValue::TypeString) {
                ROS_ERROR("Invalid/Missing joint name for servo index %d, id: %d", i, info.id);
                ROS_BREAK();
				}
            else {
                //store the servo's corresponding joint                
                info.joint_name = static_cast<std::string>(servos[i]["joint_name"]);                
				}

			if (!servos[i]["model_name"].getType() == XmlRpc::XmlRpcValue::TypeString) {
				ROS_ERROR("Invalid/Missing model_name for servo index %d, id: %d", i, info.id);
				ROS_BREAK();
				}
			else {
				info.model_name = static_cast<std::string>(servos[i]["model_name"]);
				}

			// uint16_t model_number;
			// uint32_t model_info;	           
            if (!servos[i]["cpr"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
				ROS_ERROR("Invalid/Missing cpr for servo index %d, id=%d", i, info.id);
				ROS_BREAK();
				}
			else {
				info.cpr = static_cast<int>(servos[i]["cpr"]);
				}

            if (!servos[i]["gear_reduction"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
				ROS_ERROR("Invalid/Missing gear_reduction for servo index %d, id=%d", i, info.id);
				ROS_BREAK();
				}
			else {
				info.gear_reduction = static_cast<double>(servos[i]["gear_reduction"]);
				}

            if (!servos[i]["max_vel_value"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
				ROS_ERROR("Invalid/Missing max_vel_value for servo index %d, id=%d", i, info.id);
				ROS_BREAK();
				}
			else {
				info.max_vel_value = static_cast<double>(servos[i]["max_vel_value"]);
				}

            if (!servos[i]["max_vel_rads"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
				ROS_ERROR("Invalid/Missing max_vel_rads for servo index %d, id=%d", i, info.id);
				ROS_BREAK();
				}
			else {
				info.max_vel_rads = static_cast<double>(servos[i]["max_vel_rads"]);
				}

			if (!servos[i]["joint_type"].getType() == XmlRpc::XmlRpcValue::TypeString) {
				ROS_ERROR("Invalid/Missing type for servo index %d, id: %d", i, info.id);
				ROS_BREAK();
				}
			else {
				info.joint_type = static_cast<std::string>(servos[i]["joint_type"]);
				}

            if (!servos[i]["min_rad"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
				ROS_ERROR("Invalid/Missing min_rad for servo index %d, id=%d", i, info.id);
				ROS_BREAK();
				}
			else {
				info.min_rad = static_cast<double>(servos[i]["min_rad"]);
				}

            if (!servos[i]["max_rad"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
				ROS_ERROR("Invalid/Missing max_rad for servo index %d, id=%d", i, info.id);
				ROS_BREAK();
				}
			else {
				info.max_rad = static_cast<double>(servos[i]["max_rad"]);
				}

            if (!servos[i]["min_m"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
				if (info.joint_type == "prismatic") {
					ROS_ERROR("Invalid/Missing min_m for servo index %d, id=%d", i, info.id);
					ROS_BREAK();
					}
				else {
					info.min_m = 0.0;
					}
				}
			else {
				info.min_m = static_cast<double>(servos[i]["min_m"]);
				}

            if (!servos[i]["max_m"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
				if (info.joint_type == "prismatic") {
					ROS_ERROR("Invalid/Missing max_m for servo index %d, id=%d", i, info.id);
					ROS_BREAK();
					}
				else {
					info.max_m = 0.0;
					}
				}
			else {
				info.max_m = static_cast<double>(servos[i]["max_m"]);
				}

            // Store an index if the axis needs limit switches, otherwise -1
            if (servos[i].hasMember("lim_sw_pos"))  { 
				if (!servos[i]["lim_sw_pos"].getType() == XmlRpc::XmlRpcValue::TypeInt) {				
					info.lim_sw_pos = -1; // mark as index
				}
				else {
					info.lim_sw_pos = static_cast<int>(servos[i]["lim_sw_pos"]);
									ROS_INFO("lim_sw_pos=%d", info.lim_sw_pos);
					bIODataNeeded_ = true;
				}
			}
			else info.lim_sw_pos = -1;

            if (servos[i].hasMember("lim_sw_neg")) {
				if (!servos[i]["lim_sw_neg"].getType() == XmlRpc::XmlRpcValue::TypeInt) {				
					info.lim_sw_neg = -1; // mark as index
				}
				else {
					info.lim_sw_neg = static_cast<int>(servos[i]["lim_sw_neg"]);
                                ROS_INFO("lim_sw_neg=%d", info.lim_sw_neg);
					bIODataNeeded_ = true;
				}
			}
			else info.lim_sw_neg = -1;
			
			if(servos[i].hasMember("vel_i_gain")) {
				if (!servos[i]["vel_i_gain"].getType() == XmlRpc::XmlRpcValue::TypeInt) {				
					info.vel_i_gain = -1; // mark as index
				}
				else {
					info.vel_i_gain = static_cast<int>(servos[i]["vel_i_gain"]);
				}
			}else 
				info.vel_i_gain = -1;
		
			if(servos[i].hasMember("vel_p_gain")) {
				if (!servos[i]["vel_p_gain"].getType() == XmlRpc::XmlRpcValue::TypeInt) {				
					info.vel_p_gain = -1; // mark as index
				}
				else {
					info.vel_p_gain = static_cast<int>(servos[i]["vel_p_gain"]);
				}
			}else 
				info.vel_p_gain = -1;
				
			if(servos[i].hasMember("pos_p_gain")) {
				if (!servos[i]["pos_p_gain"].getType() == XmlRpc::XmlRpcValue::TypeInt) {				
					info.pos_p_gain = -1; // mark as index
				}
				else {
					info.pos_p_gain = static_cast<int>(servos[i]["pos_p_gain"]);
				}
			}else 
				info.pos_p_gain = -1;
				
			if(servos[i].hasMember("vel_i_gain_idle")) {
				if (!servos[i]["vel_i_gain_idle"].getType() == XmlRpc::XmlRpcValue::TypeInt) {				
					info.vel_i_gain_idle = -1; // mark as index
				}
				else {
					info.vel_i_gain_idle = static_cast<int>(servos[i]["vel_i_gain_idle"]);
				}
			}else 
				info.vel_i_gain_idle = -1;
		
			if(servos[i].hasMember("vel_p_gain_idle")) {
				if (!servos[i]["vel_p_gain_idle"].getType() == XmlRpc::XmlRpcValue::TypeInt) {				
					info.vel_p_gain_idle = -1; // mark as index
				}
				else {
					info.vel_p_gain_idle = static_cast<int>(servos[i]["vel_p_gain_idle"]);
				}
			}else 
				info.vel_p_gain_idle = -1;
				
			if(servos[i].hasMember("pos_p_gain_idle")) {
				if (!servos[i]["pos_p_gain_idle"].getType() == XmlRpc::XmlRpcValue::TypeInt) {				
					info.pos_p_gain_idle = -1; // mark as index
				}
				else {
					info.pos_p_gain_idle = static_cast<int>(servos[i]["pos_p_gain_idle"]);
				}
			}else 
				info.pos_p_gain_idle = -1;
				
			if(servos[i].hasMember("dynamic_pid")) {
				if (!servos[i]["dynamic_pid"].getType() == XmlRpc::XmlRpcValue::TypeBoolean) {				
					info.dynamic_pid = false; // 
				}
				else {
					info.dynamic_pid = static_cast<bool>(servos[i]["dynamic_pid"]);
				}
			}else 
				info.dynamic_pid = false;
			
			// To control the servo movement
			info.is_moving = false;
			joint2time_[info.joint_name] = ros::Time::now();
			joint2pidmode_[info.joint_name] = MOVING_PID_MODE;
			
            // Store in the std associative map of motor data
            ROS_INFO("joint_name = %s", info.joint_name.c_str() );
            joint2dynamixel_[info.joint_name] = info;               							          
            
			// Store in the std associative map of motor status
			dynamixel_status status;
            status.id = info.id;
            status.mode = UNKOWN;
            status.torque_enabled = false;
            id2status_[info.id] = status;
            
            // Initialize joint_states array	        
		    joint_states_.name.push_back(info.joint_name);
		    joint_states_.position.push_back(0.0);
		    joint_states_.velocity.push_back(0.0);
		    joint_states_.effort.push_back(0.0);	            
                        
			} // for
		}
    else {
        ROS_ERROR("No servos details loaded to param server");
        ROS_BREAK();
		}
}


/*!	\fn robotnik_dpro_controller_node::check_command_subscriber
 * 	\brief Checks that the controller is receiving at a correct frequency the command messages. Diagnostics
*/
void check_command_subscriber(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	ros::Time current_time = ros::Time::now();

	double diff = (current_time - last_command_time_).toSec();
		
	if(diff > 0.25){
		stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Topic is not receiving commands");
        // ROS_INFO("check_command_subscriber: cmd %lf seconds without commands", diff);
		// If this happens in safe velocity, deactivate flag in order to stop motors.
		// This condition is not relevant in position or position velocity commands
		bReceiving_ = false;
				
	}else{
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Topic receiving commands");		
	}
}

/*!	\fn robotnik_dpro_controller_node::ConnectTest()
 * 	\brief Test to connect to Motors
*/
void ConnectTest(diagnostic_updater::DiagnosticStatusWrapper& status)
{
   // Connection test or ping test
   int error;
   int id;   
   int ret = -1;
   if (driver_!=NULL) {
		
	//! Connect to each servomotor
	for (std::map<std::string, dynamixel_info>::iterator iter = joint2dynamixel_.begin(); iter != joint2dynamixel_.end(); iter++) {
		id = iter->second.id;		
		ret = driver_->Ping(id, &error);
		if (error!=0) {
			ROS_ERROR("Connection Failed. id=%d", id);			
			int hw_error_status = 0;
			ret = driver_->ReadByte(id, HW_ERROR_STATUS, (int*) &hw_error_status, &error);		
			usleep(CONTROL_PERIOD*200);
			process_error_code( hw_error_status, id );
			}
		else status.summary(0, "Connected successfully.");				
		}
	}
}

/*!	\fn robotnik_dpro_controller_node::process_error_code
 * 	\brief Creates the appropriate error msgs in case of error
*/
void process_error_code(int ErrorCode, int id)
{
// According to E-manual 7/5/2015
// Bit 0 Input Voltage Error / Input voltage exceeds limit
// Bit 1 Motor hall sensor error / Motor hall sensor value exceeds limits
// Bit 2 OverHeating Error / Internal operating temperature exceeds limit
// Bit 3 Motor encoder error / Motor encoder not working
// Bit 4 Electronical shock error / Electrical shock to circuit, or insufficient input voltage to drive motor


    if(ErrorCode & ERRBIT_VOLTAGE) ROS_ERROR("Input voltage error! id=%d", id);
    if(ErrorCode & ERRBIT_ANGLE) ROS_ERROR("Motor hall sensor error! id=%d", id);
    if(ErrorCode & ERRBIT_OVERHEAT) ROS_ERROR("Overheat error! id=%d", id);
    if(ErrorCode & ERRBIT_RANGE) ROS_ERROR("Motor encoder error! id=%d", id);
    if(ErrorCode & ERRBIT_CHECKSUM) ROS_ERROR("Electronical shock error or insuficient input voltage to drive motor id=%d", id);
    if(ErrorCode & ERRBIT_OVERLOAD) ROS_ERROR("OLD Overload error! id=%d", id);
    if(ErrorCode & ERRBIT_INSTRUCTION) ROS_ERROR("OLD Instruction code error!");	

	// stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Controller initialing");	
	// stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Controller on Failure or unknown state");	
}


/*!	\fn  robotnik_dpro_controller_node::CheckHwErrorStatus
 * 	\brief Reads the hw error status byte (contains the error flags) and creates error messages according to 
*/
void CheckHwErrorStatus()
{	
	int error = 0;
	// int ret = driver->ReadByte(id_left_, HW_ERROR_STATUS, (int*) &hw_error_status_left_, &error);
	// usleep(CONTROL_PERIOD*100);        
	// if (error!=0) process_error_code( hw_error_status_left_, id_left_ );		
}


/*!	\fn robotnik_dpro_controller_node::process_comm_status
 * 	\brief Process communication status
*/
void process_comm_status(int CommStatus)
{
    switch(CommStatus)
    {
    case COMM_TXFAIL:
        ROS_ERROR("COMM_TXFAIL: Failed transmit instruction packet!");
        break;

    case COMM_TXERROR:
        ROS_ERROR("COMM_TXERROR: Incorrect instruction packet!");
        break;

    case COMM_RXFAIL:
        ROS_ERROR("COMM_RXFAIL: Failed get status packet from device!");
        break;

    case COMM_RXWAITING:
        ROS_ERROR("COMM_RXWAITING: Now recieving status packet!");
        break;

    case COMM_RXTIMEOUT:
        ROS_ERROR("COMM_RXTIMEOUT: There is no status packet!");
        break;

    case COMM_RXCORRUPT:
        ROS_ERROR("COMM_RXCORRUPT: Incorrect status packet!");
        break;

    default:
        ROS_ERROR("This is unknown error code!");
        break;
    }
}


/*!	\fn robotnik_dpro_controller_node::controller_diagnostic
 * 	\brief Checks the status of the driver Diagnostics
*/
void controller_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	// add and addf are used to append key-value pairs.
	// stat.addf("Controller StatusWord (HEX)", "%x", sw ); // Internal controller status
	
	// TODO => Add diagnostic info 
	// stat.add("Encoder position left", (int) position_left_pps_ );
	// stat.add("Encoder position right", (int) position_right_pps_ );
}

// Some intermediate level functions
/*!     \fn  robotnik_dpro_controller::setTorqueEnabled(int id, int flag)
  *      Enable / Disable servo
*/
void setTorqueEnabled(int id, int flag)
{
	int error;
	int ret = driver_->WriteByte(id, TORQUE_ENABLE, (int8_t)flag, &error);
    
    
	if( ret != COMM_RXSUCCESS ) {
		ROS_ERROR("setTorqueEnable Error. value = %d, id=%d",flag,id);
		process_comm_status( ret );
	}    
	usleep(CONTROL_PERIOD*1000);
}

/*!     \fn  robotnik_dpro_controller::setTorqueEnabled(int id, int flag)
  *      Enable / Disable servo
*/
void setOperatingMode(int id, int new_mode)
{
	int error;
    int ret = driver_->WriteByte(id, new_mode, 1, &error);
   
    
	if( ret != COMM_RXSUCCESS ) {
		ROS_ERROR("setOperatingMode Error. id=%d", id);
		process_comm_status( ret );
	}		
	usleep(CONTROL_PERIOD*1000);
}

/*! \fn  void setMotorParams(int id)
  *    
*/
void setMotorParams(int id, int velocity_i_gain, int velocity_p_gain, int position_p_gain)
{
	int error;
    int ret = 0;
    
    
    if(velocity_i_gain >= 0){
		ret = driver_->WriteWord(id, VELOCITY_I_GAIN, velocity_i_gain, &error);
		if( ret != COMM_RXSUCCESS ) {
			ROS_ERROR("setMotorParamsMode: Error setting VELOCITY_I_GAIN param to %d. id=%d", velocity_i_gain, id);
			process_comm_status( ret );
		}		
		usleep(CONTROL_PERIOD*1000);
	}
	
	if(velocity_p_gain >= 0){
		ret = driver_->WriteWord(id, VELOCITY_P_GAIN, velocity_p_gain, &error);
		
		if( ret != COMM_RXSUCCESS ) {
			ROS_ERROR("setMotorParamsMode: Error setting VELOCITY_P_GAIN param to %d. id=%d", velocity_p_gain, id);
			process_comm_status( ret );
		}		
		usleep(CONTROL_PERIOD*1000);
	}
	
	if(position_p_gain >= 0){
		ret = driver_->WriteWord(id, POSITION_P_GAIN, position_p_gain, &error);
		
		if( ret != COMM_RXSUCCESS ) {
			ROS_ERROR("setMotorParamsMode: Error setting POSITION_P_GAIN param to %d. id=%d",position_p_gain, id);
			process_comm_status( ret );
		}		
		usleep(CONTROL_PERIOD*1000);
	}
	
	ROS_INFO("setMotorParams: id %d. setting V_I_GAIN to %d, V_P_GAIN to %d, P_P_GAIN to %d",  id, velocity_i_gain, velocity_p_gain, position_p_gain);
}

/*!     \fn  robotnik_dpro_controller::setAcceleration(int id, int goal_accel)
  *      Enable / Disable servo
*/
void setAcceleration(int id, int goal_accel)
{	
	int error;
	// Check bug, was writting only in id=11
	int ret = driver_->WriteDWord(id, GOAL_ACCELERATION , (int32_t) goal_accel, &error); 
    usleep(CONTROL_PERIOD*1000);
	if( ret != COMM_RXSUCCESS ) {
		ROS_ERROR("setAcceleration Error. id=%d", id);		
		process_comm_status( ret );
		}		
}

/*!     \fn  robotnik_dpro_controller::setVelocity(int id, int goal_vel)
  *      
*/
void setVelocity(int id, int goal_vel)
{	
	int error;
	// Check bug, was writting only in id=11
	int ret = driver_->WriteDWord(id, GOAL_VELOCITY, (int32_t) goal_vel, &error); 
    usleep(CONTROL_PERIOD*1000);
	if( ret != COMM_RXSUCCESS ) {
		ROS_ERROR("setVelocity Error. id=%d", id);		
		process_comm_status( ret );
	}		
}

/*!     \fn  robotnik_dpro_controller::setMultiPosition(std::vector<std::vector<int> > value_tuples)
  *      Send one sync msg with multiple positions for different axes
*/
bool setMultiPosition(std::vector<std::vector<int> > value_tuples)
{              
	int num_actuator = value_tuples.size();
	unsigned char param[num_actuator*(1+4)];
    
    
    // Make syncwrite packet
    for(int i = 0; i < value_tuples.size(); i++) {

        uint8_t motor_id = value_tuples[i][0];
        int32_t position = value_tuples[i][1];
		// ROS_INFO("setMultiPosition motor_id=%d i=%d pos=%d", motor_id, i, position); 
        param[i*(1+4)+0] = (unsigned char)motor_id;
        param[i*(1+4)+1] = DXL_LOBYTE(DXL_LOWORD(position));
        param[i*(1+4)+2] = DXL_HIBYTE(DXL_LOWORD(position));
        param[i*(1+4)+3] = DXL_LOBYTE(DXL_HIWORD(position));
        param[i*(1+4)+4] = DXL_HIBYTE(DXL_HIWORD(position));
        }
        
        int result = driver_->SyncWrite(GOAL_POSITION, 4, param, num_actuator*(1+4));
        usleep(CONTROL_PERIOD*100);
        if( result != COMM_RXSUCCESS ) {
            process_comm_status(result);
            return false;
			}
		else return true;	              
}

/*!     \fn  robotnik_dpro_controller::setMultiPositionVelocity(std::vector<std::vector<int> > value_tuples)
  *      Send one sync msg with multiple positions and velocities for different axes
*/
bool setMultiPositionVelocity(std::vector<std::vector<int> > value_tuples)
{              
	int num_actuator = value_tuples.size();
	unsigned char param[num_actuator*(1+8)];
    
    // Make syncwrite packet
    for(int i = 0; i < value_tuples.size(); i++) {

        uint8_t motor_id = value_tuples[i][0];
        int32_t position = value_tuples[i][1];
        int32_t velocity = value_tuples[i][2];

        param[i*(1+8)+0] = (unsigned char)motor_id;
        param[i*(1+8)+1] = DXL_LOBYTE(DXL_LOWORD(position));
        param[i*(1+8)+2] = DXL_HIBYTE(DXL_LOWORD(position));
        param[i*(1+8)+3] = DXL_LOBYTE(DXL_HIWORD(position));
        param[i*(1+8)+4] = DXL_HIBYTE(DXL_HIWORD(position));
        param[i*(1+8)+5] = DXL_LOBYTE(DXL_LOWORD(velocity));
        param[i*(1+8)+6] = DXL_HIBYTE(DXL_LOWORD(velocity));
        param[i*(1+8)+7] = DXL_LOBYTE(DXL_HIWORD(velocity));
        param[i*(1+8)+8] = DXL_HIBYTE(DXL_HIWORD(velocity));        
        }
        
        int result = driver_->SyncWrite(GOAL_POSITION, 8, param, num_actuator*(1+8));
        usleep(CONTROL_PERIOD*100);
        if( result != COMM_RXSUCCESS ) {
            process_comm_status(result);
            return false;
			}
		else return true;	              
}

/*!     \fn  robotnik_dpro_controller::setMultiVelocity(std::vector<std::vector<int> > value_tuples)
  *      Send one sync msg with multiple velocities for different axes
*/
bool setMultiVelocity(std::vector<std::vector<int> > value_tuples)
{              
	int num_actuator = value_tuples.size();
	unsigned char param[num_actuator*(1+4)];
    
    // Make syncwrite packet
    for(int i = 0; i < value_tuples.size(); i++) {

        uint8_t motor_id = value_tuples[i][0];
        int32_t velocity = (int32_t) value_tuples[i][1];
        // int velocity = value_tuples[i][1];
        
        // ROS_INFO("id=%d velocity=%d", motor_id, velocity);

        param[i*(1+4)+0] = (unsigned char)motor_id;
        param[i*(1+4)+1] = DXL_LOBYTE(DXL_LOWORD(velocity));
        param[i*(1+4)+2] = DXL_HIBYTE(DXL_LOWORD(velocity));
        param[i*(1+4)+3] = DXL_LOBYTE(DXL_HIWORD(velocity));
        param[i*(1+4)+4] = DXL_HIBYTE(DXL_HIWORD(velocity));
        }
        
        int result = driver_->SyncWrite(GOAL_VELOCITY, 4, param, num_actuator*(1+4));
        
        usleep(CONTROL_PERIOD*100);
        if( result != COMM_RXSUCCESS ) {
            process_comm_status(result);
            return false;
			}
		else return true;	              
}

/*!     \fn  robotnik_dpro_controller::setHomeOffset(int id, int offset)
  *      Send Home Offset value
*/
bool setHomeOffset(int id, int offset)
{                              
	int error;        
	int result = driver_->WriteDWord(id, HOME_OFFSET , (int32_t) offset, &error); 
    usleep(CONTROL_PERIOD*100);
    if( result != COMM_RXSUCCESS ) {
	   ROS_ERROR("setHomeOffset Error. id=%d", id);		
       process_comm_status(result);
       return false;
	   }
	else return true;	              
}

/*!	\fn robotnik_dpro_controller_node::~robotnik_dpro_controller_node()
 * 	\brief Public destructor
*/
~robotnik_dpro_controller_node(){

    if (driver_!=NULL) {   

		// For each joint 
		for (std::map<std::string, dynamixel_info>::iterator iter = joint2dynamixel_.begin(); iter != joint2dynamixel_.end(); iter++)    
		{
			this->setTorqueEnabled(iter->second.id, 0);
		}
		
		// Disconnect USB2Dynamixel before ending program
		ROS_INFO("robotnik_dpro_controller_node::Disconnecting USB2Dynamixel converter");
		driver_->Disconnect();
		delete driver_;
		}
		
    delete subs_command_freq;    
}

/*!	\fn int robotnik_dpro_controller::start()
 * 	\brief Start Controller
*/
int start(){

    for (std::map<std::string, dynamixel_info>::iterator iter = joint2dynamixel_.begin(); iter != joint2dynamixel_.end(); iter++) {
		int id = iter->second.id;
		ROS_INFO("start:: starting id = %d", id);
        this->setTorqueEnabled(id, 0);                

       // usleep(200000);
		
		// Initialize motor params
		this->setMotorParams(id, iter->second.vel_i_gain, iter->second.vel_p_gain, iter->second.pos_p_gain);
		//iter->second.current_pid_mode = MOVING_PID_MODE;
		
        // Initialize in velocity mode for all pos-vel control commands
		// this->setOperatingMode(id, 1);        
        // this->setAcceleration(id, 4);

        //usleep(200000);

        this->setTorqueEnabled(id, 1);
        id2status_[id].torque_enabled = true;

        //usleep(200000);
	}
	
		        
	freq_diag_.clear();
	running = true;
	return 0;
}


/*!	\fn int robotnik_dpro_controller::stop()
 * 	\brief Stopt Controller
*/
int stop(){

	//! Disable all servomotors
	for (std::map<std::string, dynamixel_info>::iterator iter = joint2dynamixel_.begin(); iter != joint2dynamixel_.end(); iter++)    
	{
		this->setTorqueEnabled(iter->second.id, 0);
	}
}

/*!	\fn int robotnik_dpro_controller::m2rad()
 * 	\brief returns a rad pos corresponding to a meter ref input for a given prismatic axis
*/
double m2rad( double max_rad, double min_rad, double max_m, double min_m, double val_m )
{

	double span_rad = max_rad - min_rad;
	double span_m = max_m - min_m;

	if (span_m <= 0) return 0.0;

        // Check bounds in prismatic axes
	if (val_m>max_m) val_m = max_m;
	if (val_m<min_m) val_m = min_m;

	return ((val_m - min_m) / span_m) * span_rad + min_rad;
}

/*!	\fn int robotnik_dpro_controller::rad2m()
 * 	\brief returns a m pos corresponding to a radian ref input for a given prismatic axis
*/
double rad2m( double max_rad, double min_rad, double max_m, double min_m, double val_rad )
{
	double span_rad = max_rad - min_rad;
	double span_m = max_m - min_m;

	return ((val_rad - min_rad) / span_rad) * span_m + min_m;
}

/*!	\fn int robotnik_dpro_controller::setJointCommands()
 * 	\brief Set Joint values
*/
void setJointCommands()
{	
	if (!bJointCommandReceived_) return;
	
    if (shutting_down_)
        return;
    
    

    bool has_pos = false, has_vel = false, has_torque = false;
    control_mode new_mode = UNKOWN;

    //figure out which value is going to be our setpoint
    if (joint_commands_.position.size() > 0)
        has_pos = true;
    if (joint_commands_.velocity.size() > 0)
        has_vel = true;
    else if (joint_commands_.effort.size() > 0) 
        has_torque = true; 

    //figure out which mode we are going to operate the servos in 
    if (has_pos)
        new_mode = POSITION_CONTROL;
    else if (has_vel)
        new_mode = VELOCITY_CONTROL;
    else if (has_torque) 
        new_mode = TORQUE_CONTROL;

    std::vector<int> ids, velocities, positions, torques;
    std::vector<int> positions_vel; 

	//ROS_INFO("has pos=%d vel=%d torque=%d", has_pos, has_vel, has_torque);

    //actually send the commands to the joints
    for (int i = 0; i < joint_commands_.name.size(); i++)
    {   
		  
        //lookup the information for that particular joint to be able to control it
        std::string name = joint_commands_.name[i];
        dynamixel_info info = joint2dynamixel_[name];
        
        if(joint2dynamixel_[name].dynamic_pid && joint2pidmode_[name] == IDLE_PID_MODE){
			ROS_INFO("setJointCommands servo %s: Changing PID mode", name.c_str());
			this->setMotorParams(joint2dynamixel_[name].id, joint2dynamixel_[name].vel_i_gain, joint2dynamixel_[name].vel_p_gain, joint2dynamixel_[name].pos_p_gain);
			joint2pidmode_[name] = MOVING_PID_MODE;
		}   
       
        // ROS_INFO("i=%d, name=%s, info.id=%d", i, name.c_str(), info.id);
 
        // Command only joints that have been received in the command message
        if (info.id != 0) {
          dynamixel_status &status = id2status_[info.id];

          //enable torque if needed
          if (!status.torque_enabled) {
            this->setTorqueEnabled(info.id, 1);
            ROS_INFO("Enabling torque for id=%d", info.id);            
			}
          status.torque_enabled = true;
        
          //prepare data to be sent to the motor
          ids.push_back(info.id);
          
          if (has_pos) {
			// Ref pos in prismatic and revolute
			if (info.joint_type == "prismatic") {
								
				double rad_pos = m2rad(info.max_rad, info.min_rad, info.max_m, info.min_m, joint_commands_.position[i]);
				int pos = (int) (rad_pos / (2.0 * M_PI) * info.cpr + 0.5);
				positions.push_back(pos);
                                //ROS_INFO("position in m=%5.2f rad_pos=%5.2f", joint_commands_.position[i], rad_pos);
				}
			else {  // info.joint_type == "revolute"
				double rad_pos = joint_commands_.position[i];            
				// Check bounds in revolute axes
                                if (rad_pos > info.max_rad) rad_pos = info.max_rad;
				if (rad_pos < info.min_rad) rad_pos = info.min_rad;
				
				int pos = (int) (rad_pos / (2.0 * M_PI) * info.cpr + 0.5);
				positions.push_back(pos);
				}
			}
	        
          if (has_vel) {
			            			
			// For all Series 
			double rad_s_vel = 0.0;
			if (info.joint_type == "prismatic") {	
				double ms2rads = (info.max_m - info.min_m) / fabs( info.max_rad - info.min_rad );
				if (ms2rads != 0.0) {
					rad_s_vel = joint_commands_.velocity[i] / ms2rads;
					}
				else 
					rad_s_vel = 0.0;
				//ROS_INFO("ms2rads=%5.2f rad_s_vel=%5.2f", ms2rads, rad_s_vel);
				}
			else { // info.joint_type == "revolute"
				rad_s_vel = joint_commands_.velocity[i];            
				}            
                        int vel = (int) ((rad_s_vel / info.max_vel_rads) * info.max_vel_value);
                        if (vel>info.max_vel_value) vel=(int) info.max_vel_value;
                        if (vel<-info.max_vel_value) vel=(int) -info.max_vel_value;
                        //if (info.joint_type == "prismatic") ROS_INFO("vel=%d  rad_s_vel=%5.2f", vel, rad_s_vel);
                        velocities.push_back(vel);
                     
                        // positions_vel ref (safe vel mode)            
                        double rad_pos=0.0;
                        for (int j = 0; j < joint_states_.name.size(); j++) {
				// search in the state, the joint with the same name
				if (name == joint_states_.name[j]) {
				    if (info.joint_type=="prismatic") 
					rad_pos =  m2rad(info.max_rad, info.min_rad, info.max_m, info.min_m, joint_states_.position[j]);
				    else 
                                        rad_pos = joint_states_.position[j];	
					}
				}
            
                        // Divided by real freq, not desired_freq_ 
                        double new_rad_pos;            
                        if (bReceiving_) {
				double spin=1.0;
				if (info.joint_type=="prismatic") {
					if (info.max_rad > info.min_rad) spin = 1.0;
					else spin = -1.0;
					}				

				// adjust references to real frequency
				if (cycle_freq_ > 0.0) new_rad_pos = rad_pos + rad_s_vel * spin / (0.25 * cycle_freq_);
				else new_rad_pos = rad_pos;
				
				// Bound check 
				if (spin==1.0) {
					// Check bounds standard
                                       	if (new_rad_pos > info.max_rad) new_rad_pos = info.max_rad;
                                       	if (new_rad_pos < info.min_rad) new_rad_pos = info.min_rad;
					}
				else {  // Chek bounds inverted
 					if (new_rad_pos < info.max_rad) new_rad_pos = info.max_rad;
                                        if (new_rad_pos > info.min_rad) new_rad_pos = info.min_rad;
				     }
				} // if bReceiving_
			else
				new_rad_pos = rad_pos;	// stop if no new cmds received in safe vel mode				
				
				
                        int pos = (int) (new_rad_pos / (2.0 * M_PI) * info.cpr + 0.5); 
			positions_vel.push_back(pos);                        
			
			// if (info.joint_type == "prismatic") ROS_INFO("rad_s_vel=%5.3f rad_pos=%5.3f  new_rad_pos=%5.3f", rad_s_vel, rad_pos, new_rad_pos);			
			} // if (has_vel)
        
          if (has_torque) {
            //replace the below with proper code when you can figure out the units
            static bool first_run = true;
            if (first_run)
                ROS_WARN("Dynamixel pro controller torque control mode not implemented");
			}
		 }// end_if info.id!=0        
      } // end_for

    //send the setpoints in monolithic packets to reduce bandwidth    
    if (has_pos && has_vel)
    { 		
        //send both a position and a velocity limit
        std::vector< std::vector<int> > data;
        for (int i = 0; i < ids.size(); i++)
        {
            std::vector<int> temp;
            temp.push_back(ids[i]);//order matters here
            temp.push_back(positions[i]);
            temp.push_back(abs(velocities[i])); //velocity limits should always be positive             
            data.push_back(temp);
        }
        this->setMultiPositionVelocity(data);        
    }
    else
    {
        //send only a position setpoint
        if (has_pos)
        {
            std::vector< std::vector<int> > data;
            for (int i = 0; i < ids.size(); i++)
            {
                std::vector<int> temp;
                temp.push_back(ids[i]);
                temp.push_back(positions[i]);
                data.push_back(temp);
            }
            this->setMultiPosition(data);
        }
        
		// Safe velocity mode (implemented as position vel)
        // send both a position and a velocity limit
        if (has_vel)
        {			
			std::vector< std::vector<int> > data;
			for (int i = 0; i < ids.size(); i++)
			{
				std::vector<int> temp;
				temp.push_back(ids[i]);	//order matters here
				temp.push_back(positions_vel[i]);
				temp.push_back(abs(velocities[i])); //velocity limits should always be positive 
				data.push_back(temp);
			}
			this->setMultiPositionVelocity(data);        
		}
               
    }

}

//////////////////////////////////////////////////////////////////////////////////////////////////
// CALLBACKS
// SERVICES
/*!     \fn  robotnik_dpro_controller::cmdJointStateCallback
  *      Receive joint references (joint_states)
*/
void cmdJointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{	
    sensor_msgs::JointState joint_commands = *msg; 
	bool bOK = false;
	
	// For checking if the node is receiving commands
	last_command_time_ = ros::Time::now();
	
	if (joint_commands.name.size()==0) return;
	
	// As there might be several nodes running in parallel, check if the command is for this node.	
    for (int i = 0; i < joint_commands.name.size(); i++) {
        std::string name = joint_commands.name[i];
        dynamixel_info info = joint2dynamixel_[name];
        // if at least there is 1 joint in the list that is in the group, the command is accepted
        if (info.id != 0) bOK = true;
		}
	
	// Use the message only if it is correct
	if (bOK) {
		bJointCommandReceived_ = true;
		bReceiving_ = true;
		joint_commands_ = *msg; 
		}
    return;   
}

/*!     \fn  robotnik_dpro_controller::cmdIOCallback
  *      Receive i/o values (used to limit motion in some axes if configured)
*/

void cmdIOCallback(const robotnik_msgs::inputs_outputs::ConstPtr &msg)
{
	// Just update the global structure
	bool digital_inputs[10]; // MAX_INPUTS
	robotnik_msgs::inputs_outputs io = *msg;
	int num_inputs = io.digital_inputs.size();

        bReceivingIO_ = true;
	
	// For checking if the node is receiving io data
	last_io_time_ = ros::Time::now();

	if (num_inputs==0) return; // this should never happen
	
	for (int i=0; i< num_inputs; i++) digital_inputs[i]=io.digital_inputs[i];
			
	//! Check bounds of each servo axis
	//! If bound detected stop. This could be refined in the future. 
	for (std::map<std::string, dynamixel_info>::iterator iter = joint2dynamixel_.begin(); iter != joint2dynamixel_.end(); iter++) {
		int input = iter->second.lim_sw_pos;
		if ((input!=-1)&&(input<num_inputs)) {
			if (!digital_inputs[input]) {
				ROS_ERROR("cmdIOCallback:: Joint %s - Detected positive limit switch: Stopping.", iter->second.joint_name.c_str());
				this->stop();
				}
			}		
		input = iter->second.lim_sw_neg;
		if ((input!=-1)&&(input<num_inputs)) {
			if (!digital_inputs[input]) {
				ROS_ERROR("cmdIOCallback:: Joint %s - Detected negative limit switch: Stopping.", iter->second.joint_name.c_str());
				this->stop();
				}
			}		
		}
		
	return;
}


/*!     \fn  Service Stop
  *      Stop robot
*/
bool srvCallback_Stop(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("robotnik_dpro_controller:: STOP");
    // Disable all axes
	this->stop();
	
	return true;
}


/*!     \fn  Service SetHomeOffset (TEST)
  *      Stop robot
*/
bool srvCallback_SetHomeOffset(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("robotnik_dpro_controller:: setHomeOffset");
     
	// TEST
	this->setHomeOffset( id_prismatic_, -present_position_prismatic_);
	
	return true;
}



/*!    \fn read_and_publish
 *     read joint states and status
 */
int read_and_publish(){

    int result;
    int val1, val2, val3;
    val1 = val2 = val3 = 0;
    static double position_ant[MAX_JOINTS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
	// Time measurement	
	ros::Time current_time = ros::Time::now();
	double diff = (current_time - last_read_time_).toSec();
	last_read_time_ = current_time;
	if (diff>0.0) cycle_freq_ = 1.0 / diff;
	
    for (int i = 0; i < joint_states_.name.size(); i++) {
        //lookup the information for that particular joint to be able to control it
        std::string name = joint_states_.name[i];
        //dynamixel_info info = joint2dynamixel_[name];
        
        // Read all joints that belong to this node and publish their data
        
		// Fill message    		
		int id = joint2dynamixel_[name].id;
	
		// Read effort, velocity and position from controller 
		int error=0;
		
		// Reading current and velocity consumes a lot of time
		/*
		result = driver_->ReadWord(id, PRESENT_CURRENT, &val1, &error);
		if (result == COMM_RXSUCCESS) joint_states_.effort[i] = val1;
		if (error) {
			ROS_ERROR("read_and_publish Error reading present current. id=%d", id);
			process_error_code( error, id );
			}
		result = driver_->ReadDWord(id, PRESENT_VELOCITY, (long*) &val2, &error);
		if (result == COMM_RXSUCCESS) joint_states_.velocity[i] = (double) val2 / 400.0 * 2.98452;			
		if (error) {
			ROS_ERROR("read_and_publish Error reading present velocity. id=%d", id);
			process_error_code( error, id );
			}
		*/

		joint_states_.effort[i] = 0.0;
		joint_states_.velocity[i] = 0.0;
		
		result = driver_->ReadDWord(id, PRESENT_POSITION, (long*) &val3, &error);
		if (result == COMM_RXSUCCESS) {			
			if (joint2dynamixel_[name].joint_type == "prismatic") {	
				
				// TEST
				id_prismatic_ = id;
				present_position_prismatic_ = val3;
				ROS_INFO("present_position_prismatic_ = %ld", present_position_prismatic_);
							
				double val_rad = ((double) val3 / (double) joint2dynamixel_[name].cpr) * 2.0 * M_PI;				
				joint_states_.position[i]  = rad2m( joint2dynamixel_[name].max_rad, joint2dynamixel_[name].min_rad, joint2dynamixel_[name].max_m, joint2dynamixel_[name].min_m, val_rad );
								//ROS_INFO("val_rad = %5.2f, pos in m = %5.2f", val_rad, joint_states_.position[i]);								
				}
			else {
				joint_states_.position[i] = ((double) val3 / (double) joint2dynamixel_[name].cpr) * 2.0 * M_PI;
				}
		}

		// TODO - store hw_error_status for each joint ! - and send this info also via diagnostics...
		
		// Process hw error status code if error flag activated 
		if (error!=0) {
			int hw_error_status = 0;
			int ret = driver_->ReadByte(id, HW_ERROR_STATUS, (int*) &hw_error_status, &error);
			usleep(CONTROL_PERIOD*100);        
			if (error!=0) process_error_code( hw_error_status, id );						
		}
		
		// REading moving flag
		//bool is_moving = false;
		result = driver_->ReadByte(id, P_MOVING, (int*) &joint2dynamixel_[name].is_moving, &error);
		if (result == COMM_RXSUCCESS) {	
			// if it's moving saves the time
			if(joint2dynamixel_[name].is_moving){
				joint2time_[name] = current_time;
			}else{
				ros::Time t_now = ros::Time::now();
				double t_diff = ( t_now - joint2time_[name]).toSec();
				
				if(t_diff > DPRO_IDLE_TIME){
					//ROS_INFO("servo %s: %.2f without moving. dynamic =%d, mode= %d, command = %d", name.c_str(),t_diff, joint2dynamixel_[name].dynamic_pid, joint2pidmode_[name], bReceiving_);
					if(joint2dynamixel_[name].dynamic_pid && not bReceiving_ && joint2pidmode_[name] == MOVING_PID_MODE){
						ROS_INFO("servo %s:  Changing PID mode", name.c_str());
						this->setMotorParams(id, joint2dynamixel_[name].vel_i_gain_idle, joint2dynamixel_[name].vel_p_gain_idle, joint2dynamixel_[name].pos_p_gain_idle);
						joint2pidmode_[name] = IDLE_PID_MODE;
					}
				}
			}
		}
		
		if (diff>0.0) {			
			joint_states_.velocity[i] = ((double) joint_states_.position[i] - position_ant[i]) / diff;
		}
		position_ant[i] = joint_states_.position[i];
		
		/*std::map<std::string, dynamixel_info>::iterator it = joint2dynamixel_.find(name); 
		if (it != joint2dynamixel_.end())
			it->second = info;*/
		//joint2dynamixel_.at(name) = info;
		usleep(CONTROL_PERIOD*100);
			
	}// end for

	// Publish message	
    joint_states_.header.stamp = ros::Time::now();		    
	joint_state_pub_.publish( joint_states_ );
}

/*!     \fn  check_io_subscriber
  *      If there is an axis with limit switches, check that the node receives 
*/
void check_io_subscriber()
{
	ros::Time current_time = ros::Time::now();
	if (bIODataNeeded_) {
		double diff = (current_time - last_io_time_).toSec();
		if (bReceivingIO_) {
			if(diff > 0.25){
				ROS_ERROR("check_io_subscriber - not receiving IO data - Stopping");
				this->stop();
				}
			}
		else {
                double diff1 = (current_time - startup_time_).toSec();
		        if(diff1 > 5.0) {
				ROS_ERROR("check_io_subscriber - did not start receiving IO data");
				this->stop();
				}
			}
		}

}

/*!     \fn  Spin
  *      Main Loop
*/
bool spin()
{
    ros::Rate r(desired_freq_);
    
    while (!(shutting_down_=ros::isShuttingDown())) // Using ros::isShuttingDown to avoid restarting the node during a shutdown.
    {		
      if (start() == 0)
      {
		  while(ros::ok() && node_handle_.ok()) {
					  
			  read_and_publish();
							   
			  check_io_subscriber();
			  
			  if (bJointCommandReceived_ && bReceiving_) setJointCommands();
				
			  self_test_.checkTest();
			  diagnostic_.update();
			  ros::spinOnce();
			  r.sleep();
			  }
		   ROS_INFO("robotnik_dpro_controller::spin - END OF ros::ok() !!!");
       } else {
		   // No need for diagnostic here since a broadcast occurs in start
		   // when there is an error.
		   usleep(1000000);
		   self_test_.checkTest();
		   ros::spinOnce();
       
      }
   }

   return true;
}

		
}; // class robotnik_dpro_controller_node

// MAIN
int main(int argc, char** argv)
{
	ros::init(argc, argv, "robotnik_dpro_controller_node");
	
	ros::NodeHandle n;		
  	robotnik_dpro_controller_node ctrl(n);

  	ctrl.spin();

	return (0);
}
// EOF
