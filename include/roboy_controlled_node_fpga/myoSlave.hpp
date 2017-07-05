#pragma once

#include <vector>
#include <map>
#include <iostream>
#include <math.h>
#include <chrono>
#include <fstream>
#include "myoControlRegister.hpp"

#include "xap.h"

#include <oplk/oplk.h>
#include <oplk/debugstr.h>

#include <system/system.h>
#include <obdcreate/obdcreate.h>
#include <console/console.h>
#include <eventlog/eventlog.h>
#include <limits.h>

#if defined(CONFIG_USE_PCAP)
#include <pcap/pcap-console.h>
#endif

#include <stdio.h>
#include <limits.h>

#include <user/sdoudp.h>

#include <arpa/inet.h>
#include <user/sdoudp.h>
#include <oplk/frame.h>

#define CYCLE_LEN         50000
#define NODEID            1                   // could be changed by command param
#define IP_ADDR           0xc0a86401          // 192.168.100.1
#define DEFAULT_GATEWAY   0xC0A864FE          // 192.168.100.C_ADR_RT1_DEF_NODE_ID
#define SUBNET_MASK       0xFFFFFF00          // 255.255.255.0

#undef min
#undef max

#include <ros/ros.h>
#include <roboy_communication_middleware/JointStatus.h>
#include <roboy_communication_middleware/MotorConfig.h>
#include <roboy_communication_middleware/MotorConfigService.h>
#include <roboy_communication_middleware/MotorStatus.h>
#include <roboy_communication_middleware/MotorCommand.h>
#include <roboy_communication_middleware/MotorRecord.h>
#include <roboy_communication_middleware/MotorRecordConfig.h>
#include <roboy_communication_middleware/MotorTrajectoryControl.h>
#include <common_utilities/CommonDefinitions.h>
#include "roboy_controlled_node_fpga/timer.hpp"
#include "roboy_controlled_node_fpga/am4096.hpp"
#include <bitset>

using namespace std;
using namespace std::chrono;

typedef struct
{
    UINT32          nodeId;
    tEventlogFormat logFormat;
    UINT32          logLevel;
    UINT32          logCategory;
    char            devName[128];
} tOptions;

class MyoSlave{
public:
	MyoSlave(vector<int32_t*> &myo_base, vector<int32_t*> &i2c_base, vector<int> &deviceIDs, int argc, char* argv[]);
	~MyoSlave();
	/**
	 * This is the main loop, receiving commands and sending motor status via powerlink
	 */
	void mainLoop();
	/**
	 * Changes the controller of a motor
	 * @param motor for this motor
	 * @param mode choose from Position, Velocity or Displacement
	 * @param params with these controller parameters
	 */
	static void changeControl(int motor, control_Parameters_t &params);
    /**
    * Changes the controller of a motor with the saved controller parameters
    * @param motor for thsi motor
    * @param mode choose from Position, Velocity or Displacement
    */
    static void changeControl(int motor, int mode);
	/**
	 * Changes the controller of ALL motors with the saved controller parameters
	 * @param mode choose from Position, Velocity or Displacement
	 */
	static void changeControl(int mode);
	/**
	 * Toggles SPI transmission
	 * @return on/off
	 */
	static void toggleSPI(bool active);
	/**
	 * Resets all myo controllers
	 */
	static void reset();
	/**
	 * Changes setpoint for position controller
	 * @param motor for this motor
	 * @param position the new setpoint
	 */
	static void setPosition(int motor, int32_t position);
	/**
	 * Changes setpoint for velocity controller
	 * @param motor for this motor
	 * @param velocity the new setpoint
	 */
	static void setVelocity(int motor, int32_t velocity);
	/**
	 * Changes setpoint for displacement controller
	 * @param motor for this motor
	 * @param displacement the new setpoint
	 */
	static void setDisplacement(int motor, int32_t displacement);
	/**
	 * Get the parameters for the PID controller of a motor
	 * @param motor for this motor
	 */
	static void getPIDcontrollerParams(int &Pgain, int &Igain, int &Dgain, int &forwardGain, int &deadband,
									int &setPoint, int &setPointMin, int &setPointMax, int motor);
	/**
	 * Gets the current control_mode of a motor
	 * @param motor for this motor
	 */
	static uint16_t getControlMode(int motor);
	/**
	 * Gets the current pwm of a motor
	 * @param motor for this motor
	 */
    static int16_t getPWM(int motor);
	/**
	 * Gets the current position of a motor in radians
	 * @param motor for this motor
	 */
    static int32_t getPosition(int motor);
	/**
	 * Gets the current velocity of a motor in radians/seconds
	 * @param motor for this motor
	 */
    static int16_t getVelocity(int motor);
	/**
	 * Gets the displacement in encoder ticks
	 * @param motor for this motor
	 */
    static int16_t getDisplacement(int motor);
	/**
	 * Gets the current in Ampere
	 * @param motor for this motor
	 */
    static int16_t getCurrent(int motor);

	/**
	 * Fills the given params with default values for the corresponding control mode
	 * @param params pointer to control struct
	 * @param control_mode Position, Velocity, Force
	 */
	static void getDefaultControlParams(control_Parameters_t *params, int control_mode);

	/**
	 * Changes the control mode for all motors to Position
	 * @param pos new setPoint
	 */
	static void allToPosition(int32_t pos);
	/**
	 * Changes the control mode for all motors to Velocity
	 * @param pos new setPoint
	 */
	static void allToVelocity(int32_t vel);
	/**
	 * Changes the control mode for all motors to Displacement
	 * @param force new setPoint
	 */
	static void allToDisplacement(int32_t displacement);
	/**
	 * Zeros the current weight
	 */
	void zeroWeight();
	/**
	 * Returns the current weight according to adc_weight_parameters
	 */
	float getWeight();
	/**
	 * Estimates the spring parameters of a motor by pulling with variable forces
	 * keeping track of displacement and weight, it will either timeout or stop when the
	 * requested number of samples was reached
	 * @param motor for this motor
	 * @param timeout in milliseconds
	 * @param numberOfDataPoints how many samples do you wanne collect
	 */
	void estimateSpringParameters(int motor, int timeout, uint numberOfDataPoints);
	/**
	 * Performs polynomial regression (http://www.bragitoff.com/2015/09/c-program-for-polynomial-fit-least-squares/)
	 * @param degree (e.g. 2 -> a * x^0 + b * x^1 + c * x^2)
	 * @param coeffs the estimated coefficients
	 * @param X the x-data
	 * @param Y the y-data
	 */
	static void polynomialRegression(int degree, vector<float> &x, vector<float> &y,
			vector<float> &coeffs);

	static map<int,map<int,control_Parameters_t>> control_params;
	uint32_t *adc_base = nullptr;
	float weight_offset = 0;
	float adc_weight_parameters[2] = {830.7, -0.455};
	bool spi_active = false;
	static uint numberOfMotors;
private:
	void MotorConfig(const roboy_communication_middleware::MotorConfig::ConstPtr &msg);
	bool MotorConfigService(roboy_communication_middleware::MotorConfigService::Request  &req,
					 roboy_communication_middleware::MotorConfigService::Response &res);
	void recordMotors(const roboy_communication_middleware::MotorRecordConfig::ConstPtr &msg);
	void trajectoryControl(const roboy_communication_middleware::MotorTrajectoryControl::ConstPtr &msg);
	void trajectoryPlayback(const roboy_communication_middleware::MotorRecord::ConstPtr &msg);

    /**
     * This initializes the process image for openPowerLink
     * @return errorCode
     */
    tOplkError initProcessImage();
    /**
     * The function parses the supplied command line parameters and stores the
     * options at pOpts_p.
     * @param argc_p Argument count.
     * @param argv_p Pointer to arguments.
     * @param pOpts_p Pointer to store options
     * @return The function returns the parsing status.
     * @retval 0 Successfully parsed
     * @retval -1 Parsing error
    */
    int getOptions(int argc_p,
                   char* const argv_p[],
                   tOptions* pOpts_p);
    /**
     * The function initializes the openPOWERLINK stack.
     * @param cycleLen_p          Length of POWERLINK cycle.
     * @param devName_p           Device name string.
     * @param macAddr_p           MAC address to use for POWERLINK interface.
     * @param nodeId_p            POWERLINK node ID.
     * @return The function returns a tOplkError error code.
    */
    tOplkError initPowerlink(UINT32 cycleLen_p,
                             const char* devName_p,
                             const UINT8* macAddr_p,
                             UINT32 nodeId_p);
    /**
     * The function implements the synchronous data handler.
     * @return The function returns a tOplkError error code.
    */
    static tOplkError processSync();
    /** Shuts down powerLink */
    void shutdownPowerlink();
    /**
     * The function processes error and warning events.
     * @param  EventType_p         Type of event
     * @param  pEventArg_p         Pointer to union which describes the event in detail
     * @param  pUserArg_p          User specific argument
     * @return The function returns a tOplkError error code.
    */
    static tOplkError processEvents(tOplkApiEventType EventType_p,
                             const tOplkApiEventArg* pEventArg_p,
                             void* pUserArg_p);
    /**
     * The function processes PDO change events.
     * @param  EventType_p         Type of event
     * @param  pEventArg_p         Pointer to union which describes the event in detail
     * @param  pUserArg_p          User specific argument
     * @return The function returns a tOplkError error code.
    */
    static tOplkError processPdoChangeEvent(tOplkApiEventType EventType_p,
                                     const tOplkApiEventArg* pEventArg_p,
                                     void* pUserArg_p);
    /**
     * The function processes error and warning events.
     * @param  EventType_p         Type of event
     * @param  pEventArg_p         Pointer to union which describes the event in detail
     * @param  pUserArg_p          User specific argument
     * @return The function returns a tOplkError error code.
    */
    static tOplkError processErrorWarningEvent(tOplkApiEventType EventType_p,
                                        const tOplkApiEventArg* pEventArg_p,
                                        void* pUserArg_p);
    /**
     * The function processes state change events.
     * @param  EventType_p         Type of event
     * @param  pEventArg_p         Pointer to union which describes the event in detail
     * @param  pUserArg_p          User specific argument
     * @return The function returns a tOplkError error code.
    */
    static tOplkError processStateChangeEvent(tOplkApiEventType EventType_p,
                                       const tOplkApiEventArg* pEventArg_p,
                                       void* pUserArg_p);

    static tOplkError processSDO(tSdoConHdl conHdl_p,
                                 const tAsySdoSeq* pSdoSeqData_p,
                                 UINT dataSize_p);
public:
    static vector<int32_t*> myo_base;
	static vector<int32_t*> i2c_base;
	static vector<boost::shared_ptr<AM4096>> jointAngle;
    static PI_IN*   pProcessImageIn_l;
    static PI_OUT*  pProcessImageOut_l;
private:
	ros::NodeHandlePtr nh;
	ros::Subscriber motorConfig, motorRecordConfig, motorTrajectoryControl, motorTrajectory;
	ros::ServiceServer motorConfig_srv;
	static ros::Publisher motorStatus, jointStatus;
	ros::Publisher motorRecord;
	static bool stopRecord, recording, playback;
	Timer timer;
	boost::shared_ptr<ros::AsyncSpinner> spinner;
    bool stop = false, rewind = false, pause = false, loop = false;
	static int powerlink_state;
};
