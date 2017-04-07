#include <UDPSocket.hpp>
#include "myoSlave.hpp"

static BOOL* pfGsOff_l;

vector<int32_t*> MyoSlave::myo_base;
PI_IN* MyoSlave::pProcessImageIn_l;
PI_OUT* MyoSlave::pProcessImageOut_l;
UDPSocket *MyoSlave::motorConfigSocket;
UDPSocket *MyoSlave::motorInfoSocket;
uint MyoSlave::numberOfMotors;
map<int,map<int,control_Parameters_t>> MyoSlave::control_params;
ros::Publisher MyoSlave::motorStatus;

MyoSlave::MyoSlave(vector<int32_t*> &myobase, int argc, char* argv[]){
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "roboy_controlled_node_fpga",
                  ros::init_options::NoSigintHandler |
                  ros::init_options::NoRosout);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);
//
    motorConfig = nh->subscribe("/roboy/MotorConfig", 1, &MyoSlave::MotorConfig, this);
    motorStatus = nh->advertise<communication::MotorStatus>("/roboy/MotorStatus", 1);
    communication::MotorStatus msg;
    msg.current.push_back(0);
    msg.current.push_back(1);
    msg.current.push_back(2);
    msg.displacement.push_back(0);
    msg.displacement.push_back(1);
    msg.displacement.push_back(2);
    msg.velocity.push_back(0);
    msg.velocity.push_back(1);
    msg.velocity.push_back(2);
    msg.position.push_back(0);
    msg.position.push_back(1);
    msg.position.push_back(2);
    msg.pwmRef.push_back(0);
    msg.pwmRef.push_back(1);
    msg.pwmRef.push_back(2);
    ros::Rate rate(1);
//    while(true){
//        ROS_INFO("sending motor status");
//        motorStatus.publish(msg);
//        ros::spinOnce();
//        rate.sleep();
//    }

    myo_base = myobase;
    reset();
    toggleSPI(false);
	// initialize control mode
	numberOfMotors = myo_base.size()*MOTORS_PER_MYOCONTROL;
	// initialize all controllers with default values
	control_Parameters_t params;
	getDefaultControlParams(&params, POSITION);
	for(uint motor=0;motor<numberOfMotors;motor++){
		changeControl(motor, params);
		control_params[motor][POSITION] = params;
	}
	getDefaultControlParams(&params, VELOCITY);
	for(uint motor=0;motor<numberOfMotors;motor++){
		control_params[motor][VELOCITY] = params;
	}
	getDefaultControlParams(&params, DISPLACEMENT);
	for(uint motor=0;motor<numberOfMotors;motor++){
		control_params[motor][DISPLACEMENT] = params;
	}

	bool powerlink_initialized = true;

	tOplkError  ret = kErrorOk;
	tOptions    opts;

	if (getOptions(argc, argv, &opts) < 0)
		powerlink_initialized = false;

	if (system_init() != 0)
	{
		fprintf(stderr, "Error initializing system!");
		powerlink_initialized = false;
	}

    pfGsOff_l = new unsigned char;

	eventlog_init(opts.logFormat,
				  opts.logLevel,
				  opts.logCategory,
				  (tEventlogOutputCb)console_printlogadd);

	printf("----------------------------------------------------\n");
	printf("openPOWERLINK myoControl CN\n");
	printf("Using openPOWERLINK stack: %s\n", oplk_getVersionString());
	printf("----------------------------------------------------\n");

	eventlog_printMessage(kEventlogLevelInfo,
						  kEventlogCategoryGeneric,
						  "demo_cn_console: Stack version:%s Stack configuration:0x%08X",
						  oplk_getVersionString(),
						  oplk_getStackConfiguration());

    const BYTE aMacAddr_l[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	ret = initPowerlink(CYCLE_LEN, opts.devName, aMacAddr_l, opts.nodeId);

	if (ret != kErrorOk)
		powerlink_initialized = false;

	ret = initProcessImage();
	if (ret != kErrorOk)
		powerlink_initialized = false;

    motorConfigSocket = new UDPSocket("192.168.0.100", 8000);
    motorInfoSocket = new UDPSocket("192.168.0.100", 8002);

    if(powerlink_initialized)
		mainLoop();
}

MyoSlave::~MyoSlave(){
	cout << "shutting down myoControl" << endl;
    oplk_freeProcessImage();
	shutdownPowerlink();
	system_exit();
}

void MyoSlave::mainLoop(){
	tOplkError  ret;
	char        cKey = 0;
	BOOL        fExit = FALSE;

#if !defined(CONFIG_KERNELSTACK_DIRECTLINK)

	#if defined(CONFIG_USE_SYNCTHREAD)
    system_startSyncThread(processSync);
#endif

#endif

	// start processing
	ret = oplk_execNmtCommand(kNmtEventSwReset);
	if (ret != kErrorOk)
		return;

	printf("Start POWERLINK stack... ok\n");
	printf("myoControl interface with openPOWERLINK is ready!\n");
	printf("\n-------------------------------\n");
	printf("Press Esc to leave the program\n");
	printf("Press r to reset the node\n");
	printf("-------------------------------\n\n");

	// wait for key hit
	while (!fExit)
	{
		if (console_kbhit())
		{
			cKey = (char)console_getch();

			switch (cKey)
			{
				case 'r':
					ret = oplk_execNmtCommand(kNmtEventSwReset);
					if (ret != kErrorOk)
						fExit = TRUE;
					break;

				case 0x1B:
					fExit = TRUE;
					break;

				default:
					break;
			}
		}

		if (system_getTermSignalState() == TRUE)
		{
			fExit = TRUE;
			printf("Received termination signal, exiting...\n");
			eventlog_printMessage(kEventlogLevelInfo,
								  kEventlogCategoryControl,
								  "Received termination signal, exiting...");
		}

		if (oplk_checkKernelStack() == FALSE)
		{
			fExit = TRUE;
			fprintf(stderr, "Kernel stack has gone! Exiting...\n");
			eventlog_printMessage(kEventlogLevelFatal,
								  kEventlogCategoryControl,
								  "Kernel stack has gone! Exiting...");
		}

#if (defined(CONFIG_USE_SYNCTHREAD) || \
     defined(CONFIG_KERNELSTACK_DIRECTLINK))
		system_msleep(100);
#else
		processSync();
#endif
	}

#if (TARGET_SYSTEM == _WIN32_)
	printf("Press Enter to quit!\n");
    console_getch();
#endif
}

void MyoSlave::MotorConfig(const communication::MotorConfig::ConstPtr &msg){
    control_Parameters_t params;
    for(auto motor:msg->motors){
        params.control_mode = msg->control_mode[motor];
        params.outputPosMax = msg->outputPosMax[motor];
        params.outputNegMax = msg->outputNegMax[motor];
        params.spPosMax = msg->spPosMax[motor];
        params.spNegMax = msg->spNegMax[motor];
        params.Kp = msg->Kp[motor];
        params.Ki = msg->Ki[motor];
        params.Kd = msg->Kd[motor];
        params.forwardGain = msg->forwardGain[motor];
        params.deadBand = msg->deadBand[motor];
        params.IntegralPosMax = msg->IntegralPosMax[motor];
        params.IntegralNegMax = msg->IntegralNegMax[motor];
        changeControl(motor, params);
    }
}

tOplkError MyoSlave::initProcessImage(){
    tOplkError      ret = kErrorOk;
    UINT            varEntries;
    tObdSize        obdSize;

    /* Allocate process image */
    printf("Initializing process image...\n");
    printf("Size of input process image: %ld\n", sizeof(PI_IN));
    printf("Size of output process image: %ld\n", sizeof(PI_OUT));
    ret = oplk_allocProcessImage(sizeof(PI_IN), sizeof(PI_OUT));
    if (ret != kErrorOk)
    {
        return ret;
    }

    pProcessImageIn_l = (PI_IN*)oplk_getProcessImageIn();
    pProcessImageOut_l = (PI_OUT*)oplk_getProcessImageOut();

    /* link process variables used by CN to object dictionary */
    fprintf(stderr, "Linking process image vars:\n");

    varEntries = 1;
    ret &= oplk_linkProcessImageObject(0x6000, 0x01, 0, FALSE, 4, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6000, 0x02, 4, FALSE, 4, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6000, 0x03, 8, FALSE, 4, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6000, 0x04, 12, FALSE, 4, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6000, 0x05, 16, FALSE, 4, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6000, 0x06, 20, FALSE, 4, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6000, 0x07, 24, FALSE, 4, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6000, 0x08, 28, FALSE, 4, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6000, 0x09, 32, FALSE, 4, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6000, 0x0A, 36, FALSE, 4, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6000, 0x0B, 40, FALSE, 4, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6000, 0x0C, 44, FALSE, 4, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6000, 0x0D, 48, FALSE, 4, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6000, 0x0E, 52, FALSE, 4, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6000, 0x0F, 56, FALSE, 4, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6000, 0x10, 60, FALSE, 4, &varEntries);
    varEntries = 1;
    ret &= oplk_linkProcessImageObject(0x6001, 0x01, 0, TRUE, 2, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6001, 0x02, 2, TRUE, 2, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6001, 0x03, 4, TRUE, 2, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6001, 0x04, 6, TRUE, 2, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6001, 0x05, 8, TRUE, 2, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6001, 0x06, 10, TRUE, 2, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6001, 0x07, 12, TRUE, 2, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6001, 0x08, 14, TRUE, 2, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6001, 0x09, 16, TRUE, 2, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6001, 0x0A, 18, TRUE, 2, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6001, 0x0B, 20, TRUE, 2, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6001, 0x0C, 22, TRUE, 2, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6001, 0x0D, 24, TRUE, 2, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6001, 0x0E, 26, TRUE, 2, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6001, 0x0F, 28, TRUE, 2, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6001, 0x10, 30, TRUE, 2, &varEntries);
    if (ret != kErrorOk)
    {
        fprintf(stderr, "linking process vars failed with \"%s\" (0x%04x)\n", debugstr_getRetValStr(ret), ret);
        return ret;
    }
    fprintf(stderr, "Linking process vars... ok\n\n");

    return kErrorOk;
}

tOplkError MyoSlave::initPowerlink(UINT32 cycleLen_p, const char* devName_p, const UINT8* macAddr_p, UINT32 nodeId_p){
    tOplkError          ret = kErrorOk;
    tOplkApiInitParam   initParam;
    static char         devName[128];

    printf("Initializing openPOWERLINK stack...\n");
    eventlog_printMessage(kEventlogLevelInfo,
                          kEventlogCategoryControl,
                          "Initializing openPOWERLINK stack");

#if defined(CONFIG_USE_PCAP)
    eventlog_printMessage(kEventlogLevelInfo,
                          kEventlogCategoryGeneric,
                          "Using libpcap for network access");
    if (devName_p[0] == '\0')
    {
        if (selectPcapDevice(devName) < 0)
            return kErrorIllegalInstance;
    }
    else
        strncpy(devName, devName_p, 128);
#else
    UNUSED_PARAMETER(devName_p);
#endif

    memset(&initParam, 0, sizeof(initParam));
    initParam.sizeOfInitParam = sizeof(initParam);

    // pass selected device name to Edrv
    initParam.hwParam.pDevName = devName;
    initParam.nodeId = nodeId_p;
    initParam.ipAddress = (0xFFFFFF00 & IP_ADDR) | initParam.nodeId;

    /* write 00:00:00:00:00:00 to MAC address, so that the driver uses the real hardware address */
    memcpy(initParam.aMacAddress, macAddr_p, sizeof(initParam.aMacAddress));

    initParam.fAsyncOnly              = FALSE;
    initParam.featureFlags            = UINT_MAX;
    initParam.cycleLen                = cycleLen_p;             // required for error detection
    initParam.isochrTxMaxPayload      = C_DLL_ISOCHR_MAX_PAYL;  // const
    initParam.isochrRxMaxPayload      = C_DLL_ISOCHR_MAX_PAYL;  // const
    initParam.presMaxLatency          = 50000;                  // const; only required for IdentRes
    initParam.preqActPayloadLimit     = 48;                     // required for initialization (+28 bytes)
    initParam.presActPayloadLimit     = 60;                     // required for initialization of Pres frame (+28 bytes)
    initParam.asndMaxLatency          = 150000;                 // const; only required for IdentRes
    initParam.multiplCylceCnt         = 0;                      // required for error detection
    initParam.asyncMtu                = 1500;                   // required to set up max frame size
    initParam.prescaler               = 2;                      // required for sync
    initParam.lossOfFrameTolerance    = 500000;
    initParam.asyncSlotTimeout        = 3000000;
    initParam.waitSocPreq             = 1000;
    initParam.deviceType              = UINT_MAX;               // NMT_DeviceType_U32
    initParam.vendorId                = UINT_MAX;               // NMT_IdentityObject_REC.VendorId_U32
    initParam.productCode             = UINT_MAX;               // NMT_IdentityObject_REC.ProductCode_U32
    initParam.revisionNumber          = UINT_MAX;               // NMT_IdentityObject_REC.RevisionNo_U32
    initParam.serialNumber            = UINT_MAX;               // NMT_IdentityObject_REC.SerialNo_U32
    initParam.applicationSwDate       = 0;
    initParam.applicationSwTime       = 0;
    initParam.subnetMask              = SUBNET_MASK;
    initParam.defaultGateway          = DEFAULT_GATEWAY;
    sprintf((char*)initParam.sHostname, "%02x-%08x", initParam.nodeId, initParam.vendorId);
    initParam.syncNodeId              = C_ADR_SYNC_ON_SOA;
    initParam.fSyncOnPrcNode          = FALSE;

    // set callback functions
    initParam.pfnCbEvent = &MyoSlave::processEvents;

#if defined(CONFIG_KERNELSTACK_DIRECTLINK)
    initParam.pfnCbSync = (tSyncCb)&MyoSlave::processSync;
#else
    initParam.pfnCbSync = NULL;
#endif

    // Initialize object dictionary
    ret = obdcreate_initObd(&initParam.obdInitParam);
    if (ret != kErrorOk)
    {
        fprintf(stderr,
                "obdcreate_initObd() failed with \"%s\" (0x%04x)\n",
                debugstr_getRetValStr(ret),
                ret);
        eventlog_printMessage(kEventlogLevelFatal,
                              kEventlogCategoryControl,
                              "obdcreate_initObd() failed with \"%s\" (0x%04x)\n",
                              debugstr_getRetValStr(ret),
                              ret);
        return ret;
    }

    // initialize POWERLINK stack
    ret = oplk_initialize();
    if (ret != kErrorOk)
    {
        fprintf(stderr,
                "oplk_initialize() failed with \"%s\" (0x%04x)\n",
                debugstr_getRetValStr(ret),
                ret);
        eventlog_printMessage(kEventlogLevelFatal,
                              kEventlogCategoryControl,
                              "oplk_initialize() failed with \"%s\" (0x%04x)\n",
                              debugstr_getRetValStr(ret),
                              ret);
        return ret;
    }

    ret = oplk_create(&initParam);
    if (ret != kErrorOk)
    {
        fprintf(stderr,
                "oplk_create() failed with \"%s\" (0x%04x)\n",
                debugstr_getRetValStr(ret),
                ret);
        eventlog_printMessage(kEventlogLevelFatal,
                              kEventlogCategoryControl,
                              "oplk_create() failed with \"%s\" (0x%04x)\n",
                              debugstr_getRetValStr(ret),
                              ret);
        return ret;
    }

    return kErrorOk;
}

tOplkError MyoSlave::processSync(){
    tOplkError      ret = kErrorOk;

    if (oplk_waitSyncEvent(100000) != kErrorOk)
        return ret;

    ret = oplk_exchangeProcessImageOut();
    if (ret != kErrorOk)
        return ret;

    // apply the setPoints for every motor from process image
    setDisplacement(0,pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_1);
    setDisplacement(1,pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_2);
    setDisplacement(2,pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_3);
    setDisplacement(3,pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_4);
    setDisplacement(4,pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_5);
    setDisplacement(5,pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_6);
    setDisplacement(6,pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_7);
    setDisplacement(7,pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_8);
    setDisplacement(8,pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_9);
    setDisplacement(9,pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_10);
    setDisplacement(10,pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_11);
    setDisplacement(11,pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_12);
    setDisplacement(12,pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_13);
    setDisplacement(13,pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_14);

    // write motor info
    pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_1 = getDisplacement(0);
    pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_2 = getDisplacement(1);
    pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_3 = getDisplacement(2);
    pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_4 = getDisplacement(3);
    pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_5 = getDisplacement(4);
    pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_6 = getDisplacement(5);
    pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_7 = getDisplacement(6);
    pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_8 = getDisplacement(7);
    pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_9 = getDisplacement(8);
    pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_10 = getDisplacement(9);
    pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_11 = getDisplacement(10);
    pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_12 = getDisplacement(11);
    pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_13 = getDisplacement(12);
    pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_14 = getDisplacement(13);

    ret = oplk_exchangeProcessImageIn();

    static int counter = 0;
    if((counter++)%50==0) {
        communication::MotorStatus msg;
        for (uint motor = 0; motor < 14; motor++) {
            msg.pwmRef.push_back(getPWM(motor));
            msg.position.push_back(getPosition(motor));
            msg.velocity.push_back(getVelocity(motor));
            msg.displacement.push_back(getDisplacement(motor));
            msg.current.push_back(getCurrent(motor));
        }
        motorStatus.publish(msg);
        ros::spinOnce();
    }

    return ret;
}

void MyoSlave::shutdownPowerlink(){
    UINT    i;

    BOOL fGsOff_l = FALSE;

#if (!defined(CONFIG_KERNELSTACK_DIRECTLINK) && \
     defined(CONFIG_USE_SYNCTHREAD))
    system_stopSyncThread();
    system_msleep(100);
#endif

    // halt the NMT state machine so the processing of POWERLINK frames stops
    oplk_execNmtCommand(kNmtEventSwitchOff);

    // small loop to implement timeout waiting for thread to terminate
    for (i = 0; i < 1000; i++)
    {
        if (fGsOff_l)
            break;
    }

    printf("Stack is in state off ... Shutdown\n");
    eventlog_printMessage(kEventlogLevelInfo,
                          kEventlogCategoryControl,
                          "Stack is in state off ... Shutdown openPOWERLINK");

    oplk_destroy();
    oplk_exit();
}

tOplkError MyoSlave::processEvents(tOplkApiEventType EventType_p,
                         const tOplkApiEventArg* pEventArg_p,
                         void* pUserArg_p){
    tOplkError          ret = kErrorOk;

    UNUSED_PARAMETER(pUserArg_p);

    switch (EventType_p)
    {
        case kOplkApiEventNmtStateChange:
            ret = processStateChangeEvent(EventType_p, pEventArg_p, pUserArg_p);
            break;

        case kOplkApiEventCriticalError:
        case kOplkApiEventWarning:
            ret = processErrorWarningEvent(EventType_p, pEventArg_p, pUserArg_p);
            break;

        case kOplkApiEventPdoChange:
            ret = processPdoChangeEvent(EventType_p, pEventArg_p, pUserArg_p);
            break;

        default:
            break;
    }
    return ret;
}

tOplkError MyoSlave::processStateChangeEvent(tOplkApiEventType EventType_p,
                                               const tOplkApiEventArg* pEventArg_p,
                                               void* pUserArg_p){
    tOplkError                  ret = kErrorOk;
    const tEventNmtStateChange*       pNmtStateChange = &pEventArg_p->nmtStateChange;

    UNUSED_PARAMETER(EventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    if (pfGsOff_l == NULL)
    {
        return kErrorGeneralError;
    }

    switch (pNmtStateChange->newNmtState)
    {
        case kNmtGsOff:
            // NMT state machine was shut down,
            ret = kErrorShutdown;

            console_printlog("StateChangeEvent:kNmtGsOff originating event = 0x%X (%s)\n",
                             pNmtStateChange->nmtEvent,
                             debugstr_getNmtEventStr(pNmtStateChange->nmtEvent));

            // signal that stack is off
            *pfGsOff_l = TRUE;
            break;

        case kNmtGsInitialising:
        case kNmtGsResetApplication:
        case kNmtGsResetConfiguration:
        case kNmtGsResetCommunication:
        case kNmtCsNotActive:
            toggleSPI(false);
            break;
        case kNmtCsPreOperational1:         // handling of
        case kNmtCsStopped:                 // different
        case kNmtCsPreOperational2:         // states here
        case kNmtCsReadyToOperate:
        case kNmtCsOperational:
            reset();
            console_printlog("myoFPGA operational StateChangeEvent(0x%X) originating event = 0x%X (%s)\n",
                             pNmtStateChange->newNmtState,
                             pNmtStateChange->nmtEvent,
                             debugstr_getNmtEventStr(pNmtStateChange->nmtEvent));
            toggleSPI(true);
            break;
        case kNmtCsBasicEthernet:           // no break;

        default:
            console_printlog("StateChangeEvent(0x%X) originating event = 0x%X (%s)\n",
                             pNmtStateChange->newNmtState,
                             pNmtStateChange->nmtEvent,
                             debugstr_getNmtEventStr(pNmtStateChange->nmtEvent));
            break;
    }

    return ret;
}

tOplkError MyoSlave::processSDO(tSdoConHdl conHdl_p,
                      const tAsySdoSeq* pSdoSeqData_p,
                      UINT dataSize_p){

    printf("received UDP %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c \n", &pSdoSeqData_p->sdoSeqPayload.aCommandData[0]);

}

tOplkError MyoSlave::processErrorWarningEvent(tOplkApiEventType EventType_p,
                                                const tOplkApiEventArg* pEventArg_p,
                                                void* pUserArg_p){
    // error or warning occurred within the stack or the application
    // on error the API layer stops the NMT state machine

    const tEventError*            pInternalError = &pEventArg_p->internalError;

    UNUSED_PARAMETER(EventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    console_printlog("Err/Warn: Source = %s (%02X) OplkError = %s (0x%03X)\n",
                     debugstr_getEventSourceStr(pInternalError->eventSource),
                     pInternalError->eventSource,
                     debugstr_getRetValStr(pInternalError->oplkError),
                     pInternalError->oplkError);

    // check additional argument
    switch (pInternalError->eventSource)
    {
        case kEventSourceEventk:
        case kEventSourceEventu:
            // error occurred within event processing
            // either in kernel or in user part
            console_printlog(" OrgSource = %s %02X\n",
                             debugstr_getEventSourceStr(pInternalError->errorArg.eventSource),
                             pInternalError->errorArg.eventSource);
            break;

        case kEventSourceDllk:
            // error occurred within the data link layer (e.g. interrupt processing)
            // the DWORD argument contains the DLL state and the NMT event
            console_printlog(" val = %X\n", pInternalError->errorArg.uintArg);
            break;

        default:
            console_printlog("\n");
            break;
    }
    return kErrorOk;
}

tOplkError MyoSlave::processPdoChangeEvent(tOplkApiEventType EventType_p,
                                             const tOplkApiEventArg* pEventArg_p,
                                             void* pUserArg_p){
    const tOplkApiEventPdoChange*     pPdoChange = &pEventArg_p->pdoChange;
    UINT                        subIndex;
    UINT64                      mappObject;
    tOplkError                  ret;
    UINT                        varLen;

    UNUSED_PARAMETER(EventType_p);
    UNUSED_PARAMETER(pUserArg_p);

    console_printlog("PDO change event: (%sPDO = 0x%X to node 0x%X with %d objects %s)\n",
                     (pPdoChange->fTx ? "T" : "R"), pPdoChange->mappParamIndex,
                     pPdoChange->nodeId, pPdoChange->mappObjectCount,
                     (pPdoChange->fActivated ? "activated" : "deleted"));

    for (subIndex = 1; subIndex <= pPdoChange->mappObjectCount; subIndex++)
    {
        varLen = sizeof(mappObject);
        ret = oplk_readLocalObject(pPdoChange->mappParamIndex, subIndex, &mappObject, &varLen);
        if (ret != kErrorOk)
        {
            console_printlog("  Reading 0x%X/%d failed with 0x%X\n\"%s\"\n",
                             pPdoChange->mappParamIndex, subIndex, ret, debugstr_getRetValStr(ret));
            continue;
        }
        console_printlog("  %d. mapped object 0x%X/%d\n", subIndex, mappObject & 0x00FFFFULL,
                         (mappObject & 0xFF0000ULL) >> 16);
    }
    return kErrorOk;
}

int MyoSlave::getOptions(int argc_p, char* const argv_p[], tOptions* pOpts_p){
    int opt;

    /* setup default parameters */
    strncpy(pOpts_p->devName, "\0", 128);
    pOpts_p->nodeId = NODEID;
    pOpts_p->logFormat = kEventlogFormatReadable;
    pOpts_p->logCategory = 0xffffffff;
    pOpts_p->logLevel = 0xffffffff;

    /* get command line parameters */
    while ((opt = getopt(argc_p, argv_p, "n:pv:t:d:")) != -1)
    {
        switch (opt)
        {
            case 'n':
                pOpts_p->nodeId = strtoul(optarg, NULL, 10);
                break;

            case 'd':
                strncpy(pOpts_p->devName, optarg, 128);
                break;

            case 'p':
                pOpts_p->logFormat = kEventlogFormatParsable;
                break;

            case 'v':
                pOpts_p->logLevel = strtoul(optarg, NULL, 16);
                break;

            case 't':
                pOpts_p->logCategory = strtoul(optarg, NULL, 16);
                break;

            default: /* '?' */
#if defined(CONFIG_USE_PCAP)
                printf("Usage: %s [-n NODE_ID] [-l LOGFILE] [-d DEV_NAME] [-v LOGLEVEL] [-t LOGCATEGORY] [-p]\n", argv_p[0]);
                printf(" -d DEV_NAME: Ethernet device name to use e.g. eth1\n");
#else
                printf("Usage: %s [-n NODE_ID] [-l LOGFILE] [-v LOGLEVEL] [-t LOGCATEGORY] [-p]\n", argv_p[0]);
#endif
                printf(" -p: Use parsable log format\n");
                printf(" -v LOGLEVEL: A bit mask with log levels to be printed in the event logger\n");
                printf(" -t LOGCATEGORY: A bit mask with log categories to be printed in the event logger\n");

                return -1;
        }
    }
    return 0;
}

void MyoSlave::changeControl(int motor, control_Parameters_t &params){
    int motorOffset = motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0;
	MYO_WRITE_control(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, params.control_mode);
	MYO_WRITE_reset_controller(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset);
	MYO_WRITE_Kp(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, params.Kp);
	MYO_WRITE_Kd(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, params.Kd);
	MYO_WRITE_Ki(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, params.Ki);
	MYO_WRITE_forwardGain(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, params.forwardGain);
	MYO_WRITE_deadBand(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, params.deadBand);
	MYO_WRITE_IntegralPosMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, params.IntegralPosMax);
	MYO_WRITE_IntegralNegMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, params.IntegralNegMax);
	MYO_WRITE_outputPosMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, params.outputPosMax);
	MYO_WRITE_outputNegMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, params.outputNegMax);
}

void MyoSlave::changeControl(int motor, int mode){
    int motorOffset = motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0;
    MYO_WRITE_control(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, mode);
    MYO_WRITE_reset_controller(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset);
    MYO_WRITE_Kp(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset,  control_params[motor][mode].Kp);
    MYO_WRITE_Kd(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset,  control_params[motor][mode].Kd);
    MYO_WRITE_Ki(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset,  control_params[motor][mode].Ki);
    MYO_WRITE_forwardGain(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset,  control_params[motor][mode].forwardGain);
    MYO_WRITE_deadBand(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, control_params[motor][mode].deadBand);
    MYO_WRITE_IntegralPosMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset,  control_params[motor][mode].IntegralPosMax);
    MYO_WRITE_IntegralNegMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset,  control_params[motor][mode].IntegralNegMax);
    MYO_WRITE_outputPosMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset,  control_params[motor][mode].outputPosMax);
    MYO_WRITE_outputNegMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset,  control_params[motor][mode].outputNegMax);
    MYO_WRITE_control(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, mode);
}

void MyoSlave::changeControl(int mode){
	for(uint motor=0;motor<numberOfMotors;motor++){
        int motorOffset = motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0;
		MYO_WRITE_control(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, mode);
		MYO_WRITE_reset_controller(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset);
		MYO_WRITE_Kp(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset,  control_params[motor][mode].Kp);
		MYO_WRITE_Kd(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset,  control_params[motor][mode].Kd);
		MYO_WRITE_Ki(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset,  control_params[motor][mode].Ki);
		MYO_WRITE_forwardGain(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset,  control_params[motor][mode].forwardGain);
		MYO_WRITE_deadBand(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, control_params[motor][mode].deadBand);
		MYO_WRITE_IntegralPosMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset,  control_params[motor][mode].IntegralPosMax);
		MYO_WRITE_IntegralNegMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset,  control_params[motor][mode].IntegralNegMax);
		MYO_WRITE_outputPosMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset,  control_params[motor][mode].outputPosMax);
		MYO_WRITE_outputNegMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset,  control_params[motor][mode].outputNegMax);
		MYO_WRITE_control(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, mode);
	}
}

void MyoSlave::toggleSPI(bool active){
	for(uint i=0;i<myo_base.size();i++)
		MYO_WRITE_spi_activated(myo_base[i],active);
}

void MyoSlave::reset(){
	for(uint i=0;i<myo_base.size();i++){
		MYO_WRITE_reset_myo_control(myo_base[i],true);
		MYO_WRITE_reset_myo_control(myo_base[i],false);
	}
}

void MyoSlave::setPosition(int motor, int32_t position){
    int motorOffset = motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0;
	MYO_WRITE_sp(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, position);
}

void MyoSlave::setVelocity(int motor, int32_t velocity){
    int motorOffset = motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0;
	MYO_WRITE_sp(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, velocity);
}

void MyoSlave::setDisplacement(int motor, int32_t displacement){
    int motorOffset = motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0;
	MYO_WRITE_sp(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, displacement);
}

void MyoSlave::getPIDcontrollerParams(int &Pgain, int &Igain, int &Dgain, int &forwardGain, int &deadband,
									int &setPoint, int &setPointMin, int &setPointMax, int motor){
    int motorOffset = motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0;
	Pgain = MYO_READ_Kp(myo_base[motor/MOTORS_PER_MYOCONTROL],motor-motorOffset);
	Igain = MYO_READ_Ki(myo_base[motor/MOTORS_PER_MYOCONTROL],motor-motorOffset);
	Dgain = MYO_READ_Kd(myo_base[motor/MOTORS_PER_MYOCONTROL],motor-motorOffset);
	forwardGain = MYO_READ_forwardGain(myo_base[motor/MOTORS_PER_MYOCONTROL],motor-motorOffset);
	deadband = MYO_READ_deadBand(myo_base[motor/MOTORS_PER_MYOCONTROL],motor-motorOffset);
	setPoint = MYO_READ_sp(myo_base[motor/MOTORS_PER_MYOCONTROL],motor-motorOffset);
	setPointMin = 0;
	setPointMax = 0;
}

uint16_t MyoSlave::getControlMode(int motor){
    int motorOffset = motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0;
	return MYO_READ_control(myo_base[motor/MOTORS_PER_MYOCONTROL],motor-motorOffset);
}

int16_t MyoSlave::getPWM(int motor){
    int motorOffset = motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0;
	return MYO_READ_pwmRef(myo_base[motor/MOTORS_PER_MYOCONTROL],motor-motorOffset);
}

int32_t MyoSlave::getPosition(int motor){
    int motorOffset = motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0;
	return MYO_READ_position(myo_base[motor/MOTORS_PER_MYOCONTROL],motor-motorOffset);
}

int16_t MyoSlave::getVelocity(int motor){
    int motorOffset = motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0;
	return MYO_READ_velocity(myo_base[motor/MOTORS_PER_MYOCONTROL],motor-motorOffset);
}

int16_t MyoSlave::getDisplacement(int motor){
    int motorOffset = motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0;
	return MYO_READ_displacement(myo_base[motor/MOTORS_PER_MYOCONTROL],motor-motorOffset);
}

int16_t MyoSlave::getCurrent(int motor){
    int motorOffset = motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0;
	return MYO_READ_current(myo_base[motor/MOTORS_PER_MYOCONTROL],motor-motorOffset);
}

void MyoSlave::getDefaultControlParams(control_Parameters_t *params, int control_mode){
    params->control_mode = control_mode;
    params->outputPosMax = 1000;
    params->outputNegMax = -1000;

    params->radPerEncoderCount = 2 * 3.14159265359 / (2000.0 * 53.0);

switch(control_mode){
case POSITION:
    params->spPosMax=10000000;
    params->spNegMax=-10000000;
    params->Kp=1;
    params->Ki=0;
    params->Kd=0;
    params->forwardGain=0;
    params->deadBand=0;
    params->IntegralPosMax=100;
    params->IntegralNegMax=-100;
	break;
case VELOCITY:
    params->spPosMax=100;
    params->spNegMax=-100;
    params->Kp=1;
    params->Ki=0;
    params->Kd=0;
    params->forwardGain=0;
    params->deadBand=0;
    params->IntegralPosMax=100;
    params->IntegralNegMax=-100;
	break;
case DISPLACEMENT:
    params->spPosMax=2000;
    params->spNegMax=0;
    params->Kp=1;
    params->Ki=0;
    params->Kd=0;
    params->forwardGain=0;
    params->deadBand=0;
    params->IntegralPosMax=100;
    params->IntegralNegMax=0;
	break;
default:
	cout << "unknown control mode" << endl;
	break;
}

}

void MyoSlave::allToPosition(int32_t pos){
	changeControl(POSITION);
	for(uint motor=0; motor<numberOfMotors; motor++){
        int motorOffset = motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0;
		MYO_WRITE_sp(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, pos);
	}
}

void MyoSlave::allToVelocity(int32_t vel){
	changeControl(VELOCITY);
	for(uint motor=0; motor<numberOfMotors; motor++){
        int motorOffset = motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0;
		MYO_WRITE_sp(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, vel);
	}
}

void MyoSlave::allToDisplacement(int32_t displacement){
	changeControl(DISPLACEMENT);
	for(uint motor=0; motor<numberOfMotors; motor++){
        int motorOffset = motor>=MOTORS_PER_MYOCONTROL?MOTORS_PER_MYOCONTROL:0;
		MYO_WRITE_sp(myo_base[motor/MOTORS_PER_MYOCONTROL], motor-motorOffset, displacement);
	}
}

void MyoSlave::zeroWeight(){
	weight_offset = -getWeight();
}

float MyoSlave::getWeight(){
	float weight = 0;
	uint32_t adc_value = 0;
	if(adc_base!=nullptr){
		*adc_base = 0;
		adc_value = *adc_base;
		weight = (adc_weight_parameters[0]+weight_offset+adc_weight_parameters[1]*adc_value);
	}
	return weight;
}

void MyoSlave::estimateSpringParameters(int motor, int timeout,  uint numberOfDataPoints){
	vector<float> weight, displacement, coeffs;
	setDisplacement(motor,0);
	changeControl(motor,DISPLACEMENT);
	float force_min = 0, force_max = 4.0;
	milliseconds ms_start = duration_cast< milliseconds >(system_clock::now().time_since_epoch()), ms_stop, t0, t1;
	ofstream outfile;
	outfile.open ("springParameters_calibration.csv");
	do{
		float f = (rand()/(float)RAND_MAX)*(force_max-force_min)+force_min;
		setDisplacement(motor, f);
		t0 = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
		do{// wait a bit until force is applied
			// update control
			t1 = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
		}while((t1-t0).count()<500);

		// note the weight
		weight.push_back(getWeight());
		// note the force
		displacement.push_back(getDisplacement(motor));
		outfile << displacement.back() << ", " << weight.back() << endl;
		ms_stop = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
	}while((ms_stop-ms_start).count()<timeout && weight.size()<numberOfDataPoints);
	polynomialRegression(3, displacement, weight, coeffs);
//	polyPar[motor] = coeffs;
	outfile.close();
}

void MyoSlave::polynomialRegression(int degree, vector<float> &x, vector<float> &y,
			vector<float> &coeffs){
		int N = x.size(), i, j, k;
	    double X[2*degree+1];                        //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
	    for (i=0;i<2*degree+1;i++)
	    {
	        X[i]=0;
	        for (j=0;j<N;j++)
	            X[i]=X[i]+pow(x[j],i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
	    }
	    double B[degree+1][degree+2],a[degree+1];            //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
	    for (i=0;i<=degree;i++)
	        for (j=0;j<=degree;j++)
	            B[i][j]=X[i+j];            //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
	    double Y[degree+1];                    //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^degree*yi)
	    for (i=0;i<degree+1;i++)
	    {
	        Y[i]=0;
	        for (j=0;j<N;j++)
	        Y[i]=Y[i]+pow(x[j],i)*y[j];        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
	    }
	    for (i=0;i<=degree;i++)
	        B[i][degree+1]=Y[i];                //load the values of Y as the last column of B(Normal Matrix but augmented)
	    degree=degree+1;                //degree is made degree+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations
	    for (i=0;i<degree;i++)                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
	        for (k=i+1;k<degree;k++)
	            if (B[i][i]<B[k][i])
	                for (j=0;j<=degree;j++)
	                {
	                    double temp=B[i][j];
	                    B[i][j]=B[k][j];
	                    B[k][j]=temp;
	                }

	    for (i=0;i<degree-1;i++)            //loop to perform the gauss elimination
	        for (k=i+1;k<degree;k++)
	            {
	                double t=B[k][i]/B[i][i];
	                for (j=0;j<=degree;j++)
	                    B[k][j]=B[k][j]-t*B[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
	            }
	    for (i=degree-1;i>=0;i--)                //back-substitution
	    {                        //x is an array whose values correspond to the values of x,y,z..
	        a[i]=B[i][degree];                //make the variable to be calculated equal to the rhs of the last equation
	        for (j=0;j<degree;j++)
	            if (j!=i)            //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
	                a[i]=a[i]-B[i][j]*a[j];
	        a[i]=a[i]/B[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
	    }
	    for (i=0;i<degree;i++)
	        coeffs.push_back(a[i]);	//the values of x^0,x^1,x^2,x^3,....
}
