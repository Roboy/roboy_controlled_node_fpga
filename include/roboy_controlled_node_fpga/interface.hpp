#pragma once

#include <ncurses.h>
#include <vector>
#include <cstring>
#include <unistd.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <limits.h>
#include "hwlib.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include "socal/alt_gpio.h"
#include "roboy_controlled_node_fpga/hps_0.h"
#include "roboy_controlled_node_fpga/am4096.hpp"
#include "roboy_controlled_node_fpga/interface.hpp"
#include "roboy_controlled_node_fpga/myoControl.hpp"


#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

//#define ADC_MEASUREMENT

using namespace std;

class Interface {
public:
    Interface(vector<int32_t*> &myo_base);
    Interface(vector<int32_t*> &myo_base, vector<int32_t*> &i2c_base, vector<int> &deviceIds);

    ~Interface();

    void printMessage(uint row, uint col, char *msg);

    void printMessage(uint row, uint col, char *msg, uint color);

    void print(uint row, uint startcol, uint length, const char *s);

    void clearAll(uint row);

    void querySensoryData();

    void processing(char *msg1, char *what, char *msg2);

    void processing(char *msg1, char *msg2);

    void toggleSPI();

    void reset();

    void setGains();

    void positionControl();

    void velocityControl();

    void displacementControl();

    void switchMotor();

    void zeroWeight();

    void setAllTo();

    void estimateSpringParameters();

    void recordTrajectories();

    void playTrajectories();

    MyoControl *myoControl;
    vector<boost::shared_ptr<AM4096>> jointAngle;

    uint timeout_ms = 10;
private:
    uint rows, cols;
    int32_t pos;
    uint ganglion_id = 0;
    uint motor_id = 0;
    char inputstring[30];
};

int main(int argc, char* argv[]);