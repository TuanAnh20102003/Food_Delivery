#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <iostream>
#include "modbus.h"
#include <vector>
#include <cstdint>
#include <cmath>
#include <stdexcept>
#include <cstdint>
#include <algorithm>


class Controller
{
    private:
        modbus_t *ctx;
    std::vector<uint16_t> FAULT_LIST;
    /*
        *Register Address
        */

        // Initialize register addresses
        uint16_t CONTROL_REG = 0x200E;
        uint16_t OPR_MODE = 0x200D;
        uint16_t L_ACL_TIME = 0x2080;
        uint16_t R_ACL_TIME = 0x2081;
        uint16_t L_DCL_TIME = 0x2082;
        uint16_t R_DCL_TIME = 0x2083;

        //Velocity control
        uint16_t L_CMD_RPM = 0x2088;
        uint16_t R_CMD_RPM = 0x2089;
        uint16_t L_FB_RPM = 0x20AB;
        uint16_t R_FB_RPM = 0x20AC;

        //Position control
        uint16_t POS_CONTROL_TYPE = 0x200F;

        uint16_t L_MAX_RPM_POS = 0x208E;
        uint16_t R_MAX_RPM_POS = 0x208F;

        uint16_t L_CMD_REL_POS_HI = 0x208A;
        uint16_t L_CMD_REL_POS_LO = 0x208B;
        uint16_t R_CMD_REL_POS_HI = 0x208C;
        uint16_t R_CMD_REL_POS_LO = 0x208D;

        uint16_t L_FB_POS_HI = 0x20A7;
        uint16_t L_FB_POS_LO = 0x20A8;
        uint16_t R_FB_POS_HI = 0x20A9;
        uint16_t R_FB_POS_LO = 0x20AA;

        //Troubleshooting
        uint16_t L_FAULT = 0x20A5;
        uint16_t R_FAULT = 0x20A6;

        /*
        *Control commands
        */
        uint16_t EMER_STOP = 0x05;
        uint16_t ALRM_CLR = 0x06;
        uint16_t DOWN_TIME = 0x07;
        uint16_t ENABLE = 0x08;
        uint16_t POS_SYNC = 0x10;
        uint16_t POS_L_START = 0x11;
        uint16_t POS_R_START = 0x12;

        /*
        *Operation modes
        */
        uint16_t POS_REL_CONTROL = 1;
        uint16_t POS_ABS_CONTROL = 2;
        uint16_t VEL_CONTROL = 3;

        /*
        *Fault codes
        */
        uint16_t NO_FAULT = 0x0000;
        uint16_t OVER_VOLT = 0x0001;
        uint16_t UNDER_VOLT = 0x0002;
        uint16_t OVER_CURR = 0x0004;
        uint16_t OVER_LOAD = 0x0008;
        uint16_t CURR_OUT_TOL = 0x0010;
        uint16_t ENCOD_OUT_TOL = 0x0020;
        uint16_t MOTOR_BAD = 0x0040;
        uint16_t REF_VOLT_ERROR = 0x0080;
        uint16_t EEPROM_ERROR = 0x0100;
        uint16_t WALL_ERROR = 0x0200;
        uint16_t HIGH_TEMP = 0x0400;

        // Wheel parameters
        
    public:
        Controller() {};
        Controller(const std::string& port);
        ~Controller();
        void set_mode(int mode);
        int get_mode();
        void enable_motor();
        void disable_motor();
        std::pair<bool, uint16_t> get_fault_code();
        void clear_alarm();
        void set_accel_time(int L_ms, int R_ms);
        void set_decel_time(int L_ms, int R_ms);
        void set_rpm(int16_t L_rpm, int16_t R_rpm);
        std::pair<double, double> get_rpm();
        std::pair<double, double> get_linear_velocities();
        void set_maxRPM_pos(int max_L_rpm, int max_R_rpm);
        void set_relative_angle(int32_t ang_L, int32_t ang_R);
        std::pair<double, double> get_wheels_travelled();
		void stop();

};

#endif // CONTROLLER_H