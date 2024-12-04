// Project3.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include "modbus.h"
#include "controller.h"
#include "mag_line.h"
#include "laser_sensor.h"

double Kp = 0.5852, Ki = 0.038, Kd = 6.3;

bool pid_line_sensor(Magnetic mag, Controller vehicle, double prevError, double target, uint16_t* cmds)
{
	double error = 0, d_value = 0, i_value = 0; /*error_data = 0*/
	double position = mag.get_position_value();
	if (position == 0)
	{
		vehicle.disable_motor();
		std::cout << "No line detected" << std::endl;
		return 0;
	}
	error = position - target;
	d_value = error - prevError;
	i_value += error;
	double correction = Kp * error + Kd * d_value + Ki * i_value;
	uint16_t right_rpm = cmds[0], left_rpm = cmds[1];
	if (error < 0)
	{
		right_rpm = (uint16_t)(cmds[0] - correction);
	}
	else
	{
		left_rpm = (uint16_t)(cmds[1] - correction);
	}
	prevError = error;
	vehicle.set_rpm(right_rpm, left_rpm);
	Sleep(1500);
	return 1;
	
}

int main()
{
	float target = 7.5;
    
	Controller vehicle("COM8");
	
	Magnetic mag("COM7");
	vehicle.disable_motor();
	vehicle.set_accel_time(1000, 1000);
	vehicle.set_decel_time(1000, 1000);
	vehicle.set_maxRPM_pos(60,60);
	vehicle.set_mode(3);
	
	vehicle.enable_motor();
	uint16_t cmds[2] = { 30, -30 };
	vehicle.set_rpm(cmds[0], cmds[1]); //left, right
	Sleep(1500);
	float prevError = mag.get_position_value() - target;
	//vehicle.stop();
	//int a = vehicle.get_mode();
	//std::cout << a << std::endl;

	while (1)
	{
		bool flag = pid_line_sensor(mag, vehicle, prevError, target, cmds);
		if (flag)
		{
			continue;
		}
		else break;
	}
	
	/*laser.get_lase_sensor_id();
	
	while(1)
	{
		std::vector <uint16_t> values = laser.get_probe_output();
		for (size_t i = 0; i < values.size(); i++)
		{
			std::cout << " " << values[i] ;
		}
		std::cout << std::endl;
		Sleep(500);
	}*/
	
	
	
}

