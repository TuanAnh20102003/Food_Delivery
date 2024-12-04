#ifndef LASE
#define LASE

#include <iostream>
#include "modbus.h"
#include <cstdint>
#include <vector>
#include <map>

class Laser
{
	private:
		modbus_t* ctx3;
		/*
		Register Address
		*/
		// Device Address
		uint16_t LASER_ID_ADDRESS = 0x07;
		//Baud Rate Setting Address
		uint16_t BAUD_RATE = 0x08;
		// Analog Output/Detection value
		uint16_t PROBE_1 = 0x00; //Starting address (0x00 -> 0x06)
		//Baud rate map
		std::map<int, int> baud_rate_map = { {2400, 0x0001}, {4800, 0x0002}, {9600, 0x0003}, {19200, 0x0004}, {38400, 0x0005}, {57600, 0x0006}, {115200, 0x0007} };
		
	public:
		std::vector <uint16_t> modbusFailReadHandle(uint16_t addr, uint16_t nb);
		Laser();
		Laser(const std::string& port);
		~Laser();
		std::vector <uint16_t> get_probe_output();
		void set_lase_sensor_id(int id);
		int get_lase_sensor_id();
		void set_baud_rate(int baud);
		int get_baud_rate();
		
		

};

#endif // !LASE