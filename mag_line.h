#ifndef MAG
#define MAG
#include <iostream>
#include "modbus.h"
#include <cstdint>
#include <vector>


class Magnetic 
{
	private:
		modbus_t* ctx2;
		/*
		Register Address
		*/
		// Analog Output/Detection value - 16 channels/16 bytes - 8 values - High byte: EVEN, Low byte: ODD
		uint16_t HIGH_2_LOW_1 = 0x20;
		uint16_t HIGH_4_LOW_3 = 0x21;
		uint16_t HIGH_6_LOW_5 = 0x22;
		uint16_t HIGH_8_LOW_7 = 0x23;
		uint16_t HIGH_10_LOW_9 = 0x24;
		uint16_t HIGH_12_LOW_11 = 0x25;
		uint16_t HIGH_14_LOW_13 = 0x26;
		uint16_t HIGH_16_LOW_15 = 0x27;
		
		// Digital Output/Detection value - 16 channels/2 bytes - 1 value
		uint16_t SWITCH_OUTPUT_16_CHANNELS = 0x28;

		// RS-232 and RS-485 MODBUS-based device ID (R/W)
		uint16_t MAG_ID_ADDRESS = 0x33;
	public:
		std::vector <uint16_t> modbusFailReadHandle(uint16_t addr, uint16_t nb);
		Magnetic();
		Magnetic(const std::string& port);
		~Magnetic();
		uint16_t get_digital_output();
		std::vector <uint8_t> get_analog_output();
		void set_id(int id);
		double get_position_value();


};
#endif // !MAG
