#include "mag_line.h"
#include <windows.h>
#include "modbus_helper.h"
//Handle the case when the modbus read fails
std::vector <uint16_t> Magnetic::modbusFailReadHandle(uint16_t addr, uint16_t nb)
{
	std::vector <uint16_t> reg(nb, 0);
	int rc;
	while (1)
	{
		rc = read_register(ctx2, addr, nb, reg.data());
		if (rc == -1)
		{
			std::cerr << "Error reading register of magnetic sensor: " << modbus_strerror(errno) << std::endl;
			Sleep(100);
		}
		else break;
	}
	return reg;
}
//Constructor
Magnetic::Magnetic(const std::string& port)
{
	ctx2 = modbus_new_rtu(port.c_str(), 115200, 'N', 8, 1);
	if (ctx2 == nullptr) {
		throw std::runtime_error("Unable to allocate modbus context");
	}
	int slave_flag = modbus_set_slave(ctx2, 2);  // Receive data from the device of ID 2
	if (slave_flag == -1) {
		std::cerr << "Error setting slave ID: " << modbus_strerror(errno) << std::endl;
		throw std::runtime_error("Failed to set slave ID");
	}
	// Connect to the Modbus device
	int connection = modbus_connect(ctx2);
	if (connection == -1) {
		throw std::runtime_error("Connection failed");
		modbus_free(ctx2);
	}
	if (connection == 0)
	{
		std::cout << "Connection successful" << std::endl;
	}
}
//Destructor
Magnetic::~Magnetic()
{
	modbus_close(ctx2);
	modbus_free(ctx2);
	std::cout << "Magnetic sensor destructor called." << std::endl;
}

//get the digital output of the magnetic line sensor
uint16_t Magnetic::get_digital_output()
{
	std::vector <uint16_t> values = modbusFailReadHandle(SWITCH_OUTPUT_16_CHANNELS, 1);
	return values[0];
}
//Get the analog output of the magnetic line sensor
std::vector <uint8_t> Magnetic::get_analog_output()
{
	std::vector <uint8_t> reg(16, 0);
	std::vector <uint16_t> values = modbusFailReadHandle(HIGH_2_LOW_1, 8);
	
	for (size_t i = 0; i < values.size(); ++i)
	{
		reg[2 * i + 1] = static_cast<uint8_t>((values[i] & 0xFF00) >> 8);
		reg[2 * i] = static_cast<uint8_t>(values[i] & 0x00FF);
	}
	for (size_t i = 0; i < reg.size(); ++i) //if the value is less than 5, set it to 0
	{
		if (reg[i] <= 5)
		{
			reg[i] = 0;
		}
	}
	return reg;
}
//Set the ID of the magnetic line sensor
void Magnetic::set_id(int id)
{
	modbus_write_register(ctx2, MAG_ID_ADDRESS, (uint16_t) id);
	uint16_t id_re;
	int rc = modbus_read_registers(ctx2, MAG_ID_ADDRESS, 1, &id_re);
	if (rc == -1) {
		std::cerr << "Error reading MAG_ID_ADDRESS register: " << modbus_strerror(errno) << std::endl;
		throw std::runtime_error("Failed to read MAG_ID_ADDRESS register");
	}
	else std::cout << "ID of magnetic line sensor is: " << id_re << std::endl;
}
double Magnetic::get_position_value() //get the position of the line detected (at which probe)
{

	std::vector <uint8_t> values = get_analog_output();
	int weight = 0;
	for (int i = 0; i < values.size(); i++)
	{
		weight += i * values[i];
	}
	int sum = 0;
	for (int i = 0; i < values.size(); i++)
	{
		sum += values[i];
	}
	if (sum == 0) //no line detected
	{
		return 100;
	}
	else
	{
		double position = static_cast<float> (weight) / static_cast<float> (sum);
		return position;
	}
}
