#include "mag_line.h"

std::vector <uint16_t> Magnetic::modbusFailReadHandle(uint16_t addr, uint16_t nb)
{
	std::vector <uint16_t> reg(nb, 0);
	int rc;
	while (1)
	{
		rc = modbus_read_registers(ctx2, addr, nb, reg.data());
		if (rc == -1)
		{
			std::cerr << "Error reading register: " << modbus_strerror(errno) << std::endl;

		}
		else break;
	}
	return reg;
}
Magnetic::Magnetic(const std::string& port)
{
	ctx2 = modbus_new_rtu(port.c_str(), 115200, 'N', 8, 1);
	if (ctx2 == nullptr) {
		throw std::runtime_error("Unable to allocate modbus context");
	}
	modbus_set_slave(ctx2, 2);  // Set slave ID to 2

	// Connect to the Modbus device
	int connection = modbus_connect(ctx2);
	if (connection == -1) {
		throw std::runtime_error("Connection failed");
	}
	if (connection == 0)
	{
		std::cout << "Connection successful" << std::endl;
	}
}
Magnetic::~Magnetic()
{
	modbus_close(ctx2);
	modbus_free(ctx2);
}


uint16_t Magnetic::get_digital_output()
{
	std::vector <uint16_t> values = modbusFailReadHandle(SWITCH_OUTPUT_16_CHANNELS, 1);
	return (uint16_t) values[0];
}

std::vector <uint8_t> Magnetic::get_analog_output()
{
	std::vector <uint8_t> reg(16, 0);
	std::vector <uint16_t> values = modbusFailReadHandle(HIGH_2_LOW_1, 8);
	for (size_t i = 0; i < values.size(); ++i)
	{
		reg[2 * i + 1] = static_cast<uint8_t>((values[i] & 0xFF00) >> 8);
		reg[2 * i] = static_cast<uint8_t>(values[i] & 0x00FF);
	}
	return reg;
}
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
		return 0;
	}
	else
	{
		double position = (float) weight / (float) sum;
		return position;
	}
}