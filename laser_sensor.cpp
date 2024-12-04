#include "laser_sensor.h"



std::vector <uint16_t> Laser::modbusFailReadHandle(uint16_t addr, uint16_t nb)
{
	std::vector <uint16_t> reg(nb, 0);
	int rc;
	while (1)
	{
		rc = modbus_read_registers(ctx3, addr, nb, reg.data());
		if (rc == -1)
		{
			std::cerr << "Error reading register: " << modbus_strerror(errno) << std::endl;

		}
		else break;
	}
	return reg;
}
Laser::Laser(const std::string& port)
{
	ctx3 = modbus_new_rtu(port.c_str(), 115200, 'N', 8, 1);
	if (ctx3 == nullptr) {
		throw std::runtime_error("Unable to allocate modbus context");
	}
	modbus_set_slave(ctx3, 1);  // Set slave ID to 2 (it's just a function which help the client know which device it is talking to, not set the id of this device)

	// Connect to the Modbus device
	int connection = modbus_connect(ctx3);
	if (connection == -1) {
		throw std::runtime_error("Connection failed");
	}
	if (connection == 0)
	{
		std::cout << "Connection successful" << std::endl;
	}
}
Laser::~Laser()
{
	modbus_close(ctx3);
	modbus_free(ctx3);
}
std::vector <uint16_t> Laser::get_probe_output()
{
	std::vector <uint16_t> values = modbusFailReadHandle(PROBE_1, 7);
	return values;
}
void Laser::set_lase_sensor_id(int id)
{
	modbus_write_register(ctx3, LASER_ID_ADDRESS, (uint16_t) id);
	uint16_t id_re;
	/*int rc = modbus_read_registers(ctx3, LASER_ID_ADDRESS, 1, &id_re);
	if (rc == -1) {
		std::cerr << "Error reading LASER_ID_ADDRESS register: " << modbus_strerror(errno) << std::endl;
		throw std::runtime_error("Failed to read LASER_ID_ADDRESS register");
	}
	else std::cout << "ID of laser sensor is: " << id_re << std::endl;*/
}
int Laser::get_lase_sensor_id()
{
	uint16_t id_re;
	int rc = modbus_read_registers(ctx3, LASER_ID_ADDRESS, 1, &id_re);
	if (rc == -1) {
		std::cerr << "Error reading LASER_ID_ADDRESS register: " << modbus_strerror(errno) << std::endl;
		throw std::runtime_error("Failed to read LASER_ID_ADDRESS register");
	}
	else std::cout << "ID of laser sensor iss: " << id_re << std::endl;
	return id_re;
}
void Laser::set_baud_rate(int baud)
{
	if (baud_rate_map.find(baud) == baud_rate_map.end())
	{
		std::cerr << "Invalid baud rate" << std::endl;
		return;
	}
	modbus_write_register(ctx3, BAUD_RATE, baud_rate_map[baud]);
}
int Laser::get_baud_rate()
{
	uint16_t baud_re;
	int rc = modbus_read_registers(ctx3, BAUD_RATE, 1, &baud_re);
	if (rc == -1) {
		std::cerr << "Error reading BAUD_RATE register: " << modbus_strerror(errno) << std::endl;
		throw std::runtime_error("Failed to read BAUD_RATE register");
	}
	else std::cout << "Baud rate of laser sensor is: " << baud_re << std::endl;
	return baud_re;
}
