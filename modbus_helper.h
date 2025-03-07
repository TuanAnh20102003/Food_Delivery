#ifndef MODBUS_HELPER_HPP
#define MODBUS_HELPER_HPP

#include "modbus.h"


//Since read and write register functions are used multiple times, 
// and they take integer addresses while the registers are defined as uint16_t, we need to cast the addresses to int before passing them to the modbus functions.
// Template function for each register type

template <typename T>
void write_register(modbus_t* ctx, T addr, const uint16_t value) 
{   
    modbus_write_register(ctx, static_cast<int>(addr), value);
}
template <typename T>
void write_register(modbus_t* ctx, T addr, int nb, const uint16_t* value)
{
    modbus_write_registers(ctx, static_cast<int>(addr), nb, value);
}
template <typename T>
int read_register(modbus_t* ctx, T addr, int nb, uint16_t* dest)
{
    int read_flag = modbus_read_registers(ctx, static_cast<int>(addr), nb, dest);
    return read_flag;
}
#endif // MODBUS_HELPER_HPP
