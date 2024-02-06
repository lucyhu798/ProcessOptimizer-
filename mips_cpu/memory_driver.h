#ifndef __INC__MEMORY_DRIVER_H__
#define __INC__MEMORY_DRIVER_H__

#include <cstdint>

#include "Vmips_core.h"
#include "memory.h"

class MemoryDriver
{
public:
    MemoryDriver(Vmips_core *const dut, Memory *const mem) : dut(dut), mem(mem) {}
    void drive_reset() const;
    void drive(uint64_t time);
    void consume(uint64_t time);

private:
    Vmips_core *dut;
    Memory *mem;

    void drive_write_address(uint64_t time);
    void drive_write_data(uint64_t time);
    void drive_write_response(uint64_t time);
    void drive_read_address(uint64_t time);
    void drive_read_data(uint64_t time);
    void consume_write_address(uint64_t time);
    void consume_write_data(uint64_t time);
    void consume_write_response(uint64_t time);
    void consume_read_address(uint64_t time);
    void consume_read_data(uint64_t time);
};

#endif
