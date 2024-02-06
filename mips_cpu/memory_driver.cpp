#include "memory_driver.h"
#include <csignal>

extern int memory_debug;

void MemoryDriver::drive_reset() const
{
    dut->AWREADY = 0;
    dut->WREADY = 0;
    dut->BVALID = 0;
    dut->ARREADY = 0;
    dut->RVALID = 0;
}

void MemoryDriver::drive(uint64_t time)
{
    drive_write_address(time);
    drive_write_data(time);
    drive_write_response(time);
    drive_read_address(time);
    drive_read_data(time);
}

void MemoryDriver::consume(uint64_t time)
{
    consume_write_address(time);
    consume_write_data(time);
    consume_write_response(time);
    consume_read_address(time);
    consume_read_data(time);
}

void MemoryDriver::drive_write_address(uint64_t time)
{
    dut->AWREADY = mem->full_write_address() == PUSH_OK;
}
void MemoryDriver::consume_write_address(uint64_t time)
{
    if (dut->AWREADY && dut->AWVALID)
    {
        AxiWriteAddress pkt{dut->AWID, dut->AWLEN, dut->AWADDR, time, false};
        mem->push_write_address(pkt);
        if (memory_debug >= 2)
            std::cout << "[" << std::dec << time << "] Push " << pkt;
    }
}
void MemoryDriver::drive_write_data(uint64_t time)
{
    dut->WREADY = mem->full_write_data() == PUSH_OK;
}
void MemoryDriver::consume_write_data(uint64_t time)
{
    if (dut->WREADY && dut->WVALID)
    {
        AxiWriteData pkt{dut->WID, dut->WLAST, dut->WDATA, time};
        mem->push_write_data(pkt);
        if (memory_debug >= 2)
            std::cout << "[" << std::dec << time << "] Push " << pkt;
    }
}
void MemoryDriver::drive_write_response(uint64_t time)
{
    auto pkt = mem->peek_write_response();
    dut->BVALID = 0;
    if (pkt != NULL)
    {
        dut->BVALID = 1;
        dut->BID = pkt->bid;
    }
}
void MemoryDriver::consume_write_response(uint64_t time)
{
    if (dut->BVALID && dut->BREADY)
    {
        if (memory_debug >= 2)
        {
            auto pkt = mem->peek_write_response();
            std::cout << "[" << std::dec << time << "] Pop  " << *pkt;
        }
        mem->pop_write_response();
    }
}
void MemoryDriver::drive_read_address(uint64_t time)
{
    dut->ARREADY = mem->full_read_address() == PUSH_OK;
}
void MemoryDriver::consume_read_address(uint64_t time)
{
    if (dut->ARREADY && dut->ARVALID)
    {
        AxiReadAddress pkt{dut->ARID, dut->ARLEN, dut->ARADDR, time, false};
        mem->push_read_address(pkt);
        if (memory_debug >= 2)
            std::cout << "[" << std::dec << time << "] Push " << pkt;
    }
}
void MemoryDriver::drive_read_data(uint64_t time)
{
    auto pkt = mem->peek_read_data();
    dut->RVALID = 0;
    if (pkt != NULL)
    {
        dut->RVALID = 1;
        dut->RID = pkt->rid;
        dut->RLAST = pkt->rlast;
        dut->RDATA = pkt->rdata;
    }
}
void MemoryDriver::consume_read_data(uint64_t time)
{
    if (dut->RVALID && dut->RREADY)
    {
        if (memory_debug >= 2)
        {
            auto pkt = mem->peek_read_data();
            std::cout << "[" << std::dec << time << "] Pop  " << *pkt;
        }
        mem->pop_read_data();
    }
}