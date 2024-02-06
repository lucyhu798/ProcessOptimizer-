#include <iostream>
#include <fstream>

#include "memory.h"

extern int memory_debug;

Memory::Memory(const char *const hex_file, double delay_factor) : delay_factor(delay_factor)
{
    std::ifstream f(hex_file);
    if (!f.is_open())
    {
        std::cerr << "Failed to open file: " << hex_file << std::endl;
        exit(-1);
    }

    uint addr = 0;
    uint32_t data;
    if (memory_debug >= 3)
        std::cout << std::hex << std::showbase;
    while (f >> std::hex >> data)
    {
        if (memory_debug >= 3)
            std::cout << "Preload addr=" << addr << " data=" << data << std::endl;
        m[addr++] = data;
    }
    if (memory_debug >= 3)
        std::cout << std::noshowbase;

    f.close();
}

void Memory::process(uint64_t time)
{
    process_pipe();
    process_read(time);
    process_write(time);
}

void Memory::process_pipe()
{
    if (write_address_pipe != NULL && full_write_address() == PUSH_OK)
    {
        write_address[write_address_pipe->awid].push(*write_address_pipe);
        delete write_address_pipe;
        write_address_pipe = NULL;
    }
    if (write_data_pipe != NULL && full_write_data() == PUSH_OK)
    {
        write_data[write_data_pipe->wid].push(*write_data_pipe);
        delete write_data_pipe;
        write_data_pipe = NULL;
    }
    if (read_address_pipe != NULL && full_read_address() == PUSH_OK)
    {
        read_address[read_address_pipe->arid].push(*read_address_pipe);
        delete read_address_pipe;
        read_address_pipe = NULL;
    }
}

bool Memory::full_write_address() const
{
    if (write_address_pipe == NULL)
        return PUSH_OK;

    if (write_address[write_address_pipe->awid].size() >= AXI_WRITE_ADDR_MAX_PENDING_PER_ID)
        return PUSH_FULL;

    int size = 0;
    for (int i = 0; i < AXI_ID_COUNT; i++)
        size += write_address[i].size();
    if (size >= AXI_WRITE_ADDR_MAX_PENDING)
        return PUSH_FULL;

    return PUSH_OK;
}
void Memory::push_write_address(const AxiWriteAddress &pkt)
{
    write_address_pipe = new AxiWriteAddress(pkt);
}

bool Memory::full_write_data() const
{
    if (write_data_pipe == NULL)
        return PUSH_OK;

    if (write_data[write_data_pipe->wid].size() >= AXI_WRITE_DATA_MAX_PENDING_PER_ID)
        return PUSH_FULL;

    int size = 0;
    for (int i = 0; i < AXI_ID_COUNT; i++)
        size += write_data[i].size();
    if (size >= AXI_WRITE_DATA_MAX_PENDING)
        return PUSH_FULL;

    return PUSH_OK;
}
void Memory::push_write_data(const AxiWriteData &pkt)
{
    write_data_pipe = new AxiWriteData(pkt);
}

bool Memory::full_read_address() const
{
    if (read_address_pipe == NULL)
        return PUSH_OK;

    if (read_address[read_address_pipe->arid].size() >= AXI_READ_ADDR_MAX_PENDING_PER_ID)
        return PUSH_FULL;

    int size = 0;
    for (int i = 0; i < AXI_ID_COUNT; i++)
        size += read_address[i].size();
    if (size >= AXI_READ_ADDR_MAX_PENDING)
        return PUSH_FULL;

    return PUSH_OK;
}
void Memory::push_read_address(const AxiReadAddress &pkt)
{
    read_address_pipe = new AxiReadAddress(pkt);
}

const AxiWriteResponse *const Memory::peek_write_response() const
{
    if (!write_response.empty())
        return &write_response.front();
    return NULL;
}
const AxiReadData *const Memory::peek_read_data() const
{
    if (!read_data.empty())
        return &read_data.front();
    return NULL;
}

void Memory::pop_write_response()
{
    auto pkt = write_response.front();
    write_address[pkt.bid].pop();
    write_response.pop();
}
void Memory::pop_read_data()
{
    auto pkt = read_data.front();
    if (pkt.rlast)
        read_address[pkt.rid].pop();
    read_data.pop();
}

void Memory::process_read(uint64_t time)
{
    for (int i = 0; i < AXI_ID_COUNT; i++)
    {
        if (read_address[i].empty())
            continue;
        auto &pkt = read_address[i].front();

        // Skip if already committed
        if (pkt.committed)
            continue;

        // Delay by 100 cycles
        if (pkt.time_start + 1000 * delay_factor > time)
            continue;

        commit_read(pkt, time);
    }
}
void Memory::process_write(uint64_t time)
{
    for (int i = 0; i < AXI_ID_COUNT; i++)
    {
        if (write_address[i].empty())
            continue;
        auto &pkt = write_address[i].front();

        // Skip if already committed
        if (pkt.committed)
            continue;

        // Skip if not all data are received
        if (write_data[i].size() < pkt.awlen)
            continue;

        // Delay by 120 cycles
        if (pkt.time_start + 1200 * delay_factor > time)
            continue;

        commit_write(pkt, time);
    }
}

void Memory::commit_read(AxiReadAddress &pkt, uint64_t time)
{
    if (memory_debug)
        std::cout << "[" << std::dec << time << "] Commit memory READ\n  " << pkt << "  data=[ "
                  << std::hex << std::showbase;
    for (int i = 0; i < pkt.arlen; i++)
    {
        auto data = m[(pkt.araddr >> 2) + i];
        read_data.push(AxiReadData{pkt.arid, i == pkt.arlen - 1, data});
        if (memory_debug)
            std::cout << data << " ";
    }
    if (memory_debug)
        std::cout << "]\n"
                  << std::noshowbase;
    pkt.committed = true;
}

void Memory::commit_write(AxiWriteAddress &pkt, uint64_t time)
{
    if (memory_debug)
        std::cout << "[" << std::dec << time << "] Commit memory WRITE\n  " << pkt << "  data=[ "
                  << std::hex << std::showbase;
    for (int i = 0; i < pkt.awlen; i++)
    {
        auto &data = write_data[pkt.awid].front();
        m[(pkt.awaddr >> 2) + i] = data.wdata;
        if (memory_debug)
            std::cout << data.wdata << " ";
        write_data[pkt.awid].pop();
    }
    if (memory_debug)
        std::cout << "]\n"
                  << std::noshowbase;

    write_response.push(AxiWriteResponse{pkt.awid});
    pkt.committed = true;
}